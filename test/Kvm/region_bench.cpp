// SPDX-License-Identifier: GPL-3.0-only
// Translated-region experiment: same DSP/x64 code, no whole-emulator claim.
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "guest.hpp"
#include "region_execution.hpp"
#include "region_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace dsp56300::test::kvm::region {
namespace {
using Clock = std::chrono::steady_clock;
constexpr unsigned warmups = 16, trials = 9;
constexpr unsigned minimum_replays = 16, maximum_replays = 4096;
constexpr unsigned trial_instructions = 1U << 20;
constexpr std::array iteration_counts{1U, 4U, 16U, 64U, 256U, 1024U, 4096U, 16384U, 65536U};
constexpr std::array verification_counts{1U, 7U, 257U, 65535U};
constexpr int skipped = 77;

template <RegionExecutor Runtime> class Replay {
public:
  template <typename... Arguments>
  Replay(const Shape shape, const std::span<const std::uint8_t> code, Arguments &&...arguments)
      : runtime_(code, std::forward<Arguments>(arguments)...),
        seed_(region::cpu(shape)),
        expected_(region::expected(shape)),
        count_(region::instructions(shape)) {}

  void prepare() {
    runtime_.cpu() = seed_;
    runtime_.context() = {};
    runtime_.context().budget = count_;
  }

  void execute() {
    require(runtime_.enter() == count_, "Translated region returns the exact DSP retirement count");
  }

  void validate() {
    const auto &cpu = runtime_.cpu();
    auto state = cpu.reg;
    state.a >>= detail::padding_bits;
    state.b >>= detail::padding_bits;
    require(state == expected_, "All registers and loop frames equal the independent region model");
    require(cpu.instructions == seed_.instructions + count_ &&
                cpu.cycles == seed_.cycles + count_ && cpu.repeat_remaining == 0 &&
                cpu.repeat_pc == 0 && cpu.repeat_lc == 0 && cpu.repeat_opcode == 0 &&
                !cpu.repeat_loaded && !cpu.repeat_completed && !cpu.interrupt_inhibited &&
                cpu.processing == ProcessingState::running && cpu.debug_flags == 0,
            "Region preserves exact progress and hidden execution boundaries");
  }

  double trial(const unsigned replays) {
    double elapsed = 0;
    for (unsigned replay = 0; replay < replays; ++replay) {
      prepare();
      const auto before = Clock::now();
      execute();
      const auto after = Clock::now();
      elapsed += std::chrono::duration<double, std::nano>(after - before).count();
    }
    validate();
    return elapsed / replays;
  }

  [[nodiscard]] Runtime &runtime() noexcept { return runtime_; }

private:
  Runtime runtime_;
  detail::Cpu seed_;
  Registers expected_;
  Word count_;
};

template <RegionExecutor Runtime> void warm(Replay<Runtime> &replay) {
  for (unsigned iteration = 0; iteration < warmups; ++iteration) {
    replay.prepare();
    replay.execute();
  }
  replay.validate();
}

void verify_case(const Shape shape) {
  const auto code = compile(shape);
  Replay<DirectRegion> direct(shape, code);
  Replay<ProtectedRegion> ioctls(shape, code, RegisterTransfer::ioctls);
  Replay<ProtectedRegion> automatic(shape, code, RegisterTransfer::automatic);
  // Reuse the same installed bytes across multiple state resets and entries.
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    direct.prepare();
    direct.execute();
    direct.validate();
    ioctls.prepare();
    ioctls.execute();
    ioctls.validate();
    automatic.prepare();
    automatic.execute();
    automatic.validate();
  }
}

void verify() {
  for (const auto mode : modes) {
    for (const auto body : {Word{1}, static_cast<Word>(chain.size())}) {
      for (const auto iterations : verification_counts) {
        verify_case({.body = body, .iterations = iterations, .mode = mode});
      }
    }
  }
  std::cout << checks << " direct/KVM prepared-region checks passed\n";
}

void report(const std::string_view name,
            const Shape shape,
            std::array<double, trials> times,
            const std::size_t bytes,
            const unsigned replays) {
  std::ranges::sort(times);
  const auto median = times[trials / 2];
  std::cout << name << " body=" << shape.body << " iterations=" << shape.iterations
            << " retired=" << instructions(shape) << " code_bytes=" << bytes
            << " replays=" << replays << " median_ns=" << median
            << " ns_per_instruction=" << median / instructions(shape)
            << " range_ns=" << times.front() << ".." << times.back() << '\n';
}

void benchmark(const Shape shape) {
  const auto code = compile(shape);
  Replay<DirectRegion> direct(shape, code);
  Replay<ProtectedRegion> ioctls(shape, code, RegisterTransfer::ioctls);
  Replay<ProtectedRegion> automatic(shape, code, RegisterTransfer::automatic);
  const auto replays =
      std::clamp(trial_instructions / instructions(shape), minimum_replays, maximum_replays);
  warm(direct);
  warm(ioctls);
  warm(automatic);
  std::array<std::array<double, trials>, 3> timings{};
  for (unsigned trial = 0; trial < trials; ++trial) {
    // Alternate order to reduce systematic frequency/thermal drift.
    if (trial % 2 == 0) {
      timings[0][trial] = direct.trial(replays);
      timings[1][trial] = ioctls.trial(replays);
      timings[2][trial] = automatic.trial(replays);
    } else {
      timings[2][trial] = automatic.trial(replays);
      timings[1][trial] = ioctls.trial(replays);
      timings[0][trial] = direct.trial(replays);
    }
    for (std::size_t index = 0; index < timings.size(); ++index) {
      std::cout << "trial body=" << shape.body << " iterations=" << shape.iterations
                << " path=" << index << " index=" << trial << " ns=" << timings[index][trial]
                << '\n';
    }
  }
  report("direct", shape, timings[0], code.size(), replays);
  report("kvm-ioctls", shape, timings[1], code.size(), replays);
  report(automatic.runtime().shared_registers() ? "kvm-shared" : "kvm-auto-ioctls",
         shape,
         timings[2],
         code.size(),
         replays);
}

void run(const int argc, const char *const *const argv) {
  if (argc != 2) { throw std::invalid_argument("Usage: dsp56300_kvm_bench --verify|--bench"); }
  if (std::string_view(argv[1]) == "--verify") {
    verify();
    return;
  }
  if (std::string_view(argv[1]) != "--bench") { throw std::invalid_argument("Invalid argument"); }
  std::cout
      << std::fixed << std::setprecision(2)
      << "One DSP region per runtime, identical generated bytes, full register checks.\n"
      << "Own arithmetic loops only; no public VM backend, Blofeld or device measurement.\n"
      << "16 warmups, 9 alternating trials; median and range, no overhead subtraction.\n"
      << "Compilation, VM setup, code installation and CPU restoration excluded.\n"
      << "Execution includes timers, retirement checks, KVM register exchange and HLT exit.\n";
  for (const auto body : {Word{1}, static_cast<Word>(chain.size())}) {
    for (const auto iterations : iteration_counts) {
      benchmark({.body = body, .iterations = iterations, .mode = mode_rm});
    }
  }
}
} // namespace
} // namespace dsp56300::test::kvm::region

int main(const int argc, const char *const *const argv) {
  try {
    dsp56300::test::kvm::region::run(argc, argv);
  } catch (const dsp56300::test::kvm::Unavailable &error) {
    std::cerr << "KVM region experiment unavailable: " << error.what() << '\n';
    return dsp56300::test::kvm::region::skipped;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
