// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
constexpr unsigned warmup_calls = 64, trial_count = 9, calls_per_trial = 4096;
constexpr Word loop_flag = Word{1} << 15;

double elapsed(const Clock::time_point begin, const Clock::time_point end) {
  return std::chrono::duration<double, std::nano>(end - begin).count();
}

struct Timing {
  double execution = 0, preparation = 0;
};

class Replay {
public:
  Replay(const BinaryCase &item, const Backend backend)
      : item_(item),
        initial_(initial_memory(item)),
        expected_(initial_),
        memory_(initial_),
        core_({.program = memory_[0], .x = memory_[1], .y = memory_[2]},
              nullptr,
              {.backend = backend}) {
    require(item.fault.kind == FaultKind::none && item.interruptible && item.dispatches != 0,
            "Benchmark requires a successful, interruptible completion");
    require(((item.initial.sr | item.expected.sr) & loop_flag) == 0,
            "Benchmark replay requires boundaries outside active DO loops");
    patch(expected_, item.after);
    // Host replay cannot restore program writes without invalidating cached code.
    require(initial_[0] == expected_[0], "Benchmark replay requires unchanged program memory");
  }

  void prepare() {
    std::ranges::copy(initial_[1], memory_[1].begin());
    std::ranges::copy(initial_[2], memory_[2].begin());
    core_.set_registers(item_.initial);
  }

  void execute() {
    const auto result = core_.run(item_.budget);
    require(result.dispatches == item_.dispatches && result.fault.kind == FaultKind::none,
            "Benchmark execution completes");
  }

  void validate() const {
    require(core_.interruptible() && core_.registers() == item_.expected && memory_ == expected_,
            "Benchmark reproduces the qualified fixture at a quiescent boundary");
  }

  [[nodiscard]] JitStatistics statistics() const noexcept { return core_.jit_statistics(); }

  Timing trial() {
    Timing timing;
    const auto before = core_.progress();
    for (unsigned call = 0; call < calls_per_trial; ++call) {
      const auto begin = Clock::now();
      prepare();
      const auto prepared = Clock::now();
      execute();
      const auto end = Clock::now();
      timing.preparation += elapsed(begin, prepared);
      timing.execution += elapsed(prepared, end);
    }
    validate();
    const auto after = core_.progress();
    require(after.instructions - before.instructions ==
                    item_.progress.instructions * calls_per_trial &&
                after.estimated_cycles - before.estimated_cycles ==
                    item_.progress.estimated_cycles * calls_per_trial,
            "Benchmark retains instruction/base-cost accounting across replays");
    timing.execution /= calls_per_trial;
    timing.preparation /= calls_per_trial;
    return timing;
  }

private:
  const BinaryCase &item_;
  FixtureMemory initial_, expected_, memory_;
  Core core_;
};

void benchmark(const BinaryCase &item, const Backend backend) {
  Replay replay(item, backend);
  for (unsigned call = 0; call < warmup_calls; ++call) {
    replay.prepare();
    replay.execute();
  }
  replay.validate();
  std::array<double, trial_count> execution{};
  std::array<double, trial_count> preparation{};
  for (std::size_t trial = 0; trial < execution.size(); ++trial) {
    const auto timing = replay.trial();
    execution[trial] = timing.execution;
    preparation[trial] = timing.preparation;
  }
  std::ranges::sort(execution);
  std::ranges::sort(preparation);
  constexpr auto middle = trial_count / 2;
  std::cout << backend_name(backend) << ": median execution " << execution[middle] << " ns/call ("
            << execution[middle] / static_cast<double>(item.progress.instructions)
            << " ns/instruction), trial range " << execution.front() << ".." << execution.back()
            << "; host preparation " << preparation[middle] << " ns/call\n";
  if (backend == Backend::jit) {
    const auto statistics = replay.statistics();
    std::cout << "  native ALU/retirement: " << statistics.native_alu_instructions
              << ", full helpers: " << statistics.helper_instructions
              << ", compiled blocks: " << statistics.compiled_blocks
              << ", executable bytes: " << statistics.executable_bytes << '\n';
  }
}

std::size_t case_index(const std::string_view text) {
  std::size_t value = 0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    throw std::invalid_argument("Invalid fixture case index");
  }
  return value;
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2 || argc > 3) {
      throw std::runtime_error("Usage: dsp56300_fixture_bench FILE.d56 [CASE_INDEX]");
    }
    const auto cases = dsp56300::test::read_fixture(argv[1]);
    const auto index = argc == 3 ? dsp56300::test::case_index(argv[2]) : std::size_t{0};
    const auto &item = cases.at(index);
    std::cout << std::fixed << std::setprecision(2) << item.name << '\n'
              << "One DSP instance per backend, 64 warmups, median of 9 x 4096 replays.\n"
              << "Execution includes timer/retirement-check overhead; no overhead subtraction.\n"
              << "Host RAM/register restoration is timed separately; no cold-load/device claim.\n";
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::benchmark(item, backend);
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
