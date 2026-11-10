// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fir_host.hpp"
#include "fixture_memory.hpp"
#include "host_features.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace dsp56300::test::fir_host {
namespace {
using Clock = std::chrono::steady_clock;
constexpr Word output_address = 16;
constexpr unsigned trials = 9, repetitions = 4096;
constexpr Word medium_taps = 64, large_taps = 512;
constexpr Word dual_postincrement_x0_y0 = 0x709800, store_a_fields = 0x120000;
constexpr Word mac_x0_y0_selector = 5U << 4;
constexpr auto clear = isa::opcode<isa::Opcode::clr_a>()[0];
constexpr auto mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | mac_x0_y0_selector;
constexpr auto parallel = isa::opcode<isa::Opcode::move_xy>()[0] | dual_postincrement_x0_y0;

void check(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

class GuardedWords {
public:
  explicit GuardedWords(const std::span<const Word> words) {
    const auto page = sysconf(_SC_PAGESIZE);
    check(page > 0, "Guard-page verification needs the real host page size");
    page_ = static_cast<std::size_t>(page);
    check(words.size_bytes() <= page_, "Guarded FIR input fits one readable page");
    mapping_ = mmap(nullptr, 2 * page_, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    check(mapping_ != MAP_FAILED, "Cannot allocate guarded FIR input");
    if (mprotect(static_cast<std::byte *>(mapping_) + page_, page_, PROT_NONE) != 0) {
      munmap(mapping_, 2 * page_);
      throw std::runtime_error("Cannot protect FIR input boundary");
    }
    auto *const end = static_cast<Word *>(mapping_) + (page_ / sizeof(Word));
    view_ = std::span<Word>(end - words.size(), words.size());
    static_cast<void>(std::ranges::uninitialized_copy(words, view_));
  }

  ~GuardedWords() { munmap(mapping_, 2 * page_); }

  GuardedWords(const GuardedWords &) = delete;
  GuardedWords &operator=(const GuardedWords &) = delete;

  [[nodiscard]] std::span<const Word> view() const noexcept { return view_; }

private:
  void *mapping_ = nullptr;
  std::size_t page_ = 0;
  std::span<Word> view_;
};

void guarded_inputs(const BinaryCase &item) {
  auto initial = initial_memory(item);
  const auto taps = item.budget - 4;
  const auto data_words = std::span<Word>(initial[1]).subspan(item.initial.r[0], taps);
  const auto coefficient_words = std::span<Word>(initial[2]).subspan(item.initial.r[4], taps);
  // Host padding bits are not part of a DSP word.
  for (const auto words : {data_words, coefficient_words}) {
    for (auto &word : words) { word |= ~word_mask; }
  }
  const GuardedWords data(data_words);
  const GuardedWords coefficients(coefficient_words);
  for (const auto host_isa : {HostIsa::scalar, HostIsa::vector, HostIsa::cpp}) {
    if (host_isa == HostIsa::vector && !vector_supported()) { continue; }
    const auto result = execute(item.initial, data.view(), coefficients.view(), host_isa);
    check(result.registers == item.expected && result.output == item.after[0].words[0],
          "Guarded host inputs retain complete state without a vector or scalar tail overread");
  }
}

std::array<Word, 7> program(const Word taps) {
  const auto count = taps - 1;
  return {parallel | (clear & status_byte),
          isa::opcode<isa::Opcode::do_immediate>()[0] | ((count & status_byte) << 8) | (count >> 8),
          start + 3,
          parallel | (mac & status_byte),
          mac,
          isa::opcode<isa::Opcode::rnd_a>()[0],
          isa::opcode<isa::Opcode::move_x_absolute_short>()[0] | store_a_fields |
              (output_address << 8)};
}

class Replay {
public:
  explicit Replay(const BinaryCase &item)
      : memory_(initial_memory(item)),
        core_(Memory::separate({.program = memory_[0], .x = memory_[1], .y = memory_[2]})) {}

  void prepare(const BinaryCase &item) {
    const auto initial = initial_memory(item);
    check(initial[0] == memory_[0], "Replay retains the same immutable guest program");
    std::ranges::copy(initial[1], memory_[1].begin());
    std::ranges::copy(initial[2], memory_[2].begin());
    host_memory_ = initial;
    data_ = std::span<const Word>(host_memory_[1]).subspan(item.initial.r[0], item.budget - 4);
    coefficients_ =
        std::span<const Word>(host_memory_[2]).subspan(item.initial.r[4], item.budget - 4);
    auto expected = initial;
    patch(expected, item.after);
    for (const auto host_isa : {HostIsa::scalar, HostIsa::vector, HostIsa::cpp}) {
      if (host_isa == HostIsa::vector && !vector_supported()) { continue; }
      run_host(item, host_isa);
      check(host_memory_ == expected,
            "Each host ISA matches independent full registers and all P/X/Y words");
    }
    const auto before = core_.state().progress;
    run_core(item);
    check(memory_ == expected && core_.interrupt_ready(),
          "Core matches independent full P/X/Y memory at a quiescent boundary");
    const auto after = core_.state().progress;
    check(after.instructions - before.instructions == item.progress.instructions &&
              after.estimated_cycles - before.estimated_cycles == item.progress.estimated_cycles,
          "Core preserves independently modeled instruction and cycle deltas");
  }

  void run_core(const BinaryCase &item) {
    memory_[1][output_address] = 0;
    core_.set_registers(item.initial);
    const auto result = core_.run(item.budget);
    check(result.retired == item.retired && result.state == item.state &&
              result.fault == item.fault && !result.yielded && core_.registers() == item.expected &&
              memory_[1][output_address] == item.after[0].words[0],
          "Core execution matches all registers and the output word");
  }

  [[nodiscard]] Progress progress() const { return core_.state().progress; }

  void run_host(const BinaryCase &item, const HostIsa host_isa) {
    host_memory_[1][output_address] = 0;
    const auto result = execute(item.initial, data_, coefficients_, host_isa);
    host_memory_[1][output_address] = result.output;
    check(result.registers == item.expected &&
              host_memory_[1][output_address] == item.after[0].words[0],
          "Host execution matches all registers and the output word");
  }

private:
  FixtureMemory memory_;
  FixtureMemory host_memory_;
  Core core_;
  std::span<const Word> data_, coefficients_;
};

void validate_shape(const BinaryCase &item) {
  check(item.budget > 4 && item.before.size() == 3 && item.after.size() == 1,
        "Only the independently authored seven-word FIR fixture is accepted");
  check(std::ranges::equal(item.before[0].words, program(item.budget - 4)) &&
            item.before[0].address == start && item.before[0].space == Space::program &&
            item.after[0].space == Space::x && item.after[0].address == output_address &&
            item.after[0].words.size() == 1,
        "Named ISA/operand fields match the pinned independent FIR program");
  check(item.initial.pc == start && item.initial.n[0] == 1 && item.initial.n[4] == 1 &&
            item.initial.m[0] == word_mask && item.initial.m[4] == word_mask &&
            item.state == ExecutionState::running && item.fault.kind == FaultKind::none,
        "Host baseline models the same linear AGUs and successful running boundary");
}

template <typename Function> double measure(Function &&function) {
  const auto begin = Clock::now();
  for (unsigned repetition = 0; repetition < repetitions; ++repetition) { function(); }
  return std::chrono::duration<double>(Clock::now() - begin).count();
}

void benchmark(Replay &replay, const BinaryCase &item) {
  const auto before = codegen::publications();
  const auto progress = replay.progress();
  std::array<double, trials> core_times{};
  std::array<double, trials> scalar_times{};
  std::array<double, trials> vector_times{};
  std::array<double, trials> cpp_times{};
  const bool vectors = vector_supported();
  const unsigned engines = vectors ? 4U : 3U;
  for (unsigned trial = 0; trial < trials; ++trial) {
    const auto invoke = [&](const unsigned engine) {
      if (engine == 0) {
        core_times[trial] = measure([&] { replay.run_core(item); });
      } else if (engine == 1) {
        scalar_times[trial] = measure([&] { replay.run_host(item, HostIsa::scalar); });
      } else if (engine == 2) {
        cpp_times[trial] = measure([&] { replay.run_host(item, HostIsa::cpp); });
      } else {
        vector_times[trial] = measure([&] { replay.run_host(item, HostIsa::vector); });
      }
    };
    for (unsigned step = 0; step < engines; ++step) { invoke((trial + step) % engines); }
  }
  std::ranges::sort(core_times);
  std::ranges::sort(scalar_times);
  std::ranges::sort(vector_times);
  std::ranges::sort(cpp_times);
  const auto core = core_times[trials / 2];
  const auto scalar = scalar_times[trials / 2];
  const auto cpp = cpp_times[trials / 2];
  const auto vector = vectors ? vector_times[trials / 2] : 0;
  const bool vector_faster = vectors && vector < scalar;
  const auto host = vector_faster ? vector : scalar;
  const auto after = codegen::publications();
  const auto expected = std::uint64_t{item.budget} * trials * repetitions;
  check(after == before && replay.progress().instructions - progress.instructions == expected &&
            replay.progress().estimated_cycles - progress.estimated_cycles ==
                std::uint64_t{item.progress.estimated_cycles} * trials * repetitions,
        "Warm measurements retain exact instruction/cycle charges and publish no native code");
  constexpr double million = 1000000.0;
  std::cout << item.name << " core_s=" << core << " host_s=" << host << " host_scalar_s=" << scalar
            << " host_cpp_o3_s=" << cpp << " jit_to_cpp_o3_percent=" << cpp / core * 100.0
            << " host_" << host_vector_name << "_s=" << vector
            << " host_isa=" << (vector_faster ? host_vector_name : "scalar")
            << " jit_to_host_percent=" << host / core * 100.0
            << " guest_mips=" << static_cast<double>(item.budget) * repetitions / core / million
            << " warm_publications=" << after.count - before.count << '\n';
}

bool selected(const BinaryCase &item) {
  const auto taps = item.budget - 4;
  return (taps == 16 || taps == medium_taps || taps == large_taps) && item.initial.sr == 0 &&
         (item.name.find(" signal=3 coefficients=0 ") != std::string::npos ||
          item.name.find(" signal=2 coefficients=3 ") != std::string::npos);
}

void run(const std::vector<BinaryCase> &cases, const bool verify_only) {
  std::unique_ptr<Replay> replay;
  Word previous_budget = 0;
  std::size_t measured = 0;
  for (const auto &item : cases) {
    try {
      validate_shape(item);
      if (item.budget != previous_budget) {
        guarded_inputs(item);
        replay = std::make_unique<Replay>(item);
        previous_budget = item.budget;
      }
      replay->prepare(item);
      if (!verify_only && selected(item)) {
        benchmark(*replay, item);
        ++measured;
      }
    } catch (const std::exception &error) {
      throw std::runtime_error(item.name + ": " + error.what());
    }
  }
  constexpr std::size_t expected_cases = 2340;
  constexpr std::size_t expected_measurements = 6;
  check(
      cases.size() == expected_cases && (verify_only || measured == expected_measurements),
      "Complete tap/mode/signal matrix and intermediate-overflow cancellation cases are required");
  std::cout << "verified_cases=" << cases.size() << " measured=" << measured << " trials=" << trials
            << " repetitions=" << repetitions << '\n';
}
} // namespace
} // namespace dsp56300::test::fir_host

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2 || argc > 3 || (argc == 3 && std::string_view(argv[2]) != "--verify")) {
      throw std::invalid_argument("Usage: dsp56300_fir_host_bench FIR.d56 [--verify]");
    }
    std::cout << std::setprecision(9)
              << "scope=reset+run+full-register/output-check; immutable-code-and-inputs; "
                 "warm-translation; best-qualified-scalar/SIMD-host-asm; median-nine; "
                 "host-cpp=clang19-O3; llvm-preparation=not-requested; "
                 "intermediate-dead-flags-elided-but-sticky-L-preserved; "
                 "full-memory-and-cycle-checks-outside-timing; FIR-only-not-performance-gate\n";
    dsp56300::test::fir_host::run(dsp56300::test::read_fixture(argv[1]), argc == 3);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Matched FIR benchmark: " << error.what() << '\n';
    return 1;
  }
}
