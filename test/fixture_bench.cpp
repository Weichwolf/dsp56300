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
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <ratio>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
constexpr unsigned warmup_calls = 64, trial_count = 9, default_replays = 4096;
constexpr Word loop_flag = Word{1} << 15;
constexpr Word cache_flag = Word{1} << 19;
constexpr double nanoseconds_per_second = 1e9;
constexpr double mips_time_scale = 1000;
constexpr unsigned target_instructions_per_trial = 100000;
constexpr std::size_t sample_cases = 8;

double elapsed(const Clock::time_point begin, const Clock::time_point end) {
  return std::chrono::duration<double, std::nano>(end - begin).count();
}

struct Timing {
  double execution = 0, preparation = 0, cpu = 0;
};

double cpu_time() {
  const auto value = std::clock();
  if (value == static_cast<std::clock_t>(-1)) { throw std::runtime_error("CPU clock unavailable"); }
  return static_cast<double>(value) * nanoseconds_per_second / static_cast<double>(CLOCKS_PER_SEC);
}

std::string_view exclusion(const BinaryCase &item) {
  if (item.fault.kind != FaultKind::none) { return "expected fault"; }
  if (item.dispatches == 0 || item.progress.instructions == 0 || !item.interruptible) {
    return "no quiescent retirement boundary";
  }
  if (((item.initial.sr | item.expected.sr) & (loop_flag | cache_flag)) != 0) {
    return "active loop or architectural cache state";
  }
  if (item.state != ProcessingState::running || item.debug_interrupt_mode ||
      item.debug != DebugStatus{} || !item.reset_actions.empty()) {
    return "processing/debug/host-script state";
  }
  if (std::ranges::any_of(item.after,
                          [](const auto &block) { return block.space == Space::program; })) {
    return "program mutation";
  }
  return {};
}

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
    require(result.dispatches == item_.dispatches && result.fault.kind == FaultKind::none &&
                result.state == item_.state,
            "Benchmark execution completes");
  }

  void validate() const {
    require(core_.interruptible() && core_.registers() == item_.expected && memory_ == expected_ &&
                core_.processing_state() == item_.state && core_.debug_status() == item_.debug,
            "Benchmark reproduces the qualified fixture at a quiescent boundary");
  }

  [[nodiscard]] JitStatistics statistics() const noexcept { return core_.jit_statistics(); }

  Timing trial(const unsigned calls_per_trial) {
    Timing timing;
    const auto before = core_.progress();
    const auto cpu_begin = cpu_time();
    for (unsigned call = 0; call < calls_per_trial; ++call) {
      const auto begin = Clock::now();
      prepare();
      const auto prepared = Clock::now();
      execute();
      const auto end = Clock::now();
      timing.preparation += elapsed(begin, prepared);
      timing.execution += elapsed(prepared, end);
    }
    timing.cpu = (cpu_time() - cpu_begin) / calls_per_trial;
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

bool benchmark(const BinaryCase &item, const Backend backend, const unsigned replays) {
  Replay replay(item, backend);
  replay.prepare();
  const auto cold_begin = Clock::now();
  replay.execute();
  const auto cold_end = Clock::now();
  replay.validate();
  auto previous = replay.statistics();
  for (unsigned call = 0; call < warmup_calls; ++call) {
    previous = replay.statistics();
    replay.prepare();
    replay.execute();
  }
  replay.validate();
  const auto warmed = replay.statistics();
  if (warmed.compiled_blocks != previous.compiled_blocks) {
    std::cout << "SKIP: replay repeatedly compiles code\n";
    return false;
  }
  std::array<double, trial_count> execution{};
  std::array<double, trial_count> preparation{};
  std::array<double, trial_count> cpu{};
  for (std::size_t trial = 0; trial < execution.size(); ++trial) {
    const auto timing = replay.trial(replays);
    execution[trial] = timing.execution;
    preparation[trial] = timing.preparation;
    cpu[trial] = timing.cpu;
  }
  std::ranges::sort(execution);
  std::ranges::sort(preparation);
  std::ranges::sort(cpu);
  constexpr auto middle = trial_count / 2;
  std::cout << backend_name(backend) << ": median execution " << execution[middle] << " ns/call ("
            << execution[middle] / static_cast<double>(item.progress.instructions)
            << " ns/instruction), trial range " << execution.front() << ".." << execution.back()
            << "; host preparation " << preparation[middle] << " ns/call\n";
  std::cout << "  guest instructions/call: " << item.progress.instructions
            << ", base units/call: " << item.progress.estimated_cycles << ", guest MIPS: "
            << static_cast<double>(item.progress.instructions) * mips_time_scale / execution[middle]
            << ", process CPU: " << cpu[middle] << " ns/call (includes preparation/timer)\n"
            << "  cold execution: " << elapsed(cold_begin, cold_end)
            << " ns (includes any first-use compilation); replays/trial: " << replays << '\n';
  require(replay.statistics().compiled_blocks == warmed.compiled_blocks,
          "Warm measurements cannot silently include JIT recompilation");
  if (backend == Backend::jit) {
    const auto statistics = replay.statistics();
    const auto calls = static_cast<double>(trial_count) * replays;
    std::cout << "  native ALU/retirement per call: "
              << static_cast<double>(statistics.native_alu_instructions -
                                     warmed.native_alu_instructions) /
                     calls
              << ", full helpers: "
              << static_cast<double>(statistics.helper_instructions - warmed.helper_instructions) /
                     calls
              << ", native moves: "
              << static_cast<double>(statistics.native_move_instructions -
                                     warmed.native_move_instructions) /
                     calls
              << ", compiled blocks: " << statistics.compiled_blocks
              << ", executable bytes: " << statistics.executable_bytes << '\n';
  }
  return true;
}

std::size_t case_index(const std::string_view text) {
  std::size_t value = 0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    throw std::invalid_argument("Invalid fixture case index");
  }
  return value;
}

std::optional<Backend> selected_backend(const std::string_view name) {
  if (name == "all") { return std::nullopt; }
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    if (name == backend_name(backend)) { return backend; }
  }
  throw std::invalid_argument("Invalid backend: use all, reference, cached or jit");
}

std::vector<std::size_t> select_cases(const std::vector<BinaryCase> &cases,
                                      const std::string_view selection) {
  if (selection != "all" && selection != "sample") { return {case_index(selection)}; }
  std::vector<std::size_t> eligible;
  eligible.reserve(cases.size());
  for (std::size_t index = 0; index < cases.size(); ++index) {
    if (exclusion(cases[index]).empty()) { eligible.push_back(index); }
  }
  if (selection == "all" || eligible.size() <= sample_cases) { return eligible; }
  std::vector<std::size_t> result;
  result.reserve(sample_cases);
  for (std::size_t index = 0; index < sample_cases; ++index) {
    result.push_back(eligible[index * (eligible.size() - 1) / (sample_cases - 1)]);
  }
  return result;
}

bool run_cases(const std::vector<BinaryCase> &cases,
               const std::string_view selection,
               const std::optional<Backend> selected,
               const std::string_view replay_option) {
  const auto indices = select_cases(cases, selection);
  const auto eligible =
      std::ranges::count_if(cases, [](const auto &item) { return exclusion(item).empty(); });
  std::map<std::string_view, std::size_t> excluded;
  for (const auto &item : cases) {
    const auto reason = exclusion(item);
    if (!reason.empty()) { ++excluded[reason]; }
  }
  std::size_t measured = 0;
  for (const auto index : indices) {
    const auto &item = cases.at(index);
    const auto reason = exclusion(item);
    if (!reason.empty()) {
      std::cout << "case " << index << " SKIP: " << reason << '\n';
      continue;
    }
    const auto replays =
        replay_option == "auto"
            ? std::clamp<std::size_t>(
                  target_instructions_per_trial / item.progress.instructions, 1, default_replays)
            : case_index(replay_option);
    constexpr unsigned maximum_replays = 1U << 20;
    if (replays == 0 || replays > maximum_replays) {
      throw std::invalid_argument("Replays must be in [1,1048576]");
    }
    std::cout << "case " << index << ": " << item.name << '\n';
    for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
      if (selected.has_value() && *selected != backend) { continue; }
      measured += benchmark(item, backend, static_cast<unsigned>(replays)) ? 1U : 0U;
    }
  }
  std::cout << "Fixture summary: total=" << cases.size() << ", warm-eligible=" << eligible
            << ", selected=" << indices.size() << ", measured backend/cases=" << measured
            << "; excluded cases remain correctness-only\n";
  for (const auto &[reason, count] : excluded) {
    std::cout << "  excluded " << count << ": " << reason << '\n';
  }
  return measured != 0;
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2 || argc > 5) {
      throw std::runtime_error(
          "Usage: dsp56300_fixture_bench FILE.d56 [INDEX|sample|all [BACKEND [REPLAYS|auto]]]");
    }
    const auto selected = dsp56300::test::selected_backend(argc >= 4 ? argv[3] : "all");
    const auto cases = dsp56300::test::read_fixture(argv[1]);
    std::cout
        << std::fixed << std::setprecision(2)
        << "One DSP instance per backend/case, one cold call, 64 warmups, median of 9 trials.\n"
        << "Execution includes timer/retirement-check overhead; no overhead subtraction.\n"
        << "Host RAM/register restoration is timed separately; no cold-load/device claim.\n";
    constexpr int no_warm_cases = 77;
    return dsp56300::test::run_cases(
               cases, argc >= 3 ? argv[2] : "0", selected, argc == 5 ? argv[4] : "4096")
               ? 0
               : no_warm_cases;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
