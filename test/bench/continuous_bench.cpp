// SPDX-License-Identifier: GPL-3.0-only
// Continuous guest execution; initialization and expected-state checks are untimed.
#include "backend_support.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fir_stream_program.hpp"
#include "fixture_memory.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "spectral_stream_program.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double million = 1e6, reference_hz = 275e6;
constexpr int deadline_failure = 2;

struct Workload {
  Word warmup_budget = 0, block_budget = 0, blocks = 0;
  std::uint64_t block_cycles = 0;
  double input_seconds = 0;
};

Workload workload(const std::string_view name) {
  if (name == "fir") {
    return {.warmup_budget = 1 + (fir_stream::sample_rate * fir_stream::frame_instructions),
            .block_budget = fir_stream::block_frames * fir_stream::frame_instructions,
            .blocks = fir_stream::sample_rate / fir_stream::block_frames,
            .block_cycles = std::uint64_t{fir_stream::block_frames} * fir_stream::frame_cycles,
            .input_seconds = 1};
  }
  if (name == "iir") {
    return {.warmup_budget = 1 + (iir_stream::sample_rate * iir_stream::frame_instructions),
            .block_budget = iir_stream::block_frames * iir_stream::frame_instructions,
            .blocks = iir_stream::sample_rate / iir_stream::block_frames,
            .block_cycles = std::uint64_t{iir_stream::block_frames} * iir_stream::frame_cycles,
            .input_seconds = 1};
  }
  if (name == "spectral") {
    const auto plan = spectral_stream::program();
    const auto instructions = static_cast<Word>(plan.hop_progress.instructions);
    return {.warmup_budget = 1 + (spectral_stream::hops_per_second * instructions),
            .block_budget = instructions,
            .blocks = spectral_stream::hops_per_second,
            .block_cycles = plan.hop_progress.estimated_cycles,
            .input_seconds = 1};
  }
  if (name == "nonlinear") {
    return {.warmup_budget =
                1 + (nonlinear_stream::sample_rate * nonlinear_stream::frame_instructions),
            .block_budget = nonlinear_stream::block_frames * nonlinear_stream::frame_instructions,
            .blocks = nonlinear_stream::sample_rate / nonlinear_stream::block_frames,
            .block_cycles =
                std::uint64_t{nonlinear_stream::block_frames} * nonlinear_stream::frame_cycles,
            .input_seconds = 1};
  }
  throw std::invalid_argument("Unknown continuous workload");
}

struct Engine {
  Backend kind;
  BatchBackend batch;
  std::string_view name;
};

constexpr std::array engines{
    Engine{.kind = Backend::reference, .batch = BatchBackend::automatic, .name = "reference"},
    Engine{.kind = Backend::cached, .batch = BatchBackend::automatic, .name = "cached"},
    Engine{.kind = Backend::jit, .batch = BatchBackend::automatic, .name = "jit"},
    Engine{.kind = Backend::jit, .batch = BatchBackend::scalar, .name = "jit-scalar"},
    Engine{.kind = Backend::jit, .batch = BatchBackend::avx2, .name = "jit-avx2"},
    Engine{.kind = Backend::jit, .batch = BatchBackend::disabled, .name = "jit-unbatched"}};

std::size_t index(const std::string_view text) {
  std::size_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw std::invalid_argument("Case must be sample, all, or a numeric index");
  }
  return value;
}

double cpu_seconds() {
  const auto value = std::clock();
  if (value == static_cast<std::clock_t>(-1)) { throw std::runtime_error("CPU clock unavailable"); }
  return static_cast<double>(value) / static_cast<double>(CLOCKS_PER_SEC);
}

double seconds(const Clock::time_point begin) {
  return std::chrono::duration<double>(Clock::now() - begin).count();
}

template <CoreImplementation Implementation> void advance(Implementation &core, const Word budget) {
  const auto result = core.run(budget);
  require(result.dispatches == budget && result.fault.kind == FaultKind::none &&
              result.state == ProcessingState::running,
          "Continuous block must retire its exact budget without faults or sleep");
}

struct Measurement {
  Progress progress;
  JitStatistics before, after;
  double warmup = 0, wall = 0, cpu = 0, maximum_block = 0;
  Word missed_blocks = 0;
};

template <CoreImplementation Implementation>
Measurement measure(Implementation &core, const Workload spec) {
  Measurement result;
  const auto warmup_begin = Clock::now();
  advance(core, spec.warmup_budget);
  result.warmup = seconds(warmup_begin);
  const auto progress_before = core.progress();
  result.before = core.jit_statistics();
  const auto block_seconds = spec.input_seconds / spec.blocks;
  const auto cpu_begin = cpu_seconds();
  const auto begin = Clock::now();
  for (Word block = 0; block < spec.blocks; ++block) {
    const auto block_begin = Clock::now();
    advance(core, spec.block_budget);
    const auto wall = seconds(block_begin);
    result.maximum_block = std::max(result.maximum_block, wall);
    if (wall > block_seconds) { ++result.missed_blocks; }
  }
  result.wall = seconds(begin);
  result.cpu = cpu_seconds() - cpu_begin;
  result.progress = {.instructions = core.progress().instructions - progress_before.instructions,
                     .estimated_cycles =
                         core.progress().estimated_cycles - progress_before.estimated_cycles};
  result.after = core.jit_statistics();
  require(result.progress.instructions == std::uint64_t{spec.blocks} * spec.block_budget &&
              result.progress.estimated_cycles == spec.blocks * spec.block_cycles,
          "The measured second must contain exactly the declared instructions and base cycles");
  return result;
}

bool report(const Measurement &value, const Engine engine, const Workload spec) {
  const auto deadline = value.wall <= spec.input_seconds;
  const auto million_instructions = static_cast<double>(value.progress.instructions) / million;
  const auto native = value.after.native_alu_instructions - value.before.native_alu_instructions;
  const auto helpers = value.after.helper_instructions - value.before.helper_instructions;
  if (engine.kind == Backend::jit) {
    require(native + helpers == value.progress.instructions,
            "Measured native and helper accounting must charge every instruction once");
  }
  std::cout << "  " << engine.name << ": input_s=" << spec.input_seconds << " wall_s=" << value.wall
            << " process_cpu_s=" << value.cpu
            << " realtime_factor=" << spec.input_seconds / value.wall
            << " equivalent_cores=" << value.cpu / spec.input_seconds
            << " MIPS=" << million_instructions / value.wall
            << " guest_instructions=" << value.progress.instructions
            << " base_cycles=" << value.progress.estimated_cycles << " reference_load="
            << static_cast<double>(value.progress.estimated_cycles) /
                   (reference_hz * spec.input_seconds)
            << '\n'
            << "    blocks=" << spec.blocks << " max_block_s=" << value.maximum_block
            << " block_budget_s=" << spec.input_seconds / spec.blocks
            << " late_blocks=" << value.missed_blocks
            << " package_deadline=" << (deadline ? "PASS" : "FAIL")
            << " cold_warmup_s=" << value.warmup << '\n'
            << "    native_alu=" << native << " helper_retirements=" << helpers << " native_moves="
            << value.after.native_move_instructions - value.before.native_move_instructions
            << " batch=" << value.after.batch_instructions - value.before.batch_instructions
            << " avx2=" << value.after.avx2_instructions - value.before.avx2_instructions
            << " code_bytes=" << value.after.executable_bytes << " measured_compiled_blocks="
            << value.after.compiled_blocks - value.before.compiled_blocks << " full_state=PASS\n";
  return deadline;
}

template <CoreImplementation Implementation>
bool run(const BinaryCase &item, const Engine engine, const Workload spec) {
  require(item.fault.kind == FaultKind::none && item.state == ProcessingState::running &&
              item.reset_actions.empty() &&
              item.budget == spec.warmup_budget + (spec.blocks * spec.block_budget),
          "Continuous fixture must cover exactly warmup plus one measured input second");
  auto memory = initial_memory(item);
  auto expected = memory;
  patch(expected, item.after);
  Implementation core({.program = memory[0], .x = memory[1], .y = memory[2]},
                      nullptr,
                      {.backend = engine.kind, .batch_backend = engine.batch});
  core.set_registers(item.initial);
  const auto result = measure(core, spec);
  require(
      core.progress() == item.progress && core.registers() == item.expected,
      "Continuous completion must match independent complete registers and cumulative progress");
  require(memory == expected, "All P/X/Y words, including untouched regions and stream guards");
  require(core.interruptible() == item.interruptible && core.debug_status() == item.debug,
          "Continuous completion retains the declared interrupt and debug boundary");
  return report(result, engine, spec);
}

bool run_backend(const BinaryCase &item, const Engine engine, const Workload spec) {
  switch (engine.kind) {
    case Backend::automatic: return run<BackendCore<Backend::automatic>>(item, engine, spec);
    case Backend::reference: return run<BackendCore<Backend::reference>>(item, engine, spec);
    case Backend::cached: return run<BackendCore<Backend::cached>>(item, engine, spec);
    case Backend::jit:
      switch (engine.batch) {
        case BatchBackend::automatic: return run<BackendCore<Backend::jit>>(item, engine, spec);
        case BatchBackend::scalar:
          return run<BackendCore<Backend::jit, BatchBackend::scalar>>(item, engine, spec);
        case BatchBackend::avx2:
          return run<BackendCore<Backend::jit, BatchBackend::avx2>>(item, engine, spec);
        case BatchBackend::disabled:
          return run<BackendCore<Backend::jit, BatchBackend::disabled>>(item, engine, spec);
        default: break;
      }
      break;
    default: break;
  }
  throw std::invalid_argument("Unknown continuous backend");
}

bool selected(const std::string_view name) {
  return name == "all" ||
         std::ranges::any_of(engines, [name](const auto engine) { return engine.name == name; });
}

struct Selection {
  std::string_view backend;
  std::size_t chosen = 0;
  bool all = false;
};

bool run_cases(const std::vector<BinaryCase> &cases, const Workload spec, const Selection choice) {
  bool met = true;
  std::size_t measured = 0;
  for (std::size_t number = 0; number < cases.size(); ++number) {
    if (!choice.all && choice.chosen != number) { continue; }
    std::cout << "case " << number << ": " << cases[number].name << '\n';
    for (const auto engine : engines) {
      if (choice.backend != "all" && choice.backend != engine.name) { continue; }
      met = run_backend(cases[number], engine, spec) && met;
      ++measured;
    }
  }
  std::cout << "Continuous summary: available_cases=" << cases.size() << " measured=" << measured
            << " every_package_deadline=" << (met ? "PASS" : "FAIL") << '\n';
  return met;
}

int execute(const int argc, const char *const *const argv) {
  if (argc < 3 || argc > 6) {
    throw std::runtime_error("Usage: dsp56300_continuous_bench fir|iir|spectral|nonlinear FILE.d56 "
                             "[INDEX|sample|all [BACKEND [report|enforce]]]");
  }
  const auto spec = workload(argv[1]);
  const auto cases = read_fixture(argv[2]);
  const std::string_view selection = argc >= 4 ? argv[3] : "all";
  const std::string_view backend = argc >= 5 ? argv[4] : "jit";
  const std::string_view policy = argc >= 6 ? argv[5] : "enforce";
  if (!selected(backend) || (policy != "enforce" && policy != "report")) {
    throw std::invalid_argument("Unknown backend or deadline policy");
  }
  const bool all = selection == "all" || selection == "sample";
  const auto chosen = all ? 0 : index(selection);
  if (cases.empty() || chosen >= cases.size()) {
    throw std::invalid_argument("Case outside fixture");
  }
  std::cout << std::fixed << std::setprecision(6)
            << "Continuous single DSP: one input second warmup, one input second measured.\n"
            << "No host register/RAM restoration, no repeated calls of a completed program.\n"
            << "Wall includes run calls, clocks and budget checks; no overhead subtraction.\n"
            << "Process CPU includes all process threads; no board/wait-state or energy claim.\n"
            << "Deadline policy=" << policy << "; base cycles exclude pipeline/bus stalls.\n";
  const auto met = run_cases(cases, spec, {.backend = backend, .chosen = chosen, .all = all});
  return policy == "enforce" && !met ? deadline_failure : 0;
}

} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    return dsp56300::test::execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
