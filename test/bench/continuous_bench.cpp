// SPDX-License-Identifier: GPL-3.0-only
// Continuous guest execution; initialization and expected-state checks are untimed.
#include "backend_support.hpp"
#include "binary_fixture.hpp"
#include "codegen_observer.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "executable_mappings.hpp"
#include "fir_stream_program.hpp"
#include "fixture_memory.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "spectral_stream_program.hpp"
#include "stream_host.hpp"
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
#include <limits>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double million = 1e6, reference_hz = 275e6, host_target_fraction = 0.7;
constexpr int deadline_failure = 2;

struct Workload {
  std::string_view name;
  Word warmup_budget = 0, block_budget = 0, blocks = 0;
  std::uint64_t block_cycles = 0;
  double input_seconds = 0;
};

Workload workload(const std::string_view name) {
  if (name == "fir") {
    return {.name = "fir",
            .warmup_budget = 1 + (fir_stream::sample_rate * fir_stream::frame_instructions),
            .block_budget = fir_stream::block_frames * fir_stream::frame_instructions,
            .blocks = fir_stream::sample_rate / fir_stream::block_frames,
            .block_cycles = std::uint64_t{fir_stream::block_frames} * fir_stream::frame_cycles,
            .input_seconds = 1};
  }
  if (name == "iir") {
    return {.name = "iir",
            .warmup_budget = 1 + (iir_stream::sample_rate * iir_stream::frame_instructions),
            .block_budget = iir_stream::block_frames * iir_stream::frame_instructions,
            .blocks = iir_stream::sample_rate / iir_stream::block_frames,
            .block_cycles = std::uint64_t{iir_stream::block_frames} * iir_stream::frame_cycles,
            .input_seconds = 1};
  }
  if (name == "spectral") {
    const auto plan = spectral_stream::program();
    const auto instructions = static_cast<Word>(plan.hop_progress.instructions);
    return {.name = "spectral",
            .warmup_budget = 1 + (spectral_stream::hops_per_second * instructions),
            .block_budget = instructions,
            .blocks = spectral_stream::hops_per_second,
            .block_cycles = plan.hop_progress.estimated_cycles,
            .input_seconds = 1};
  }
  if (name == "nonlinear") {
    return {.name = "nonlinear",
            .warmup_budget =
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
  std::string_view name;
};

constexpr std::array engines{Engine{.kind = Backend::jit, .name = "jit"}};

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

template <CoreImplementation Implementation>
RunResult advance(Implementation &core, const Word budget) {
  const auto result = core.run(budget);
  require(result.retired == budget && result.fault.kind == FaultKind::none &&
              result.state == ExecutionState::running,
          "Continuous block must retire its exact budget without faults or sleep");
  return result;
}

struct Measurement {
  Progress progress;
  codegen::Publications before, after;
  double warmup = 0, wall = 0, cpu = 0, maximum_block = 0, maximum_cold_block = 0;
  double minimum_cold_hardware_factor = std::numeric_limits<double>::max();
  Word missed_blocks = 0, missed_cold_blocks = 0;
  std::uint64_t cold_compiled_entries = 0;
  std::size_t cold_compiled_bytes = 0;
  std::uint64_t reserved_rx_bytes = 0;
};

template <CoreImplementation Implementation>
Measurement measure(Implementation &core, const Workload spec) {
  Measurement result;
  const auto block_seconds = spec.input_seconds / spec.blocks;
  const auto cold_publications = codegen::publications();
  const auto warmup_begin = Clock::now();
  const auto setup = spec.warmup_budget - (spec.blocks * spec.block_budget);
  auto previous_cycles = core.progress().estimated_cycles;
  for (Word block = 0; block < spec.blocks; ++block) {
    const auto begin = Clock::now();
    const auto run = advance(core, spec.block_budget + (block == 0 ? setup : 0));
    const auto elapsed = seconds(begin);
    const auto cycles = run.progress.estimated_cycles - previous_cycles;
    previous_cycles = run.progress.estimated_cycles;
    result.maximum_cold_block = std::max(result.maximum_cold_block, elapsed);
    result.minimum_cold_hardware_factor = std::min(
        result.minimum_cold_hardware_factor, static_cast<double>(cycles) / reference_hz / elapsed);
    if (elapsed > block_seconds) { ++result.missed_cold_blocks; }
  }
  result.warmup = seconds(warmup_begin);
  const auto progress_before = core.progress();
  result.before = codegen::publications();
  result.cold_compiled_entries = result.before.count - cold_publications.count;
  result.cold_compiled_bytes = result.before.bytes - cold_publications.bytes;
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
  result.after = codegen::publications();
  result.reserved_rx_bytes = jit_executable_bytes();
  require(result.before == result.after,
          "Warm sustained measurements must not silently include new translations");
  require(result.progress.instructions == std::uint64_t{spec.blocks} * spec.block_budget &&
              result.progress.estimated_cycles == spec.blocks * spec.block_cycles,
          "The measured second must contain exactly the declared instructions and base cycles");
  return result;
}

bool report(const Measurement &value, const Engine engine, const Workload spec) {
  const auto minimum_warm_hardware_factor =
      static_cast<double>(spec.block_cycles) / reference_hz / value.maximum_block;
  const auto hardware =
      minimum_warm_hardware_factor >= 1 && value.minimum_cold_hardware_factor >= 1;
  const auto deadline = hardware && value.wall <= spec.input_seconds && value.missed_blocks == 0 &&
                        value.missed_cold_blocks == 0;
  const auto million_instructions = static_cast<double>(value.progress.instructions) / million;
  std::cout << "  " << engine.name << ": input_s=" << spec.input_seconds << " wall_s=" << value.wall
            << " process_cpu_s=" << value.cpu
            << " realtime_factor=" << spec.input_seconds / value.wall << " hardware_factor="
            << static_cast<double>(value.progress.estimated_cycles) / reference_hz / value.wall
            << " warm_min_hardware_factor=" << minimum_warm_hardware_factor
            << " hardware_status=" << (hardware ? "PASS" : "FAIL")
            << " cold_min_hardware_factor=" << value.minimum_cold_hardware_factor
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
            << " cold_max_block_s=" << value.maximum_cold_block
            << " cold_late_blocks=" << value.missed_cold_blocks
            << " package_deadline=" << (deadline ? "PASS" : "FAIL")
            << " cold_warmup_s=" << value.warmup << '\n'
            << "    cold_compiled_entries=" << value.cold_compiled_entries
            << " published_code_bytes=" << value.cold_compiled_bytes
            << " reserved_rx_bytes=" << value.reserved_rx_bytes
            << " measured_compiled_entries=" << value.after.count - value.before.count
            << " full_state=PASS\n";
  return deadline;
}

template <CoreImplementation Implementation>
bool run(const BinaryCase &item, const Engine engine, const Workload spec) {
  require(item.fault.kind == FaultKind::none && item.state == ExecutionState::running &&
              item.reset_actions.empty() &&
              item.budget == spec.warmup_budget + (spec.blocks * spec.block_budget),
          "Continuous fixture must cover exactly warmup plus one measured input second");
  auto memory = initial_memory(item);
  auto expected = memory;
  patch(expected, item.after);
  Implementation core(
      {.program = memory[0], .x = memory[1], .y = memory[2]}, nullptr, {.backend = engine.kind});
  core.set_registers(item.initial);
  const auto result = measure(core, spec);
  require(
      core.progress() == item.progress && core.registers() == item.expected,
      "Continuous completion must match independent complete registers and cumulative progress");
  require(memory == expected, "All P/X/Y words, including untouched regions and stream guards");
  require(core.interrupt_ready() == item.interrupt_ready,
          "Continuous completion retains its declared interrupt boundary");
  HostStreamTimes host;
  if (spec.name == "fir") {
    host = measure_fir_host(item);
  } else if (spec.name == "spectral") {
    host = measure_spectral_host(item);
  } else {
    host = measure_numeric_host(spec.name, item);
  }
  auto fastest = host.avx2_seconds > 0 ? std::min(host.scalar_seconds, host.avx2_seconds)
                                       : host.scalar_seconds;
  if (host.asm_seconds > 0) { fastest = std::min(fastest, host.asm_seconds); }
  std::cout << "  host_scalar_s=" << host.scalar_seconds << " host_avx2_s=" << host.avx2_seconds;
  if (host.asm_seconds > 0) {
    std::cout << " host_asm_s=" << host.asm_seconds
              << " jit_to_asm_host_percent=" << 100.0 * host.asm_seconds / result.wall;
  }
  std::cout << " jit_to_scalar_host_percent=" << 100.0 * host.scalar_seconds / result.wall
            << " jit_to_host_percent=" << 100.0 * fastest / result.wall << " jit_target_70="
            << (fastest >= host_target_fraction * result.wall ? "PASS" : "FAIL")
            << " host_comparison=" << host.scope << " host_implementation=" << host.implementation
            << " host_memory=PASS\n";
  return report(result, engine, spec);
}

bool run_backend(const BinaryCase &item, const Engine engine, const Workload spec) {
  return run<TestCore>(item, engine, spec);
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
  if (argc == 3 && std::string_view(argv[1]) == "--verify-fir-host") {
    verify_fir_host(read_fixture(argv[2]));
    return 0;
  }
  if (argc == 4 && std::string_view(argv[1]) == "--verify-host") {
    const std::string_view name = argv[2];
    const auto cases = read_fixture(argv[3]);
    if (name == "spectral") {
      verify_spectral_host(cases);
    } else {
      verify_numeric_host(name, cases);
    }
    return 0;
  }
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
            << "reference_hz=" << reference_hz << " reference_model=ISA-base-cycles\n"
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
