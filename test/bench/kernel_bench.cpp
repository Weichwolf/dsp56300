// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "kernel_counters.hpp"
#include "kernel_host.hpp"
#include "kernel_program.hpp"
#include "kernel_reference.hpp"
#include "retirement_observer.hpp"
#include "support.hpp"

#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace dsp56300;
namespace k = test::kernels;
namespace reference = core_kernels;
using test::require;
using Clock = std::chrono::steady_clock;
constexpr unsigned bits = reference::dsp_fraction_bits;
constexpr double nanoseconds = 1e9;
constexpr std::uint64_t minimum_instructions = 10'000'000;
constexpr std::string_view fir_hash =
    "e4cee01f3826c7629135ecb224a4f8fcd18a225d6c248d1505dd7b15e6f54489";
constexpr std::string_view biquad_hash =
    "fc87b101ab8b0be1ddb5aa13db865b2c6a44f059450432ea5c872f3849351381";
constexpr std::string_view state_hash =
    "d3fb6119d7cc9de2918f841f54772c88902132bb8d19357c0fae8522f8866439";

std::string digest(const std::span<const reference::DspSample> values) {
  llvm::SHA256 hash;
  for (const auto value : values) {
    std::array<std::uint8_t, sizeof(std::uint32_t)> bytes{};
    const auto word = static_cast<std::uint32_t>(value);
    for (std::size_t index = 0; index < bytes.size(); ++index) {
      bytes[index] = static_cast<std::uint8_t>((word >> (index * 8)) & test::status_byte);
    }
    hash.update(bytes);
  }
  constexpr std::string_view hex = "0123456789abcdef";
  std::string result;
  for (const auto byte : hash.final()) {
    result += hex[byte >> 4];
    result += hex[byte & 15U];
  }
  return result;
}

struct Workload {
  explicit Workload(const bool filter) : biquad(filter), code(filter ? k::biquads() : k::fir()) {
    const auto signal = reference::signal<bits>(reference::samples);
    std::ranges::copy(signal, std::span(input).subspan(biquad ? 0 : reference::taps - 1).begin());
    expected.resize(reference::samples);
    if (biquad) {
      reference::host_biquads(input, weights, final_state, expected);
    } else {
      reference::host_fir(input, taps, expected);
    }
    require(digest(expected) == (biquad ? biquad_hash : fir_hash),
            "Complete host output matches the independent bigint golden");
    if (biquad) {
      std::array<reference::DspSample, reference::sections * 2> flattened{};
      for (std::size_t section = 0; section < reference::sections; ++section) {
        std::ranges::copy(final_state[section], flattened.begin() + (2 * section));
      }
      require(digest(flattened) == state_hash, "Complete host filter state matches its golden");
    }
  }

  bool biquad;
  std::vector<Word> code;
  std::vector<reference::DspSample> input =
      std::vector<reference::DspSample>(reference::samples + (biquad ? 0 : reference::taps - 1));
  std::vector<reference::DspSample> expected;
  std::array<reference::DspSample, reference::taps> taps = reference::fir_coefficients<bits>();
  reference::Coefficients<bits> weights = reference::biquad_coefficients<bits>();
  reference::State<bits> final_state = reference::initial_state<bits>();
};

double host_time(const Workload &workload) {
  std::vector<reference::DspSample> output(reference::samples);
  auto state = reference::initial_state<bits>();
  const auto run = [&] {
    state = reference::initial_state<bits>();
    if (workload.biquad) {
      reference::host_biquads(workload.input, workload.weights, state, output);
    } else {
      reference::host_fir(workload.input, workload.taps, output);
    }
  };
  run();
  std::array<double, 3> times{};
  for (auto &time : times) {
    const auto start = Clock::now();
    run();
    time = std::chrono::duration<double>(Clock::now() - start).count();
  }
  require(output == workload.expected && (!workload.biquad || state == workload.final_state),
          "The timed opaque host reference preserves complete output and state");
  std::ranges::sort(times);
  return times[1];
}

struct Machine {
  explicit Machine(const Workload &workload, const NativeImage &image, Io *const io = nullptr)
      : core(Memory::separate({.program = program, .x = x, .y = y}), io) {
    std::ranges::copy(workload.code, program.begin() + k::entry);
    std::ranges::copy(k::irq, program.begin() + k::irq_vector);
    for (std::size_t index = 0; index < workload.input.size(); ++index) {
      x[k::input + index] = static_cast<Word>(workload.input[index]) & word_mask;
    }
    if (workload.biquad) {
      const auto state = reference::initial_state<bits>();
      for (std::size_t section = 0; section < reference::sections; ++section) {
        for (std::size_t index = 0; index < state[section].size(); ++index) {
          x[k::states + (2 * section) + index] =
              static_cast<Word>(state[section][index]) & word_mask;
        }
        for (std::size_t index = 0; index < workload.weights[section].size(); ++index) {
          y[k::coefficients + (5 * section) + index] =
              static_cast<Word>(workload.weights[section][index]) & word_mask;
        }
      }
    } else {
      for (std::size_t index = 0; index < reference::taps; ++index) {
        y[k::coefficients + index] =
            static_cast<Word>(workload.taps[reference::taps - 1 - index]) & word_mask;
      }
    }
    Registers registers;
    registers.pc = k::entry;
    registers.m.fill(word_mask);
    registers.r[0] = k::input;
    registers.r[1] = k::output;
    registers.r[4] = k::coefficients;
    registers.n[0] = reference::taps - 1;
    registers.n[4] = reference::taps;
    if (!workload.biquad) { registers.x = std::uint64_t{1} << test::word_bits; }
    core.set_registers(registers);
    core.install(image, 0);
  }

  void verify(const Workload &workload, const std::uint64_t interrupts = 0) const {
    const auto state = core.state();
    require(state.failure == Fault{} && state.execution == ExecutionState::stopped,
            "Guest terminates at the real STOP without faults");
    require(state.progress.instructions >= minimum_instructions,
            "Every measurement retires at least ten million guest instructions");
    for (std::size_t index = 0; index < workload.expected.size(); ++index) {
      if (test::signed24(y[k::output + index]) != workload.expected[index]) {
        throw std::runtime_error("Guest output mismatch at sample " + std::to_string(index));
      }
    }
    if (workload.biquad) {
      for (std::size_t section = 0; section < reference::sections; ++section) {
        for (std::size_t index = 0; index < workload.final_state[section].size(); ++index) {
          require(test::signed24(x[k::states + (2 * section) + index]) ==
                      workload.final_state[section][index],
                  "Guest preserves all final filter state words");
        }
      }
    }
    const auto per_sample = workload.biquad ? 62U : 70U;
    const auto expected =
        2 + (reference::blocks * (2 + (reference::frames * per_sample))) + (2 * interrupts);
    require(state.progress.instructions == expected,
            "Retirement denominator matches the independently counted guest program");
    require(x[0] == interrupts, "The guest IRQ handler counts every delivered interrupt");
  }

  std::vector<Word> program = std::vector<Word>(k::program_words);
  std::vector<Word> x = std::vector<Word>(k::data_words);
  std::vector<Word> y = std::vector<Word>(k::data_words);
  Core core;
};

NativeImage prepare(const Workload &workload, const bool regions, const bool observed = false) {
  const std::array spans{CodeSpan{.address = k::entry, .words = workload.code},
                         CodeSpan{.address = k::irq_vector, .words = k::irq}};
  const std::array entries{k::entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array memory{
      RegionLayout{.space = Space::program, .first = 0, .size = k::program_words},
      RegionLayout{.space = Space::x, .first = 0, .size = k::data_words},
      RegionLayout{.space = Space::y, .first = 0, .size = k::data_words}};
  const std::array layouts{VariantLayout{.id = 0, .memory = memory}};
  const std::array interrupts{InterruptRequest{.vector = k::irq_vector, .level = 0}};
  const std::array modes{Word{0}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto bindings = observed ? test::retirement_binding() : std::vector<std::byte>{};
  return Core::compile(
      {.variants = variants},
      {.memory = layouts,
       .interrupts = interrupts,
       .transitions = {},
       .callbacks = observed ? std::span(callbacks) : std::span<const CallbackBinding>{},
       .bindings = bindings,
       .region_modes = regions ? std::span<const Word>(modes) : std::span<const Word>{}});
}

enum class RunMode : std::uint8_t { continuous, bounded, interrupts };

std::string_view kernel_name(const Workload &workload, const RunMode mode) {
  if (mode == RunMode::interrupts) { return "K6"; }
  return workload.biquad ? "K2" : "K1";
}

struct RunStats {
  std::uint64_t calls = 0, interrupts = 0, maximum_irq_delay = 0;
};

RunStats execute(Core &core, const RunMode mode) {
  if (mode == RunMode::continuous) {
    static_cast<void>(core.run());
    return {.calls = 1};
  }
  RunStats stats;
  auto deadline = reference::run_cycles;
  auto next_irq = reference::interrupt_cycles;
  for (;;) {
    const auto result = core.run_until(deadline);
    ++stats.calls;
    if (result.fault != Fault{}) { throw std::runtime_error("Bounded execution fault"); }
    if (result.state == ExecutionState::stopped) { return stats; }
    if (mode == RunMode::interrupts && core.progress().estimated_cycles >= next_irq) {
      while (!core.interrupt_ready()) {
        const auto advance = core.run(1);
        ++stats.calls;
        if (advance.retired == 0 || advance.fault != Fault{}) {
          throw std::runtime_error("Delayed interrupt makes no progress");
        }
      }
      stats.maximum_irq_delay =
          std::max(stats.maximum_irq_delay, core.progress().estimated_cycles - next_irq);
      core.interrupt(k::irq_vector, 0);
      ++stats.interrupts;
      next_irq += reference::interrupt_cycles;
    }
    deadline += reference::run_cycles;
  }
}

void observe(const Workload &workload, const bool regions, const RunMode mode) {
  const auto image = prepare(workload, regions, true);
  test::RetirementIo io;
  Machine machine(workload, image, &io);
  const auto stats = execute(machine.core, mode);
  machine.verify(workload, stats.interrupts);
  const auto instructions = machine.core.progress().instructions;
  require(io.deferred <= io.batched && io.batched <= io.counts[0] &&
              io.counts[0] + io.counts[1] == instructions,
          "The separate observer accounts for every retired guest instruction");
  std::cout << R"({"observation":true,"kernel":")" << kernel_name(workload, mode)
            << R"(","regions_enabled":)" << (regions ? "true" : "false") << R"(,"bounded":)"
            << (mode == RunMode::continuous ? "false" : "true") << R"(,"interrupts":)"
            << stats.interrupts << R"(,"region_instructions":)" << io.counts[0]
            << R"(,"precise_instructions":)" << io.counts[1] << R"(,"batched_instructions":)"
            << io.batched << R"(,"deferred_instructions":)" << io.deferred << "}\n"
            << std::flush;
}

void measure(const Workload &workload,
             const NativeImage &image,
             const bool regions,
             const RunMode mode,
             const double preparation) {
  Machine warm(workload, image);
  const auto warm_stats = execute(warm.core, mode);
  warm.verify(workload, warm_stats.interrupts);
  Machine machine(workload, image);
  reference::Counters counters;
  counters.start();
  const auto start = Clock::now();
  const auto stats = execute(machine.core, mode);
  const auto seconds = std::chrono::duration<double>(Clock::now() - start).count();
  const auto counts = counters.stop();
  machine.verify(workload, stats.interrupts);
  const auto reference_seconds = host_time(workload);
  const auto progress = machine.core.progress();
  const auto instructions = static_cast<double>(progress.instructions);
  std::cout << std::setprecision(10) << R"({"kernel":")" << kernel_name(workload, mode)
            << R"(","regions_enabled":)" << (regions ? "true" : "false") << R"(,"bounded":)"
            << (mode == RunMode::continuous ? "false" : "true") << R"(,"preparation_s":)"
            << preparation << R"(,"native_bytes":)" << image.report().native_bytes
            << R"(,"prepared_guest_sites":)" << image.report().prepared_entries
            << R"(,"foreground_guest_sites":)" << workload.code.size() - (workload.biquad ? 2U : 3U)
            << R"(,"guest_instructions":)" << progress.instructions << R"(,"guest_cycles":)"
            << progress.estimated_cycles << R"(,"guest_instructions_per_sample":)"
            << instructions / reference::samples << R"(,"guest_cycles_per_sample":)"
            << static_cast<double>(progress.estimated_cycles) / reference::samples << R"(,"calls":)"
            << stats.calls << R"(,"interrupts":)" << stats.interrupts
            << R"(,"maximum_irq_delay_cycles":)" << stats.maximum_irq_delay << R"(,"seconds":)"
            << seconds << R"(,"host_reference_s":)" << reference_seconds
            << R"(,"guest_over_host_time":)" << seconds / reference_seconds << R"(,"ns_per_guest":)"
            << seconds * nanoseconds / instructions;
  if (counts) {
    std::cout << R"(,"host_instructions":)" << counts->instructions << R"(,"host_cycles":)"
              << counts->cycles << R"(,"pmc_enabled_ns":)" << counts->enabled_ns
              << R"(,"pmc_running_ns":)" << counts->running_ns
              << R"(,"host_instructions_per_guest":)"
              << static_cast<double>(counts->instructions) / instructions
              << R"(,"host_cycles_per_guest":)"
              << static_cast<double>(counts->cycles) / instructions;
  } else {
    std::cout << R"(,"host_instructions_per_guest":null,"host_cycles_per_guest":null)";
  }
  std::cout << "}\n" << std::flush;
}
}

int main() {
  try {
    for (const bool biquad : {false, true}) {
      const Workload workload(biquad);
      for (const bool regions : {false, true}) {
        const auto preparing = Clock::now();
        const auto image = prepare(workload, regions);
        const auto preparation = std::chrono::duration<double>(Clock::now() - preparing).count();
        measure(workload, image, regions, RunMode::continuous, preparation);
        if (!biquad) {
          for (const auto mode : {RunMode::bounded, RunMode::interrupts}) {
            measure(workload, image, regions, mode, 0);
          }
        }
        observe(workload, regions, RunMode::continuous);
        if (!biquad) { observe(workload, regions, RunMode::interrupts); }
      }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
