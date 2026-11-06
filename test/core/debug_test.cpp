// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "debug_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace dsp56300::test {
namespace {
using debug_model::Input;
constexpr Word words = 2048, external = 1024, host_vector = 64;
constexpr Word mac_a = isa::opcode<isa::Opcode::mac_register_a>()[0] | (5U << 4);
constexpr Word mac_b = mac_a | 8U;
constexpr Word adc_a = isa::opcode<isa::Opcode::adc_x_a>()[0];
constexpr Word round_a = isa::opcode<isa::Opcode::rnd_a>()[0];
constexpr Word ce = Word{1} << 19, be = Word{1} << 10;
constexpr Word invalid = isa::opcode<isa::Opcode::illegal>()[0];

struct Machine {
  std::array<Word, words> p{};
  std::array<Word, 1> x{}, y{};
  TestCore core;

  explicit Machine(const Backend backend)
      : core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend}) {}
};

template <typename Action> bool rejected(Action action) {
  try {
    action();
  } catch (const std::exception &) { return true; }
  return false;
}

void alu(Registers &state, const Word operation) {
  const auto old = state;
  sequence_model::alu(state, old, operation);
  ++state.pc;
}

void quiescent(TestCore &core, const Registers &state, const Progress progress) {
  for (const auto budget : {0U, std::numeric_limits<std::uint32_t>::max()}) {
    const auto result = core.run(budget);
    require(result.retired == 0 && result.state == ExecutionState::debugging &&
                result.fault.kind == FaultKind::none && result.progress == progress &&
                core.registers() == state && !core.interrupt_ready(),
            "Debug halt cannot execute, fetch a successor or consume an arbitrary budget");
  }
  core.clear_code_cache();
  core.set_registers(state);
  require(core.processing_state() == ExecutionState::debugging && core.run(1).retired == 0 &&
              core.registers() == state,
          "Cache clearing and visible-register edits preserve debug mode and cause");
  require(rejected([&core] { core.interrupt(host_vector, 3); }) &&
              rejected([&core] { core.resume_from_stop(); }),
          "Ordinary IRQ and STOP completion cannot replace a halt");
}

void native_debug(const Backend backend, const Input input) {
  Machine machine(backend);
  auto &core = machine.core;
  const Word debug =
      input.form == 0 ? isa::debug[0] : isa::opcode<isa::Opcode::debug_cc>()[0] | (input.form - 1);
  const std::array code{mac_a, mac_b, debug, adc_a, round_a, invalid};
  core.load(Space::program, start, code);
  auto initial = debug_model::seed(input);
  auto warm = initial;
  warm.pc = start + 3;
  warm.sr = 0;
  core.set_registers(warm);
  require(core.run(2).retired == 2, "Warm the debug continuation under an old mode");
  core.set_registers(initial);

  auto expected = initial;
  alu(expected, mac_a);
  alu(expected, mac_b);
  const bool requested = debug_model::taken(input, expected.sr);
  ++expected.pc;
  const Word prefix_cost = 4 + debug_model::cycles(input);
  const auto result = core.run(3);
  const auto state = requested ? ExecutionState::debugging : ExecutionState::running;
  require(result.retired == 3 && result.state == state && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 5, .estimated_cycles = prefix_cost} &&
              core.registers() == expected && core.interrupt_ready() == !requested,
          "DEBUGcc consumes freshly materialized native flags and preserves the complete state");
  if (requested) {
    quiescent(core, expected, result.progress);
    core.reset();
    require(core.processing_state() == ExecutionState::running && core.progress() == Progress{} &&
                core.fault().kind == FaultKind::none,
            "Architectural reset recovers a genuine guest DEBUG halt");
    return;
  }
  alu(expected, adc_a);
  alu(expected, round_a);
  const auto continued = core.run(2);
  require(continued.retired == 2 && continued.state == ExecutionState::running &&
              continued.fault.kind == FaultKind::none && core.registers() == expected &&
              continued.progress ==
                  Progress{.instructions = 7, .estimated_cycles = prefix_cost + 2},
          "An untaken DEBUGcc preserves exact native continuation and full architectural state");
}

void fast_halt(const Backend backend, const bool second_slot) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code =
      second_slot ? std::array{mac_a, isa::debug[0]} : std::array{isa::debug[0], mac_b};
  core.load(Space::program, host_vector, code);
  const Input input{.status = mode_rm | mode_down | flag_c, .pointer = 2};
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  if (second_slot) {
    const auto old = expected;
    sequence_model::alu(expected, old, mac_a);
  }
  core.interrupt(host_vector, 0);
  const Progress progress{.instructions = second_slot ? 2U : 1U,
                          .estimated_cycles = second_slot ? 2U : 1U};
  require(core.processing_state() == ExecutionState::debugging && core.registers() == expected &&
              core.progress() == progress,
          "DEBUG in either fast slot pauses with the foreground PC and exact vector progress");
  quiescent(core, expected, progress);
}

void cached_halt(const Backend backend) {
  std::array<Word, words> memory{};
  TestCore core({.program = memory, .x = memory, .y = memory},
                nullptr,
                {.backend = backend, .cacheable_begin = external, .cacheable_end = words});
  core.load(Space::program, external, std::array{isa::debug[0], mac_a, round_a, Word{0}});
  auto expected = debug_model::seed({.status = ce | flag_c, .pointer = 2});
  expected.pc = external;
  expected.omr |= be;
  core.set_registers(expected);
  ++expected.pc;
  require(core.run(1).state == ExecutionState::debugging,
          "Cache-enabled program enters debug halt");
  core.write(Space::program, external + 1, adc_a);
  quiescent(core, expected, {.instructions = 1, .estimated_cycles = 1});
}

void native_matrix() {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    for (const auto mode : debug_model::modes) {
      for (const Word flags : {0U, status_byte}) {
        auto input = debug_model::input({.form = form, .flags = flags});
        input.status = mode | flags;
        native_debug(Backend::jit, input);
      }
    }
  }
}

void repeated_halt() {
  Machine machine(Backend::jit);
  constexpr Word count = 4;
  const Word repeat = isa::opcode<isa::Opcode::rep_immediate>()[0] | (count << 8);
  machine.core.load(Space::program, start, std::array{repeat, isa::debug[0], mac_a});
  auto expected = debug_model::seed({});
  machine.core.set_registers(expected);
  expected.pc = start + 1;
  expected.lc = count - 1;
  const auto result = machine.core.run(count + 2);
  require(result.retired == 2 && result.state == ExecutionState::debugging &&
              result.fault.kind == FaultKind::none && machine.core.registers() == expected &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 6},
          "REP DEBUG halts at its first real body without consuming subsequent iterations");
  quiescent(machine.core, expected, result.progress);
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    native_matrix();
    fast_halt(dsp56300::Backend::jit, false);
    fast_halt(dsp56300::Backend::jit, true);
    repeated_halt();
    cached_halt(dsp56300::Backend::jit);
    std::cout << checks << " public guest DEBUG checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
