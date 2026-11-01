// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
using dsp56300::test::staged_test::Bus;
constexpr Word vector = 0x40, foreground = 0x100;
// FM12-16/A.3.7: MOVEM cannot read SSH or load SC/SP/SSH/SSL/LA/LC in fast vectors.
constexpr std::array prohibited{
    0x0760bcU, 0x07e0b1U, 0x07e0bbU, 0x07e0bcU, 0x07e0bdU, 0x07e0beU, 0x07e0bfU};
constexpr Word long_x0 = 0x44f400;
constexpr Word short_call = 0x0d0300;
constexpr unsigned effect_bound = 32;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};

Registers initial() {
  auto result = dsp56300::test::initial_state();
  result.pc = foreground;
  result.sr = 0;
  result.r[0] = vector + 1;
  return result;
}

InstructionProgress finish(Core &core, const InstructionPublication policy) {
  auto result = core.begin_fetch(policy);
  for (unsigned effect = 0; effect < effect_bound && result.pending && !result.retired &&
                            result.fault.kind == FaultKind::none;
       ++effect) {
    result = core.advance_instruction(result.pending->ticket);
  }
  return result;
}

void prohibited_move(const Backend backend,
                     const InstructionPublication policy,
                     const Word opcode) {
  Bus bus;
  bus.pc = vector;
  bus.events = {{.space = Space::program, .address = vector, .value = opcode},
                {.space = Space::program, .address = vector + 1, .value = 0}};
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = 0});
  const auto result = finish(core, policy);
  require(result.fault == Fault{.kind = FaultKind::unsupported_instruction,
                                .space = Space::program,
                                .address = vector,
                                .value = opcode,
                                .pc = vector} &&
              core.registers() == initial() && core.progress() == Progress{} && bus.exact &&
              bus.calls == 2 && !core.instruction_outcome(),
          "Prohibited MOVEM rejects before any stack/control mutation or operand transaction");
}

void excessive_second_slot(const Backend backend, const InstructionPublication policy) {
  Bus bus;
  bus.pc = vector;
  bus.events = {{.space = Space::program, .address = vector, .value = 0},
                {.space = Space::program, .address = vector + 1, .value = long_x0}};
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = 0});
  require(finish(core, policy).retired && core.progress().instructions == 1,
          "The first real NOP retires with both accepted vector words");
  const auto result = finish(core, policy);
  require(result.fault == Fault{.kind = FaultKind::unsupported_instruction,
                                .space = Space::program,
                                .address = vector + 1,
                                .value = long_x0,
                                .pc = vector + 1} &&
              core.registers() == initial() && core.progress().instructions == 1 && bus.exact &&
              bus.calls == 2 && !core.instruction_outcome(),
          "A two-word second slot fails before offering any third vector-word read");
}

void atomic_prohibited_move(const Backend backend, const Word opcode) {
  std::array<Word, dsp56300::test::memory_words> program{};
  std::array<Word, dsp56300::test::memory_words> x{};
  std::array<Word, dsp56300::test::memory_words> y{};
  program.at(vector) = opcode;
  Core core({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
  core.set_registers(initial());
  core.interrupt(vector, 0);
  require(core.fault() == Fault{.kind = FaultKind::unsupported_instruction,
                                .space = Space::program,
                                .address = vector,
                                .value = opcode,
                                .pc = vector} &&
              core.registers() == initial() && core.progress() == Progress{},
          "Atomic vector entry also rejects prohibited MOVEM before native operand effects");
}

void refused_call(const Backend backend, const InstructionPublication policy) {
  Bus bus;
  bus.pc = vector;
  bus.events = {{.space = Space::program, .address = vector, .value = short_call},
                {.space = Space::program, .address = vector + 1, .value = 0, .accept = false}};
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = 0});
  const auto result = finish(core, policy);
  require(result.fault == Fault{.kind = FaultKind::memory_read,
                                .space = Space::program,
                                .address = vector + 1,
                                .pc = vector} &&
              core.registers() == initial() && core.progress() == Progress{} && bus.exact &&
              bus.calls == 2 && bus.accepted == 1 && !core.instruction_outcome(),
          "A short JSR cannot push any frame before its second real vector read succeeds");
}

void atomic_refused_call(const Backend backend) {
  std::array<Word, vector + 1> program{};
  std::array<Word, dsp56300::test::memory_words> x{};
  std::array<Word, dsp56300::test::memory_words> y{};
  program.at(vector) = short_call;
  Core core({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
  core.set_registers(initial());
  core.interrupt(vector, 0);
  require(core.fault() == Fault{.kind = FaultKind::memory_read,
                                .space = Space::program,
                                .address = vector + 1,
                                .pc = vector} &&
              core.registers() == initial() && core.progress() == Progress{},
          "Atomic short JSR also refuses a missing second word before any frame push");
}
}

int main() {
  try {
    unsigned cases = 0;
    for (const auto backend : dsp56300::test::staged_test::backends) {
      for (const auto policy : policies) {
        for (const auto opcode : prohibited) {
          prohibited_move(backend, policy, opcode);
          ++cases;
        }
        excessive_second_slot(backend, policy);
        ++cases;
        refused_call(backend, policy);
        ++cases;
      }
      for (const auto opcode : prohibited) {
        atomic_prohibited_move(backend, opcode);
        ++cases;
      }
      atomic_refused_call(backend);
      ++cases;
    }
    constexpr unsigned expected_cases = 186;
    require(cases == expected_cases, "Every fixed vector restriction case executes");
    std::cout << "186 native vector MOVEM/length/frame guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
