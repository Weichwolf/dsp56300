// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
using dsp56300::test::staged_test::Bus;
using dsp56300::test::staged_test::pending;
using dsp56300::test::staged_test::rejected;
constexpr Word entry = 0x100;
constexpr Word operand = 0xffffc0;
constexpr Word ordinary = 0xc00300;
// FM JSET high-pp X:ffffc0,bit0; target deliberately equals the two-word fallthrough.
constexpr Word test_and_jump = 0x0a80a0;
constexpr unsigned bound = 32;
constexpr unsigned expected_cases = 36;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};

void complete(Core &core) {
  for (unsigned effect = 0; effect < bound; ++effect) {
    require(!core.instruction_outcome(), "No input or published result is a retirement receipt");
    const auto result = core.advance_instruction(pending(core).ticket);
    require(result.fault.kind == FaultKind::none, "Accepted native input remains healthy");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Native bit-flow result exceeded its bounded effects");
}

void prepare(Core &core, Bus &bus, const InstructionPublication publication, const bool level) {
  auto initial = dsp56300::test::staged_test::seed();
  initial.pc = entry;
  initial.sr = ordinary;
  core.set_registers(initial);
  static_cast<void>(core.begin_instruction({.address = entry, .opcode = 0}, publication));
  complete(core);
  require(core.instruction_outcome() ==
              InstructionOutcome{.instructions = 1, .address = entry, .opcode = 0},
          "A prior genuine NOP supplies the receipt that must never leak into later work");
  bus.pc = entry + 1;
  bus.events = {{.kind = EffectKind::bit,
                 .space = Space::x,
                 .address = operand,
                 .value = level ? 1U : 0U,
                 .action = BitAction::test}};
  static_cast<void>(core.begin_instruction(
      {.address = entry + 1, .opcode = test_and_jump, .extension = entry + 3}, publication));
  require(pending(core).effect.kind == EffectKind::bit && bus.calls == 0 &&
              !core.instruction_outcome(),
          "The real current bit request has not consumed input or exposed the prior NOP");
}

InstructionOutcome expected_outcome(const bool level) {
  return {
      .instructions = 2, .address = entry + 1, .opcode = test_and_jump, .control_transfer = level};
}

void accepted_input(const Backend backend,
                    const InstructionPublication publication,
                    const bool level) {
  Bus bus;
  Core core(bus, {.backend = backend});
  prepare(core, bus, publication, level);
  auto expected_registers = core.registers();
  expected_registers.pc = entry + 3;
  const auto input = core.advance_instruction(pending(core).ticket);
  require(input.fault.kind == FaultKind::none && !input.retired && !core.instruction_outcome() &&
              bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "A real accepted bit alone cannot publish a control-transfer receipt");
  const auto saved = core.checkpoint();
  bus.events.front().value = level ? 0U : 1U;
  Bus clone_bus = bus;
  const auto other = backend == Backend::reference ? Backend::cached : Backend::reference;
  Core clone(clone_bus, {.backend = other});
  clone.restore(saved);
  require(!clone.instruction_outcome(), "Restoring after input preserves the unretired boundary");
  complete(core);
  complete(clone);
  const auto expected = expected_outcome(level);
  require(core.registers() == expected_registers && clone.registers() == expected_registers &&
              core.instruction_outcome() == expected && clone.instruction_outcome() == expected &&
              bus.calls == 1 && clone_bus.calls == 1 && bus.exact && clone_bus.exact,
          "Current external changes cannot replace the once-accepted branch input on either owner");
  const auto retired = core.checkpoint();
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = other});
  resumed.restore(retired);
  require(resumed.instruction_outcome() == expected && resumed.registers() == expected_registers &&
              resumed_bus.calls == 1,
          "A fresh stopped owner restores the actual receipt without any new I/O");
}

void refused_input(const Backend backend,
                   const InstructionPublication publication,
                   const bool level) {
  Bus bus;
  Core core(bus, {.backend = backend});
  prepare(core, bus, publication, level);
  bus.events.front().accept = false;
  const auto saved = core.checkpoint();
  const auto input_bus = bus;
  const auto failed = core.advance_instruction(pending(core).ticket);
  const Fault expected{
      .kind = FaultKind::io_bit, .space = Space::x, .address = operand, .pc = entry + 1};
  require(failed.fault == expected && !failed.retired && !core.instruction_outcome() &&
              core.progress().instructions == 1 && bus.calls == 1 && bus.accepted == 0 && bus.exact,
          "Refused native input cannot expose either a taken transfer or the previous NOP receipt");
  rejected([&] { static_cast<void>(core.begin_fetch(publication)); });
  const auto faulted = core.checkpoint();
  Bus stopped_bus = bus;
  Core stopped(stopped_bus, {.backend = backend});
  stopped.restore(faulted);
  require(!stopped.instruction_outcome() && stopped.fault() == expected && stopped_bus.calls == 1,
          "A restored fault cannot revive the numerically matching prior instruction count");
  core.restore(saved);
  bus = input_bus;
  bus.events.front().accept = true;
  require(!core.instruction_outcome() && bus.calls == 0,
          "Trusted pre-input restore does not fabricate an accepted result");
  complete(core);
  require(core.instruction_outcome() == expected_outcome(level) && bus.calls == 1 &&
              bus.accepted == 1 && bus.exact,
          "Only a new actual accepted callback permits the recovered native transfer receipt");
}
}

int main() {
  try {
    unsigned cases = 0;
    for (const auto backend : dsp56300::test::staged_test::backends) {
      for (const auto publication : policies) {
        for (const bool level : {false, true}) {
          accepted_input(backend, publication, level);
          refused_input(backend, publication, level);
          ++cases;
        }
      }
    }
    require(cases == expected_cases, "All backend/publication/input combinations must execute");
    std::cout
        << "36 native bit-flow input cases, fault suppression and restored receipts passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
