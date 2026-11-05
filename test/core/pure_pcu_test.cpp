// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using staged_test::backends;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using unit_publication_test::field;
constexpr auto pcu = InstructionPublication::pcu_updates;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              pcu};
// FM Rev.5 table A-1, A-4: short JMP=3; absolute JMP=3+lab=4.
constexpr Word short_jump = 0x0c0000, absolute_jump = 0x0af080, short_targets = 4096;
constexpr std::array addresses{Word{0}, short_targets - 1, word_mask};
constexpr std::array statuses{Word{0}, Word{0xc00000}, Word{0x4003ff}};

struct Case {
  InstructionWords words;
  Word target = 0;
  unsigned cycles = 1;
  bool transfer = false;
};

Registers seed(const InstructionWords words, const Word status) {
  auto state = unit_publication_test::seed();
  state.pc = words.address;
  state.sr = status;
  return state;
}

InstructionEffect publication(const Case &test, const Registers &before) {
  return {.kind = EffectKind::pcu_update,
          .pc = before.pc,
          .omr = before.omr,
          .pcu_update = PcuWriteback{
              .opcode = test.words.opcode,
              .count = 1,
              .updates = {{field(RegisterStorage::pc, word_mask, before.pc, test.target)}}}};
}

void finish(Core &core, const Case &test, const Registers &before, const Progress prior) {
  auto expected = before;
  expected.pc = test.target;
  auto action = pending(core);
  require(action.computed == expected && action.opcode == test.words.opcode,
          "Captured native result matches independent PC, flags, AGU, accumulators and stack");
  if (action.effect.kind == EffectKind::pcu_update) {
    require(action.effect == publication(test, before),
            "Only the full-width PC writer is published with its original value and context");
    require(core.registers() == before, "The unaccepted PC publication stays private");
    require(!core.advance_instruction(action.ticket).retired && core.registers() == expected &&
                core.progress() == prior,
            "Accepted PC changes once; hidden instruction and base-cost accounting stay held");
    rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
    action = pending(core);
  }
  require(action.effect == InstructionEffect{.kind = EffectKind::retire,
                                             .address = before.pc,
                                             .value = test.target,
                                             .pc = before.pc,
                                             .omr = before.omr} &&
              !core.instruction_outcome(),
          "Retirement context is exact and no outcome is exposed before actual retirement");
  const auto receipt = core.advance_instruction(action.ticket);
  const Progress completed{.instructions = prior.instructions + 1,
                           .estimated_cycles = prior.estimated_cycles + test.cycles};
  require(
      receipt.retired && receipt.fault.kind == FaultKind::none && receipt.progress == completed &&
          core.registers() == expected &&
          core.instruction_outcome() == InstructionOutcome{.instructions = completed.instructions,
                                                           .address = before.pc,
                                                           .opcode = test.words.opcode,
                                                           .control_transfer = test.transfer},
      "Native retirement preserves complete state, literal base cost and taken-transfer identity");
}

void execute(Core &core, const Case &test, const Word status, const InstructionPublication policy) {
  const auto before = seed(test.words, status);
  core.set_registers(before);
  const auto prior = core.progress();
  const auto begun = core.begin_instruction(test.words, policy);
  require(begun.pending && begun.fault.kind == FaultKind::none && begun.progress == prior &&
              core.registers() == before,
          "Captured NOP/JMP computation holds every architectural field and counter");
  require(pending(core).effect.kind ==
              (policy == pcu ? EffectKind::pcu_update : EffectKind::retire),
          "Each publication policy retains exactly its documented boundary");
  finish(core, test, before, prior);
}

void matrix(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  for (const auto policy : policies) {
    for (Word target = 0; target < short_targets; ++target) {
      const auto slot = target % addresses.size();
      execute(core,
              {.words = {.address = addresses.at(slot), .opcode = short_jump | target},
               .target = target,
               .cycles = 3,
               .transfer = true},
              statuses.at(slot),
              policy);
    }
    for (const auto address : addresses) {
      for (const auto status : statuses) {
        execute(core,
                {.words = {.address = address}, .target = (address + 1) & word_mask},
                status,
                policy);
        for (const auto target : std::array{Word{0}, word_mask, (address + 2) & word_mask}) {
          execute(core,
                  {.words = {.address = address, .opcode = absolute_jump, .extension = target},
                   .target = target,
                   .cycles = 4,
                   .transfer = true},
                  status,
                  policy);
        }
      }
    }
  }
  require(bus.calls == 0,
          "Pure captured control never reads or fabricates an external bus operand");
}

void continuation(const Backend backend) {
  constexpr std::array cases{
      Case{.words = {.address = word_mask}, .target = 0},
      Case{.words = {.address = 0, .opcode = short_jump},
           .target = 0,
           .cycles = 3,
           .transfer = true},
      Case{.words = {.address = word_mask, .opcode = absolute_jump, .extension = 1},
           .target = 1,
           .cycles = 4,
           .transfer = true}};
  Bus bus;
  Core original(bus, {.backend = backend});
  for (const auto &test : cases) {
    for (unsigned cut = 0; cut < 2; ++cut) {
      const auto before = seed(test.words, statuses[1]);
      original.set_registers(before);
      const auto prior = original.progress();
      static_cast<void>(original.begin_instruction(test.words, pcu));
      if (cut != 0) { static_cast<void>(original.advance_instruction(pending(original).ticket)); }
      finish(original, test, before, prior);
    }
  }
  require(bus.calls == 0,
          "Continuing actual pure-control instructions needs no hidden external response");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend : dsp56300::test::backends) {
      dsp56300::test::matrix(backend);
      dsp56300::test::continuation(backend);
    }
    std::cout << dsp56300::test::checks << " captured pure control checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
