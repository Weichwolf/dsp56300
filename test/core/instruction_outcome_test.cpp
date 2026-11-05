// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
using dsp56300::test::staged_test::backends;
using dsp56300::test::staged_test::Bus;
using dsp56300::test::staged_test::pending;
constexpr Word address = 0x100;
constexpr Word conditional = 0x0e0000;
constexpr Word ordinary = 0xc00300;
constexpr unsigned conditions = 16;
constexpr unsigned flags = 256;
constexpr unsigned effect_limit = 32;
constexpr auto policy = InstructionPublication::pcu_updates;

struct Condition {
  unsigned code;
  Word ccr;
};

bool truth(const Condition input) {
  // Independent FM12.5.1/table12-17 Boolean conditions and table12-18 CCR bits.
  const bool carry = (input.ccr & 1U) != 0;
  const bool overflow = (input.ccr & 2U) != 0;
  const bool zero = (input.ccr & 4U) != 0;
  const bool negative = (input.ccr & 8U) != 0;
  const bool unnormalized = (input.ccr & 16U) != 0;
  constexpr Word extension_mask = 32;
  constexpr Word limit_mask = 64;
  const bool extension = (input.ccr & extension_mask) != 0;
  const bool limit = (input.ccr & limit_mask) != 0;
  const std::array predicates{!carry,
                              negative == overflow,
                              !zero,
                              !negative,
                              !zero && (unnormalized || extension),
                              !extension,
                              !limit,
                              !zero && negative == overflow,
                              carry,
                              negative != overflow,
                              zero,
                              negative,
                              zero || (!unnormalized && !extension),
                              extension,
                              limit,
                              zero || negative != overflow};
  return predicates.at(input.code);
}

void complete(Core &core) {
  for (unsigned effect = 0; effect < effect_limit; ++effect) {
    require(!core.instruction_outcome(),
            "A pending instruction cannot expose a retirement receipt");
    const auto next = pending(core);
    const auto result = core.advance_instruction(next.ticket);
    require(result.fault.kind == FaultKind::none, "The actual staged instruction remains healthy");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Staged outcome instruction exceeded its bounded effects");
}

void same_target(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  for (unsigned code = 0; code < conditions; ++code) {
    for (Word ccr = 0; ccr < flags; ++ccr) {
      auto initial = dsp56300::test::staged_test::seed();
      initial.pc = address;
      initial.sr = ordinary | ccr;
      core.set_registers(initial);
      require(!core.instruction_outcome(), "Register initialization retires no instruction");
      const auto count = core.progress().instructions;
      constexpr unsigned condition_shift = 12;
      const Word opcode = conditional | (code << condition_shift) | (address + 1);
      static_cast<void>(core.begin_instruction({.address = address, .opcode = opcode}, policy));
      const InstructionOutcome expected{.instructions = count + 1,
                                        .address = address,
                                        .opcode = opcode,
                                        .control_transfer = truth({.code = code, .ccr = ccr})};
      complete(core);
      auto expected_registers = initial;
      ++expected_registers.pc;
      require(
          core.registers() == expected_registers && core.instruction_outcome() == expected &&
              core.progress().instructions == count + 1 && bus.calls == 0,
          "Same-address Jcc distinguishes actual taken control flow despite identical PC values");
    }
  }
}

void publications(const Backend backend) {
  constexpr std::array policies{InstructionPublication::retirement,
                                InstructionPublication::address_updates,
                                InstructionPublication::move_updates,
                                InstructionPublication::alu_updates,
                                InstructionPublication::unit_updates,
                                policy};
  for (const auto publication : policies) {
    for (const bool taken : {false, true}) {
      Bus bus;
      Core core(bus, {.backend = backend});
      auto initial = dsp56300::test::staged_test::seed();
      initial.pc = address;
      initial.sr = ordinary | (taken ? 0U : 1U);
      core.set_registers(initial);
      const auto opcode = conditional | (address + 1);
      static_cast<void>(
          core.begin_instruction({.address = address, .opcode = opcode}, publication));
      complete(core);
      const InstructionOutcome expected{
          .instructions = 1, .address = address, .opcode = opcode, .control_transfer = taken};
      require(
          core.instruction_outcome() == expected,
          "Every publication policy reports its actual staged control transfer after retirement");
    }
  }
}

void speculative_candidate(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto initial = dsp56300::test::staged_test::seed();
  initial.pc = address;
  initial.sr = ordinary;
  core.set_registers(initial);
  static_cast<void>(
      core.begin_instruction({.address = address, .opcode = conditional | (address + 1)}, policy));
  const auto published = core.advance_instruction(pending(core).ticket);
  require(!published.retired && pending(core).effect.kind == EffectKind::retire &&
              !core.instruction_outcome(),
          "Published PC alone does not establish actual instruction retirement");
  complete(core);
  const auto receipt = core.instruction_outcome();
  require(receipt && receipt->control_transfer, "The actual retirement supplies its own receipt");
  core.reset();
  require(!core.instruction_outcome(), "Reset removes the prior retirement receipt");
}

void atomic_invalidation(const Backend backend) {
  std::array<Word, 8> p{};
  std::array<Word, 8> x{};
  std::array<Word, 8> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  static_cast<void>(core.begin_instruction({.address = 0, .opcode = 0}, policy));
  complete(core);
  const auto receipt = core.instruction_outcome();
  require(receipt && !receipt->control_transfer,
          "A sequential NOP has its own non-transfer receipt");
  static_cast<void>(core.run(0));
  require(core.instruction_outcome() == receipt, "Zero budget executes no atomic instruction");
  static_cast<void>(core.run(1));
  require(!core.instruction_outcome(), "Atomic execution cannot reuse a stale staged receipt");
}

void loop_redirects(const Backend backend) {
  constexpr Word do_immediate = 0x060080;
  constexpr Word loop_flag = 0x8000;
  constexpr unsigned count_shift = 8;
  constexpr unsigned word_bits = 24;
  for (const Word count : {0U, 1U, 2U}) {
    Bus bus;
    Core core(bus, {.backend = backend});
    auto expected = dsp56300::test::staged_test::seed();
    expected.pc = address;
    expected.sr = ordinary;
    core.set_registers(expected);
    const auto initial = expected;
    const Word opcode = do_immediate | (count << count_shift);
    static_cast<void>(core.begin_instruction(
        {.address = address, .opcode = opcode, .extension = address + (count == 0 ? 1U : 2U)},
        policy));
    complete(core);
    expected.pc = address + 2;
    expected.stack[1] = (std::uint64_t{expected.la} << word_bits) | expected.lc;
    if (count != 0) {
      expected.sp = expected.sc = 2;
      expected.stack[2] = (std::uint64_t{address + 2} << word_bits) | ordinary;
      expected.lc = count;
      expected.la = address + 2;
      expected.sr |= loop_flag;
    }
    require(core.registers() == expected &&
                core.instruction_outcome() == InstructionOutcome{.instructions = 1,
                                                                 .address = address,
                                                                 .opcode = opcode,
                                                                 .control_transfer = count == 0},
            "Zero DO skips to its sequential successor but still reports a real redirect");
    for (Word body = 0; body < count; ++body) {
      static_cast<void>(core.begin_instruction({.address = address + 2, .opcode = 0}, policy));
      complete(core);
      const bool again = body + 1 < count;
      if (again) {
        --expected.lc;
      } else {
        expected.pc = address + 3;
        expected.sp = expected.sc = 0;
        expected.sr = ordinary;
        expected.la = initial.la;
        expected.lc = initial.lc;
      }
      require(core.registers() == expected &&
                  core.instruction_outcome() == InstructionOutcome{.instructions = body + 2,
                                                                   .address = address + 2,
                                                                   .opcode = 0,
                                                                   .control_transfer = again},
              "DO backedges redirect; completed-loop restoration keeps its sequential outcome");
    }
    require(bus.calls == 0, "Loop outcome inspection invents no external transaction");
  }
}
}

int main() {
  try {
    for (const auto backend : backends) {
      same_target(backend);
      publications(backend);
      speculative_candidate(backend);
      atomic_invalidation(backend);
      loop_redirects(backend);
    }
    std::cout << "12288 native control-flow outcomes, publication policies and receipt boundaries "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
