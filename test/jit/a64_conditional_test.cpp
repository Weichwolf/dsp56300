// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace dsp56300::test {
namespace {
constexpr std::array raw{std::uint64_t{0},
                         std::uint64_t{1},
                         saturation_model::positive,
                         saturation_model::positive + 1,
                         saturation_model::negative - 1,
                         saturation_model::negative,
                         sign56 - 1,
                         sign56,
                         accumulator_mask};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Word cm = Word{1} << 13, mul = Word{1} << 14, a16 = Word{1} << 17;

struct Seed {
  Word status;
  std::size_t index;
};

Registers seed(const Seed input) {
  auto state = register_model::seed(0);
  state.pc = start;
  state.sr = input.status;
  state.a = raw.at(input.index % raw.size());
  state.b = raw.at((input.index + 3) % raw.size());
  constexpr std::array banks{std::uint64_t{0},
                             std::uint64_t{0x800000800000},
                             std::uint64_t{0x7fffff7fffff},
                             std::uint64_t{0xffffff000001}};
  state.x = banks.at(input.index % banks.size());
  state.y = banks.at((input.index + 1) % banks.size());
  return state;
}

void fail(a64::Expected &expected, const FaultKind kind, const Word value) {
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
  expected.failure = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

void empty(const a64::Program &program, const a64::Instruction input) {
  const auto initial = seed({.status = input.mode | status_byte, .index = 0});
  a64::Invocation invocation(initial);
  invocation.run(program, 0);
  invocation.expect(
      {.reg = initial, .retired = 0, .cycles = 0, .inhibited = true, .repeat_completed = true},
      input.opcode);
}

struct Alu {
  a64::Instruction instruction;
  std::optional<unsigned> condition = std::nullopt;
  bool update = true;
};

a64::Expected alu_expected(const Registers &initial, const Alu point) {
  a64::Expected expected{.reg = initial};
  ++expected.reg.pc;
  const auto status = initial.sr;
  const auto code = point.instruction.opcode % 256;
  const bool invalid = code == 8 || code == 4 || code == 12;
  if ((status & (cm | mul | a16)) != 0 ||
      (status & (mode_down | mode_up)) == (mode_down | mode_up) ||
      (invalid && (status & saturation_model::mode) != 0)) {
    fail(expected, FaultKind::unsupported_mode, status);
  } else if (!point.condition ||
             selection_model::test_condition({.status = status, .selector = *point.condition})) {
    if (invalid) {
      fail(expected,
           code == 8 ? FaultKind::invalid_instruction : FaultKind::unsupported_instruction,
           code);
    } else {
      a64_alu_model::apply(expected.reg, code);
      if (!point.update) { expected.reg.sr = status; }
    }
  }
  return expected;
}

void alu_case(const Alu point) {
  const a64::Program program(point.instruction);
  empty(program, point.instruction);
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (std::size_t index = 0; index < raw.size(); ++index) {
      const auto initial = seed({.status = point.instruction.mode | flags, .index = index});
      const auto expected = alu_expected(initial, point);
      a64::Invocation invocation(initial);
      invocation.run(program);
      invocation.expect(expected, point.instruction.opcode);
    }
  }
}

void all_alu_bytes() {
  for (Word code = 0; code <= status_byte; ++code) {
    for (const auto mode : modes) {
      for (const Word saturation : {0U, saturation_model::mode}) {
        alu_case({.instruction = {.opcode = parallel_none | code, .mode = mode | saturation}});
      }
    }
  }
}

void conditional_alu() {
  constexpr Word conditional_group = 0x202000;
  for (Word code = 0; code <= status_byte; ++code) {
    for (unsigned condition = 0; condition < 16; ++condition) {
      for (const Word mode :
           {0U, saturation_model::mode, saturation_model::mode | mode_up | mode_rm}) {
        for (const bool update : {false, true}) {
          const Alu point{.instruction = {.opcode = conditional_group | (condition << 8) | code |
                                                    (update ? 4096U : 0U),
                                          .mode = mode},
                          .condition = condition,
                          .update = update};
          const a64::Program program(point.instruction);
          empty(program, point.instruction);
          for (Word flags = 0; flags <= status_byte; ++flags) {
            const auto initial = seed({.status = mode | flags, .index = flags});
            a64::Invocation invocation(initial);
            invocation.run(program);
            invocation.expect(alu_expected(initial, point), point.instruction.opcode);
          }
        }
      }
    }
  }
}

struct Transfer {
  unsigned condition = 0, selector = 0, source = 0, destination = 0;
  bool accumulator_b = false, dual = false, address_only = false;
};

a64::Instruction transfer_instruction(const Transfer point, const Word mode) {
  constexpr Word accumulator = 0x020000;
  constexpr Word address = 0x020800;
  constexpr Word pair = 0x030000;
  auto base = point.dual ? pair : accumulator;
  if (point.address_only) { base = address; }
  return {.opcode = base | (point.condition << 12) | (point.source << 8) | point.destination |
                    (point.selector << 4) | (point.accumulator_b ? 8U : 0U),
          .mode = mode};
}

a64::Expected
transfer_expected(const Registers &initial, const Transfer point, const a64::Instruction input) {
  a64::Expected expected{.reg = initial};
  ++expected.reg.pc;
  if (!selection_model::test_condition({.status = initial.sr, .selector = point.condition})) {
    return expected;
  }
  if (!point.address_only) {
    if ((initial.sr & (cm | mul | a16)) != 0 ||
        (initial.sr & (mode_down | mode_up)) == (mode_down | mode_up)) {
      fail(expected, FaultKind::unsupported_mode, initial.sr);
      return expected;
    }
    if (point.selector != 0 && point.selector < 4) {
      fail(expected, FaultKind::invalid_instruction, input.opcode);
      return expected;
    }
    const auto code = (point.selector << 4) | (point.accumulator_b ? 8U : 0U);
    (point.accumulator_b ? expected.reg.b : expected.reg.a) =
        static_cast<std::uint64_t>(sequence_model::source(initial, code)) & accumulator_mask;
  }
  if (point.dual || point.address_only) {
    expected.reg.r.at(point.destination) = initial.r.at(point.source);
  }
  return expected;
}

void transfer_case(const Transfer point) {
  for (const Word mode :
       {0U, saturation_model::mode, mode_rm | mode_up, cm | mul | a16, mode_up | mode_down}) {
    const auto input = transfer_instruction(point, mode);
    const a64::Program program(input);
    empty(program, input);
    for (Word flags = 0; flags <= status_byte; ++flags) {
      const auto initial = seed({.status = mode | flags, .index = flags});
      a64::Invocation invocation(initial);
      invocation.run(program);
      invocation.expect(transfer_expected(initial, point, input), input.opcode);
    }
  }
}

void conditional_transfers() {
  for (unsigned condition = 0; condition < 16; ++condition) {
    for (unsigned selector = 0; selector < 8; ++selector) {
      for (const bool destination_b : {false, true}) {
        transfer_case(
            {.condition = condition, .selector = selector, .accumulator_b = destination_b});
        for (unsigned source = 0; source < 8; ++source) {
          transfer_case({.condition = condition,
                         .selector = selector,
                         .source = source,
                         .destination = (source + condition) % 8,
                         .accumulator_b = destination_b,
                         .dual = true});
        }
      }
    }
    for (unsigned source = 0; source < 8; ++source) {
      for (unsigned destination = 0; destination < 8; ++destination) {
        transfer_case({.condition = condition,
                       .source = source,
                       .destination = destination,
                       .address_only = true});
      }
    }
  }
}

void invalid_alu_modes() {
  constexpr Word conditional_group = 0x202000;
  for (const Word mode : {cm, mul, a16, mode_up | mode_down}) {
    for (const Word code : {0U, 4U, 8U, 12U, 0x13U, 0x32U, 0x87U}) {
      alu_case({.instruction = {.opcode = parallel_none | code, .mode = mode}});
      for (unsigned condition = 0; condition < 16; ++condition) {
        alu_case(
            {.instruction = {.opcode = conditional_group | (condition << 8) | code, .mode = mode},
             .condition = condition,
             .update = false});
      }
    }
  }
}

void invalid_immediate_case(const a64::Instruction input) {
  const a64::Program program(input);
  empty(program, input);
  const auto initial = seed({.status = input.mode | status_byte, .index = 0});
  a64::Expected expected{.reg = initial};
  expected.reg.pc += input.words;
  fail(expected,
       input.mode == 0 ? FaultKind::invalid_instruction : FaultKind::unsupported_mode,
       input.mode == 0 ? input.opcode : initial.sr);
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
}

void invalid_immediates() {
  constexpr Word short_prefix = 0x014080;
  constexpr Word long_prefix = 0x0140c0;
  for (const bool long_form : {false, true}) {
    for (const Word kind : {1U, 7U, 9U, 15U}) {
      for (const Word mode : {0U, saturation_model::mode, cm, mode_up | mode_down}) {
        invalid_immediate_case({.opcode = (long_form ? long_prefix : short_prefix) | kind,
                                .extension = word_mask,
                                .mode = mode,
                                .words = long_form ? 2U : 1U,
                                .cycles = long_form ? 2U : 1U});
      }
    }
  }
}

void prefixes() {
  constexpr std::array forms{
      Alu{.instruction = {.opcode = 0x202026}, .condition = 0, .update = false},
      Alu{.instruction = {.opcode = 0x203a13}, .condition = 10},
      Alu{.instruction = {.opcode = 0x2029d2}, .condition = 9, .update = false},
      Alu{.instruction = {.opcode = 0x203533}, .condition = 5},
      Alu{.instruction = {.opcode = 0x20271d}, .condition = 7, .update = false},
      Alu{.instruction = {.opcode = 0x200021}},
      Alu{.instruction = {.opcode = 0x202f12}, .condition = 15, .update = false}};
  for (const Word mode : {0U, saturation_model::mode, saturation_model::mode | mode_down}) {
    std::array<a64::Instruction, forms.size()> instructions{};
    for (std::size_t index = 0; index < instructions.size(); ++index) {
      instructions[index] = forms[index].instruction;
      instructions[index].pc = start + static_cast<Word>(index);
      instructions[index].mode = mode;
    }
    const a64::Program program(instructions);
    for (Word budget = 0; budget <= instructions.size(); ++budget) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        const auto initial = seed({.status = mode | flags, .index = flags});
        auto expected = initial;
        for (Word index = 0; index < budget; ++index) {
          auto point = forms.at(index);
          point.instruction = instructions.at(index);
          const auto result = alu_expected(expected, point);
          require(result.failure.kind == FaultKind::none, "Authored prefix must be legal");
          expected = result.reg;
        }
        a64::Invocation invocation(initial);
        invocation.run(program, budget);
        invocation.expect({.reg = expected,
                           .retired = budget,
                           .cycles = budget,
                           .inhibited = budget == 0,
                           .repeat_completed = budget == 0},
                          instructions.front().opcode);
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::all_alu_bytes();
    dsp56300::test::conditional_alu();
    dsp56300::test::conditional_transfers();
    dsp56300::test::invalid_alu_modes();
    dsp56300::test::invalid_immediates();
    dsp56300::test::prefixes();
    std::cout << dsp56300::test::checks << " A64 complete scalar ALU/conditional checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
