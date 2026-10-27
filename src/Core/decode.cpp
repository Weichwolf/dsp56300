// SPDX-License-Identifier: GPL-3.0-only
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace dsp56300::detail {
namespace {
constexpr Word short_jump_bit = Word{1} << 18;

struct Rule {
  Encoding bits;
  Operation operation;
  std::uint8_t words;
  std::uint8_t cycles = 1;
};

// Family manual instruction encodings and documented base costs (appendix A).
constexpr std::array rules{
    Rule{.bits = encoding("0000011001MMMRRR0S0r0000"),
         .operation = Operation::loop_effective,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000011000aaaaaa0S0r0000"),
         .operation = Operation::loop_short,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000011011DDDDDD000r0000"),
         .operation = Operation::loop_register,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("00000000000000100000001r"),
         .operation = Operation::loop_forever,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("00000000000000100001CCCC"),
         .operation = Operation::break_loop,
         .words = 1,
         .cycles = 5},
    Rule{.bits = encoding("0000011001MMMRRR0S100000"),
         .operation = Operation::repeat_effective,
         .words = 1,
         .cycles = 5},
    Rule{.bits = encoding("0000011000aaaaaa0S100000"),
         .operation = Operation::repeat_short,
         .words = 1,
         .cycles = 5},
    Rule{.bits = encoding("0000011011dddddd00100000"),
         .operation = Operation::repeat_register,
         .words = 1,
         .cycles = 5},

    Rule{.bits = encoding("0000110000011010u00sSSSD"),
         .operation = Operation::bit_field_extract,
         .words = 1},
    Rule{.bits = encoding("0000110000011000u00s000D"),
         .operation = Operation::bit_field_extract,
         .words = 2,
         .cycles = 2},
    Rule{.bits = encoding("00001100000110110qqqSSSD"),
         .operation = Operation::bit_field_insert,
         .words = 1},
    Rule{.bits = encoding("00001100000110010qqq000D"),
         .operation = Operation::bit_field_insert,
         .words = 2,
         .cycles = 2},
    Rule{.bits = encoding("00001100000110111000SSSD"),
         .operation = Operation::bit_field_merge,
         .words = 1},
    Rule{.bits = encoding("0000110000011110000000SD"),
         .operation = Operation::count_leading_bits,
         .words = 1},
    Rule{.bits = encoding("00000000000000000000100d"),
         .operation = Operation::increment,
         .words = 1},
    Rule{.bits = encoding("00000000000000000000101d"),
         .operation = Operation::decrement,
         .words = 1},
    Rule{.bits = encoding("00001100000111111111gggd"),
         .operation = Operation::compare_unsigned,
         .words = 1},
    Rule{.bits = encoding("0000101k01MMMRRR0Stbbbbb"),
         .operation = Operation::bit_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("0000101k00aaaaaa0Stbbbbb"),
         .operation = Operation::bit_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("0000101k10pppppp0Stbbbbb"),
         .operation = Operation::bit_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("000000010kqqqqqq0Stbbbbb"),
         .operation = Operation::bit_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("0000101k11DDDDDD01tbbbbb"),
         .operation = Operation::bit_register,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("0000101k01MMMRRR1Stbbbbb"),
         .operation = Operation::bit_jump,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("0000101k00aaaaaa1Stbbbbb"),
         .operation = Operation::bit_jump,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("0000101k10pppppp1Stbbbbb"),
         .operation = Operation::bit_jump,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("0000101k11DDDDDD00tbbbbb"),
         .operation = Operation::bit_jump,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("000000011kqqqqqq1Stbbbbb"),
         .operation = Operation::bit_jump,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("0000110k10MMMRRR0Stbbbbb"),
         .operation = Operation::bit_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000110k10aaaaaa1Stbbbbb"),
         .operation = Operation::bit_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000110k11pppppp0Stbbbbb"),
         .operation = Operation::bit_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000110k11DDDDDD10tbbbbb"),
         .operation = Operation::bit_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("0000010010qqqqqqkStbbbbb"),
         .operation = Operation::bit_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("00000101W1MMMRRR0S1ddddd"),
         .operation = Operation::move_control_effective,
         .words = 1},
    Rule{.bits = encoding("00000101W0aaaaaa0S1ddddd"),
         .operation = Operation::move_control_short,
         .words = 1},
    Rule{.bits = encoding("00000100W1eeeeee101ddddd"),
         .operation = Operation::move_control_register,
         .words = 1},
    Rule{.bits = encoding("00000101iiiiiiii101ddddd"),
         .operation = Operation::move_control_immediate,
         .words = 1},
    Rule{.bits = encoding("00000111W1MMMRRR10dddddd"),
         .operation = Operation::move_program_effective,
         .words = 1,
         .cycles = 6},
    Rule{.bits = encoding("00000111W0aaaaaa00dddddd"),
         .operation = Operation::move_program_short,
         .words = 1,
         .cycles = 6},
    Rule{.bits = encoding("0000100sW1dddddd00pppppp"),
         .operation = Operation::move_peripheral_register,
         .words = 1},
    Rule{.bits = encoding("00000100W1dddddd1q0qqqqq"),
         .operation = Operation::move_peripheral_register,
         .words = 1},
    Rule{.bits = encoding("00000100W1dddddd0q1qqqqq"),
         .operation = Operation::move_peripheral_register,
         .words = 1},
    Rule{.bits = encoding("0000100sW1MMMRRR1Spppppp"),
         .operation = Operation::move_peripheral_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("00000111W1MMMRRR0Sqqqqqq"),
         .operation = Operation::move_peripheral_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("00000111W0MMMRRR1Sqqqqqq"),
         .operation = Operation::move_peripheral_memory,
         .words = 1,
         .cycles = 2},
    Rule{.bits = encoding("0000100sW1MMMRRR01pppppp"),
         .operation = Operation::move_peripheral_program,
         .words = 1,
         .cycles = 6},
    Rule{.bits = encoding("000000001WMMMRRR0Sqqqqqq"),
         .operation = Operation::move_peripheral_program,
         .words = 1,
         .cycles = 6},
    Rule{.bits = encoding("00000001010000001100dkkk"),
         .operation = Operation::immediate_alu,
         .words = 2,
         .cycles = 2},
    Rule{.bits = encoding("0000000101iiiiii1000dkkk"),
         .operation = Operation::immediate_alu,
         .words = 1,
         .cycles = 1},
    Rule{.bits = encoding("00001110CCCCaaaaaaaaaaaa"),
         .operation = Operation::conditional_jump,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("00001111CCCCaaaaaaaaaaaa"),
         .operation = Operation::conditional_jump,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("0000101011MMMRRR1010CCCC"),
         .operation = Operation::conditional_jump,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("0000101111MMMRRR1010CCCC"),
         .operation = Operation::conditional_jump,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("00000010CCCC00000JJJd000"),
         .operation = Operation::conditional_transfer,
         .words = 1,
         .cycles = 1},
    Rule{.bits = encoding("00000011CCCC0ttt0JJJdTTT"),
         .operation = Operation::conditional_transfer,
         .words = 1,
         .cycles = 1},
    Rule{.bits = encoding("00000010CCCC1ttt00000TTT"),
         .operation = Operation::conditional_transfer,
         .words = 1,
         .cycles = 1},
    Rule{.bits = encoding("00000101000011aaaa0aaaaa"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("00000101000010aaaa0aaaaa"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("00000101CCCC01aaaa0aaaaa"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("00000101CCCC00aaaa0aaaaa"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("000011010001000011000000"),
         .operation = Operation::relative_branch,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("000011010001000010000000"),
         .operation = Operation::relative_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("00001101000100000100CCCC"),
         .operation = Operation::relative_branch,
         .words = 2,
         .cycles = 5},
    Rule{.bits = encoding("00001101000100000000CCCC"),
         .operation = Operation::relative_branch,
         .words = 2,
         .cycles = 4},
    Rule{.bits = encoding("0000110100011RRR11000000"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("0000110100011RRR10000000"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("0000110100011RRR0100CCCC"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},
    Rule{.bits = encoding("0000110100011RRR0000CCCC"),
         .operation = Operation::relative_branch,
         .words = 1,
         .cycles = 4},

    Rule{.bits = encoding("0000110000011100SiiiiiiD"),
         .operation = Operation::arithmetic_shift_immediate,
         .words = 1},
    Rule{.bits = encoding("0000110000011101SiiiiiiD"),
         .operation = Operation::arithmetic_shift_immediate,
         .words = 1},
    Rule{.bits = encoding("0000110000011110010SsssD"),
         .operation = Operation::arithmetic_shift_register,
         .words = 1},
    Rule{.bits = encoding("0000110000011110011SsssD"),
         .operation = Operation::arithmetic_shift_register,
         .words = 1},
    Rule{.bits = encoding("000011000001111010iiiiiD"),
         .operation = Operation::logical_shift_immediate,
         .words = 1},
    Rule{.bits = encoding("000011000001111011iiiiiD"),
         .operation = Operation::logical_shift_immediate,
         .words = 1},
    Rule{.bits = encoding("00001100000111100001sssD"),
         .operation = Operation::logical_shift_register,
         .words = 1},
    Rule{.bits = encoding("00001100000111100011sssD"),
         .operation = Operation::logical_shift_register,
         .words = 1},
    Rule{.bits = encoding("000000000000000000000000"), .operation = Operation::nop, .words = 1},
    Rule{.bits = encoding("000000000000000000001100"),
         .operation = Operation::return_subroutine,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("000000000000000000000100"),
         .operation = Operation::return_interrupt,
         .words = 1,
         .cycles = 3},
    Rule{
        .bits = encoding("000000000000000010001100"), .operation = Operation::end_loop, .words = 1},
    Rule{.bits = encoding("000011000000aaaaaaaaaaaa"),
         .operation = Operation::jump_short,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("000011010000aaaaaaaaaaaa"),
         .operation = Operation::call_short,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("0000101011MMMRRR10000000"),
         .operation = Operation::jump_effective,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("0000101111MMMRRR10000000"),
         .operation = Operation::call_effective,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("00000000iiiiiiii101110EE"),
         .operation = Operation::and_control,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("00000000iiiiiiii111110EE"),
         .operation = Operation::or_control,
         .words = 1,
         .cycles = 3},
    Rule{.bits = encoding("00000110iiiiiiii1010hhhh"),
         .operation = Operation::repeat_immediate,
         .words = 1,
         .cycles = 5},
    Rule{.bits = encoding("00000110iiiiiiii100rhhhh"),
         .operation = Operation::loop_immediate,
         .words = 2,
         .cycles = 5},
};

constexpr std::array parallel_groups{
    Encoding{.mask = 0x800000, .value = 0x800000},
    Encoding{.mask = memory_group_mask, .value = memory_group},
    Encoding{.mask = parallel_group_mask, .value = immediate_group},
    Encoding{.mask = pair_group_mask, .value = pair_group},
    Encoding{.mask = class_two_mask, .value = class_two}};

consteval bool parallel_rules_disjoint() {
  for (const auto &rule : rules) {
    for (const auto group : parallel_groups) {
      if (((rule.bits.value ^ group.value) & rule.bits.mask & group.mask) == 0) { return false; }
    }
  }
  return true;
}

static_assert(parallel_rules_disjoint(), "Prioritize overlapping specific instruction encodings");

constexpr bool parallel_encoding(const Word opcode) {
  return std::ranges::any_of(parallel_groups,
                             [opcode](const Encoding group) { return group.matches(opcode); });
}

bool has_extension(const Word opcode) noexcept {
  const unsigned mode = (opcode >> 11) & 7;
  if ((opcode & memory_group_mask) == 0x400000U) { return (opcode & 0x4000U) != 0 && mode == 6; }
  if ((opcode & pair_group_mask) == 0x100000U) { return mode == 6; }
  return false;
}

bool address_penalty(const Word opcode) noexcept {
  const unsigned mode = (opcode >> 11) & 7;
  return mode == 5 || mode == 6 || mode == 7;
}

bool parallel_address(const Word opcode) noexcept {
  const bool memory = (opcode & memory_group_mask) == memory_group && (opcode & (1U << 14)) != 0;
  return memory || (opcode & pair_group_mask) == pair_group ||
         (opcode & class_two_mask) == class_two;
}

bool transfer_address(const Operation operation) noexcept {
  return operation == Operation::move_control_effective ||
         operation == Operation::move_program_effective ||
         operation == Operation::move_peripheral_memory ||
         operation == Operation::move_peripheral_program;
}

bool rule_penalty(const Rule &rule, const Word opcode) noexcept {
  if (rule.operation == Operation::bit_memory || rule.operation == Operation::bit_jump ||
      rule.operation == Operation::bit_branch) {
    const auto operand = bit_operand({.opcode = opcode, .operation = rule.operation});
    return operand.location == BitLocation::effective && address_penalty(opcode);
  }
  if (rule.operation == Operation::loop_effective ||
      rule.operation == Operation::repeat_effective) {
    return address_penalty(opcode);
  }
  if (transfer_address(rule.operation)) {
    const auto mode = (opcode >> 11) & 7;
    const bool immediate_peripheral = rule.operation == Operation::move_peripheral_memory &&
                                      mode == 6 && ((opcode >> 8) & 7) == 4;
    return (mode == 5 || mode == 6 || mode == 7) && !immediate_peripheral;
  }
  return (rule.operation == Operation::jump_effective ||
          rule.operation == Operation::call_effective) &&
         address_penalty(opcode);
}
}

Instruction decode(const Word opcode) noexcept {
  if (parallel_encoding(opcode)) {
    return {.opcode = opcode,
            .operation = Operation::parallel,
            .words = static_cast<std::uint8_t>(has_extension(opcode) ? 2 : 1),
            .cycles = static_cast<std::uint8_t>(
                1 + (parallel_address(opcode) && address_penalty(opcode) ? 1 : 0))};
  }
  for (const auto &rule : rules) {
    if (!rule.bits.matches(opcode)) { continue; }
    auto words = rule.words;
    if ((rule.operation == Operation::jump_effective ||
         rule.operation == Operation::call_effective || transfer_address(rule.operation) ||
         (rule.operation == Operation::bit_memory &&
          bit_operand({.opcode = opcode, .operation = rule.operation}).location ==
              BitLocation::effective) ||
         (rule.operation == Operation::conditional_jump && (opcode & short_jump_bit) == 0)) &&
        ((opcode >> 11) & 7) == 6) {
      words = 2;
    }
    return {.opcode = opcode,
            .operation = rule.operation,
            .words = words,
            .cycles =
                static_cast<std::uint8_t>(rule.cycles + (rule_penalty(rule, opcode) ? 1 : 0))};
  }

  return {.opcode = opcode};
}
} // namespace dsp56300::detail
