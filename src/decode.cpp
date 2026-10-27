// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail {
namespace {
struct Rule {
  Encoding bits;
  Operation operation;
  std::uint8_t words;
  std::uint8_t cycles = 1;
};

// Instruction Set appendix A: JMP, JSR, RTS/RTI, ANDI/ORI, REP and DO.
constexpr std::array rules{
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
    Rule{.bits = encoding("00000110iiiiiiii1000hhhh"),
         .operation = Operation::loop_immediate,
         .words = 2,
         .cycles = 5},
};

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
}

Instruction decode(const Word opcode) noexcept {
  for (const auto &rule : rules) {
    if (!rule.bits.matches(opcode)) { continue; }
    auto words = rule.words;
    if ((rule.operation == Operation::jump_effective ||
         rule.operation == Operation::call_effective) &&
        ((opcode >> 11) & 7) == 6) {
      words = 2;
    }
    const bool penalty = (rule.operation == Operation::jump_effective ||
                          rule.operation == Operation::call_effective) &&
                         address_penalty(opcode);
    return {.opcode = opcode,
            .operation = rule.operation,
            .words = words,
            .cycles = static_cast<std::uint8_t>(rule.cycles + (penalty ? 1 : 0))};
  }
  if ((opcode & 0x800000U) != 0 || (opcode & memory_group_mask) == 0x400000U ||
      (opcode & parallel_group_mask) == 0x200000U || (opcode & pair_group_mask) == 0x100000U ||
      (opcode & class_two_mask) == 0x080000U) {
    return {.opcode = opcode,
            .operation = Operation::parallel,
            .words = static_cast<std::uint8_t>(has_extension(opcode) ? 2 : 1),
            .cycles = static_cast<std::uint8_t>(
                1 + (parallel_address(opcode) && address_penalty(opcode) ? 1 : 0))};
  }
  return {.opcode = opcode};
}
} // namespace dsp56300::detail
