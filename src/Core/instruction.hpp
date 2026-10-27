// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <cstdint>
#include <string_view>

namespace dsp56300::detail {
enum class Operation : std::uint8_t {
  unsupported,
  parallel,
  nop,
  jump_short,
  jump_effective,
  call_short,
  call_effective,
  return_subroutine,
  return_interrupt,
  and_control,
  or_control,
  repeat_immediate,
  loop_immediate,
  end_loop,
  arithmetic_shift_immediate,
  arithmetic_shift_register,
  logical_shift_immediate,
  logical_shift_register,
  immediate_alu,
  conditional_jump,
  relative_branch,
  conditional_transfer,
  move_control_effective,
  move_control_short,
  move_control_register,
  move_control_immediate,
  move_program_effective,
  move_program_short,
  move_peripheral_register,
  move_peripheral_memory,
  move_peripheral_program
};

[[nodiscard]] constexpr bool repeatable(const Operation operation) noexcept {
  switch (operation) {
    case Operation::parallel:
    case Operation::nop:
    case Operation::arithmetic_shift_immediate:
    case Operation::arithmetic_shift_register:
    case Operation::logical_shift_immediate:
    case Operation::logical_shift_register:
    case Operation::immediate_alu:
    case Operation::conditional_transfer:
    case Operation::move_control_effective:
    case Operation::move_control_short:
    case Operation::move_control_register:
    case Operation::move_control_immediate:
    case Operation::move_program_effective:
    case Operation::move_program_short:
    case Operation::move_peripheral_register:
    case Operation::move_peripheral_memory:
    case Operation::move_peripheral_program: return true;
    case Operation::unsupported:
    case Operation::jump_short:
    case Operation::jump_effective:
    case Operation::call_short:
    case Operation::call_effective:
    case Operation::return_subroutine:
    case Operation::return_interrupt:
    case Operation::and_control:
    case Operation::or_control:
    case Operation::repeat_immediate:
    case Operation::loop_immediate:
    case Operation::conditional_jump:
    case Operation::relative_branch:
    case Operation::end_loop:
    default: return false;
  }
}

struct Instruction {
  Word opcode = 0, extension = 0;
  Operation operation = Operation::unsupported;
  std::uint8_t words = 1;
  std::uint8_t cycles = 1;
  bool operator==(const Instruction &) const = default;
};

struct Encoding {
  Word mask = 0, value = 0;

  [[nodiscard]] constexpr bool matches(const Word opcode) const noexcept {
    return (opcode & mask) == value;
  }
};

consteval Encoding encoding(const std::string_view pattern) {
  constexpr unsigned instruction_bits = 24;
  if (pattern.size() != instruction_bits) { throw "Invalid DSP instruction encoding width"; }
  Encoding result;
  for (const char bit : pattern) {
    result.mask <<= 1;
    result.value <<= 1;
    if (bit == '0' || bit == '1') { result.mask |= 1; }
    if (bit == '1') { result.value |= 1; }
  }
  return result;
}

[[nodiscard]] Instruction decode(Word opcode) noexcept;
} // namespace dsp56300::detail
