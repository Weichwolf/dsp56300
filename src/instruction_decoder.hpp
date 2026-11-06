// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"

#include <cstdint>
#include <optional>

namespace dsp56300 {
// Encoded family/width only; recognition does not qualify all operands or modes.
enum class InstructionKind : std::uint8_t {
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
  loop_register,
  loop_short,
  loop_effective,
  loop_forever,
  break_loop,
  repeat_register,
  repeat_short,
  repeat_effective,
  end_loop,
  arithmetic_shift_immediate,
  arithmetic_shift_register,
  logical_shift_immediate,
  logical_shift_register,
  immediate_alu,
  load_relative_address,
  load_updated_address,
  divide_iteration,
  normalize_iteration,
  normalize_fast,
  viterbi_shift,
  product_immediate,
  product_power,
  product_mixed,
  product_double,
  increment,
  decrement,
  compare_unsigned,
  bit_field_extract,
  bit_field_insert,
  bit_field_merge,
  count_leading_bits,
  conditional_jump,
  relative_branch,
  conditional_transfer,
  move_control_effective,
  move_displacement,
  move_control_short,
  move_control_register,
  move_control_immediate,
  move_program_effective,
  move_program_short,
  move_peripheral_register,
  move_peripheral_memory,
  move_peripheral_program,
  bit_register,
  bit_memory,
  bit_jump,
  bit_branch,
  illegal,
  trap,
  conditional_trap,
  cache_flush,
  cache_flush_unlocked,
  cache_free,
  cache_lock,
  cache_unlock,
  cache_lock_relative,
  cache_unlock_relative,
  reset_peripherals,
  wait,
  stop,
  debug,
  conditional_debug
};

struct InstructionShape {
  InstructionKind kind = InstructionKind::unsupported;
  std::uint8_t words = 0;
  bool repeatable = false;
  bool operator==(const InstructionShape &) const = default;
};

// Pure decode: no Core, RAM, callback, clock edge or instruction fetch occurs.
// A result is a recognized encoding shape, not full ISA or pipeline acceptance.
[[nodiscard]] std::optional<InstructionShape> inspect_instruction(Word opcode) noexcept;
} // namespace dsp56300
