// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"

#include <cstdint>

namespace dsp56300::detail {
using Operation = InstructionKind;

[[nodiscard]] constexpr bool repeatable(const Operation operation) noexcept {
  switch (operation) {
    case Operation::parallel:
    case Operation::nop:
    case Operation::illegal:
    case Operation::debug:
    case Operation::conditional_debug:
    case Operation::cache_flush:
    case Operation::cache_flush_unlocked:
    case Operation::cache_free:
    case Operation::reset_peripherals:
    case Operation::cache_lock:
    case Operation::cache_unlock:
    case Operation::and_control:
    case Operation::or_control:
    case Operation::arithmetic_shift_immediate:
    case Operation::arithmetic_shift_register:
    case Operation::logical_shift_immediate:
    case Operation::logical_shift_register:
    case Operation::immediate_alu:
    case Operation::load_relative_address:
    case Operation::load_updated_address:
    case Operation::divide_iteration:
    case Operation::normalize_iteration:
    case Operation::normalize_fast:
    case Operation::viterbi_shift:
    case Operation::product_immediate:
    case Operation::product_power:
    case Operation::product_mixed:
    case Operation::product_double:
    case Operation::increment:
    case Operation::decrement:
    case Operation::compare_unsigned:
    case Operation::bit_field_extract:
    case Operation::bit_field_insert:
    case Operation::bit_field_merge:
    case Operation::count_leading_bits:
    case Operation::conditional_transfer:
    case Operation::move_control_effective:
    case Operation::move_displacement:
    case Operation::move_control_short:
    case Operation::move_control_register:
    case Operation::move_control_immediate:
    case Operation::move_program_effective:
    case Operation::move_program_short:
    case Operation::move_peripheral_register:
    case Operation::move_peripheral_memory:
    case Operation::move_peripheral_program:
    case Operation::bit_register:
    case Operation::bit_memory: return true;
    case Operation::wait:
    case Operation::stop:
    case Operation::cache_lock_relative:
    case Operation::cache_unlock_relative:
    case Operation::unsupported:
    case Operation::trap:
    case Operation::conditional_trap:
    case Operation::jump_short:
    case Operation::jump_effective:
    case Operation::call_short:
    case Operation::call_effective:
    case Operation::return_subroutine:
    case Operation::return_interrupt:
    case Operation::repeat_immediate:
    case Operation::loop_immediate:
    case Operation::loop_register:
    case Operation::loop_short:
    case Operation::loop_effective:
    case Operation::loop_forever:
    case Operation::break_loop:
    case Operation::repeat_register:
    case Operation::repeat_short:
    case Operation::repeat_effective:
    case Operation::conditional_jump:
    case Operation::relative_branch:
    case Operation::end_loop:
    case Operation::bit_jump:
    case Operation::bit_branch:
    default: return false;
  }
}

[[nodiscard]] constexpr bool is_repeat(const Operation operation) noexcept {
  return operation == Operation::repeat_immediate || operation == Operation::repeat_register ||
         operation == Operation::repeat_short || operation == Operation::repeat_effective;
}

struct Instruction {
  Word opcode = 0, extension = 0;
  Operation operation = Operation::unsupported;
  std::uint8_t words = 1;
  std::uint8_t cycles = 1;
  bool operator==(const Instruction &) const = default;
};

// Conservative upper bound from the same base-cost rules used by decoding.
[[nodiscard]] unsigned instruction_cycle_bound() noexcept;
[[nodiscard]] Instruction decode(Word opcode) noexcept;
} // namespace dsp56300::detail
