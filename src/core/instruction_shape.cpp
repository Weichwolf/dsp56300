// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "instruction.hpp"

#include <optional>

namespace dsp56300 {
namespace {
InstructionKind public_kind(const detail::Operation operation) noexcept {
  switch (operation) {
    case detail::Operation::unsupported: return InstructionKind::unsupported;
    case detail::Operation::parallel: return InstructionKind::parallel;
    case detail::Operation::nop: return InstructionKind::nop;
    case detail::Operation::jump_short: return InstructionKind::jump_short;
    case detail::Operation::jump_effective: return InstructionKind::jump_effective;
    case detail::Operation::call_short: return InstructionKind::call_short;
    case detail::Operation::call_effective: return InstructionKind::call_effective;
    case detail::Operation::return_subroutine: return InstructionKind::return_subroutine;
    case detail::Operation::return_interrupt: return InstructionKind::return_interrupt;
    case detail::Operation::and_control: return InstructionKind::and_control;
    case detail::Operation::or_control: return InstructionKind::or_control;
    case detail::Operation::repeat_immediate: return InstructionKind::repeat_immediate;
    case detail::Operation::loop_immediate: return InstructionKind::loop_immediate;
    case detail::Operation::loop_register: return InstructionKind::loop_register;
    case detail::Operation::loop_short: return InstructionKind::loop_short;
    case detail::Operation::loop_effective: return InstructionKind::loop_effective;
    case detail::Operation::loop_forever: return InstructionKind::loop_forever;
    case detail::Operation::break_loop: return InstructionKind::break_loop;
    case detail::Operation::repeat_register: return InstructionKind::repeat_register;
    case detail::Operation::repeat_short: return InstructionKind::repeat_short;
    case detail::Operation::repeat_effective: return InstructionKind::repeat_effective;
    case detail::Operation::end_loop: return InstructionKind::end_loop;
    case detail::Operation::arithmetic_shift_immediate:
      return InstructionKind::arithmetic_shift_immediate;
    case detail::Operation::arithmetic_shift_register:
      return InstructionKind::arithmetic_shift_register;
    case detail::Operation::logical_shift_immediate:
      return InstructionKind::logical_shift_immediate;
    case detail::Operation::logical_shift_register: return InstructionKind::logical_shift_register;
    case detail::Operation::immediate_alu: return InstructionKind::immediate_alu;
    case detail::Operation::load_relative_address: return InstructionKind::load_relative_address;
    case detail::Operation::load_updated_address: return InstructionKind::load_updated_address;
    case detail::Operation::divide_iteration: return InstructionKind::divide_iteration;
    case detail::Operation::normalize_iteration: return InstructionKind::normalize_iteration;
    case detail::Operation::normalize_fast: return InstructionKind::normalize_fast;
    case detail::Operation::viterbi_shift: return InstructionKind::viterbi_shift;
    case detail::Operation::product_immediate: return InstructionKind::product_immediate;
    case detail::Operation::product_power: return InstructionKind::product_power;
    case detail::Operation::product_mixed: return InstructionKind::product_mixed;
    case detail::Operation::product_double: return InstructionKind::product_double;
    case detail::Operation::increment: return InstructionKind::increment;
    case detail::Operation::decrement: return InstructionKind::decrement;
    case detail::Operation::compare_unsigned: return InstructionKind::compare_unsigned;
    case detail::Operation::bit_field_extract: return InstructionKind::bit_field_extract;
    case detail::Operation::bit_field_insert: return InstructionKind::bit_field_insert;
    case detail::Operation::bit_field_merge: return InstructionKind::bit_field_merge;
    case detail::Operation::count_leading_bits: return InstructionKind::count_leading_bits;
    case detail::Operation::conditional_jump: return InstructionKind::conditional_jump;
    case detail::Operation::relative_branch: return InstructionKind::relative_branch;
    case detail::Operation::conditional_transfer: return InstructionKind::conditional_transfer;
    case detail::Operation::move_control_effective: return InstructionKind::move_control_effective;
    case detail::Operation::move_displacement: return InstructionKind::move_displacement;
    case detail::Operation::move_control_short: return InstructionKind::move_control_short;
    case detail::Operation::move_control_register: return InstructionKind::move_control_register;
    case detail::Operation::move_control_immediate: return InstructionKind::move_control_immediate;
    case detail::Operation::move_program_effective: return InstructionKind::move_program_effective;
    case detail::Operation::move_program_short: return InstructionKind::move_program_short;
    case detail::Operation::move_peripheral_register:
      return InstructionKind::move_peripheral_register;
    case detail::Operation::move_peripheral_memory: return InstructionKind::move_peripheral_memory;
    case detail::Operation::move_peripheral_program:
      return InstructionKind::move_peripheral_program;
    case detail::Operation::bit_register: return InstructionKind::bit_register;
    case detail::Operation::bit_memory: return InstructionKind::bit_memory;
    case detail::Operation::bit_jump: return InstructionKind::bit_jump;
    case detail::Operation::bit_branch: return InstructionKind::bit_branch;
    case detail::Operation::illegal: return InstructionKind::illegal;
    case detail::Operation::trap: return InstructionKind::trap;
    case detail::Operation::conditional_trap: return InstructionKind::conditional_trap;
    case detail::Operation::cache_flush: return InstructionKind::cache_flush;
    case detail::Operation::cache_flush_unlocked: return InstructionKind::cache_flush_unlocked;
    case detail::Operation::cache_free: return InstructionKind::cache_free;
    case detail::Operation::cache_lock: return InstructionKind::cache_lock;
    case detail::Operation::cache_unlock: return InstructionKind::cache_unlock;
    case detail::Operation::cache_lock_relative: return InstructionKind::cache_lock_relative;
    case detail::Operation::cache_unlock_relative: return InstructionKind::cache_unlock_relative;
    case detail::Operation::reset_peripherals: return InstructionKind::reset_peripherals;
    case detail::Operation::wait: return InstructionKind::wait;
    case detail::Operation::stop: return InstructionKind::stop;
    case detail::Operation::debug: return InstructionKind::debug;
    case detail::Operation::conditional_debug: return InstructionKind::conditional_debug;
    default: return InstructionKind::unsupported;
  }
}
}

std::optional<InstructionShape> inspect_instruction(const Word opcode) noexcept {
  if (opcode > word_mask) { return std::nullopt; }
  const auto decoded = detail::decode(opcode);
  if (decoded.operation == detail::Operation::unsupported) { return std::nullopt; }
  return InstructionShape{.kind = public_kind(decoded.operation),
                          .words = decoded.words,
                          .repeatable =
                              decoded.words == 1 && detail::repeatable(decoded.operation)};
}
} // namespace dsp56300
