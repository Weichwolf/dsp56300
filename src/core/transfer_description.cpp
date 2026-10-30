// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"
#include "transfer_info.hpp"

#include <optional>

namespace dsp56300 {
std::optional<InstructionTransfers> inspect_transfers(const Word opcode) noexcept {
  const auto shape = inspect_instruction(opcode);
  if (!shape) { return std::nullopt; }
  detail::transfer_info::Builder builder;
  builder.result.instruction = *shape;
  switch (shape->kind) {
    case InstructionKind::parallel: detail::transfer_info::parallel(builder, opcode); break;
    case InstructionKind::nop: break;
    case InstructionKind::move_control_effective:
    case InstructionKind::move_control_short:
    case InstructionKind::move_control_register:
    case InstructionKind::move_control_immediate:
      detail::transfer_info::control(builder, opcode, shape->kind);
      break;
    case InstructionKind::move_program_effective:
    case InstructionKind::move_program_short:
      detail::transfer_info::program(builder, opcode, shape->kind);
      break;
    case InstructionKind::move_peripheral_register:
    case InstructionKind::move_peripheral_memory:
    case InstructionKind::move_peripheral_program:
      detail::transfer_info::peripheral(builder, opcode, shape->kind);
      break;
    case InstructionKind::move_displacement:
      detail::transfer_info::displacement(builder, opcode, shape->words == 2);
      break;
    case InstructionKind::jump_short:
    case InstructionKind::jump_effective:
    case InstructionKind::call_short:
    case InstructionKind::call_effective:
    case InstructionKind::return_subroutine:
    case InstructionKind::return_interrupt:
    case InstructionKind::and_control:
    case InstructionKind::or_control:
    case InstructionKind::repeat_immediate:
    case InstructionKind::loop_immediate:
    case InstructionKind::loop_register:
    case InstructionKind::loop_short:
    case InstructionKind::loop_effective:
    case InstructionKind::loop_forever:
    case InstructionKind::break_loop:
    case InstructionKind::repeat_register:
    case InstructionKind::repeat_short:
    case InstructionKind::repeat_effective:
    case InstructionKind::end_loop:
    case InstructionKind::arithmetic_shift_immediate:
    case InstructionKind::arithmetic_shift_register:
    case InstructionKind::logical_shift_immediate:
    case InstructionKind::logical_shift_register:
    case InstructionKind::immediate_alu:
    case InstructionKind::load_relative_address:
    case InstructionKind::load_updated_address:
    case InstructionKind::divide_iteration:
    case InstructionKind::normalize_iteration:
    case InstructionKind::normalize_fast:
    case InstructionKind::viterbi_shift:
    case InstructionKind::product_immediate:
    case InstructionKind::product_power:
    case InstructionKind::product_mixed:
    case InstructionKind::product_double:
    case InstructionKind::increment:
    case InstructionKind::decrement:
    case InstructionKind::compare_unsigned:
    case InstructionKind::bit_field_extract:
    case InstructionKind::bit_field_insert:
    case InstructionKind::bit_field_merge:
    case InstructionKind::count_leading_bits:
    case InstructionKind::conditional_jump:
    case InstructionKind::relative_branch:
    case InstructionKind::conditional_transfer:
    case InstructionKind::bit_register:
    case InstructionKind::bit_memory:
    case InstructionKind::bit_jump:
    case InstructionKind::bit_branch:
    case InstructionKind::illegal:
    case InstructionKind::trap:
    case InstructionKind::conditional_trap:
    case InstructionKind::cache_flush:
    case InstructionKind::cache_flush_unlocked:
    case InstructionKind::cache_free:
    case InstructionKind::cache_lock:
    case InstructionKind::cache_unlock:
    case InstructionKind::cache_lock_relative:
    case InstructionKind::cache_unlock_relative:
    case InstructionKind::reset_peripherals:
    case InstructionKind::wait:
    case InstructionKind::stop:
    case InstructionKind::debug:
    case InstructionKind::conditional_debug:
    case InstructionKind::unsupported:
    default: return std::nullopt;
  }
  return builder.valid ? std::optional(builder.result) : std::nullopt;
}
} // namespace dsp56300
