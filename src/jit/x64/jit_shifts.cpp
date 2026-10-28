// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr Word right_shift_code = 0x22, operation_mask = 0xf7;
constexpr unsigned arithmetic_right = 7, shift_left = 4;
constexpr unsigned condition_carry = 0x92, op_xor = 0x31;
}

void JitEmitter::right_shift(const NativeOp &operation) {
  const bool destination_b = (operation.instruction.opcode & 8U) != 0;
  value(Reg::ax, destination_b ? 1U : 0U, has_moves(operation));
  // Shift past padding as well: CF then contains the original DSP bit zero.
  // Restore alignment afterwards, discarding every newly shifted padding bit.
  code_.shift(arithmetic_right, Reg::ax, padding_bits + 1);
  if ((live_flags_ & carry) != 0) { code_.set(condition_carry, Reg::dx); }
  code_.shift(shift_left, Reg::ax, padding_bits);
  code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  store_accumulator(destination_b, Reg::ax);
  arithmetic_flags(operation.mode, true);
}

// Keep lowering admission separable for local matched performance controls.
bool JitEmitter::is_right_shift(const NativeOp &operation) const {
  return optimize_ && operation.instruction.operation == Operation::parallel &&
         (operation.instruction.opcode & operation_mask) == right_shift_code;
}
} // namespace dsp56300::detail
