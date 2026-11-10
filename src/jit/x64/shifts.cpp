// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr Word right_shift_code = 0x22, left_shift_code = 0x32, operation_mask = 0xf7;
constexpr unsigned arithmetic_right = 7, shift_left = 4;
constexpr unsigned condition_carry = 0x92, condition_overflow = 0x90, op_xor = 0x31;
}

void BlockEmitter::left_shift(const BlockInstruction &operation) {
  const bool destination_b = (operation.instruction.opcode & 8U) != 0;
  const auto result = result_register(operation);
  value(result, destination_b ? 1U : 0U, has_moves(operation));
  // Canonical raw56 is aligned at host bit eight: SHL 1 supplies both the
  // shifted-out guest sign in CF and the exact sign transition in OF.
  code_.shift(shift_left, result, 1);
  if (value_proof_.raw_overflow) { code_.set(condition_overflow, Reg::r10); }
  if ((live_flags_ & carry) != 0) { code_.set(condition_carry, Reg::dx); }
  if (!value_proof_.raw_overflow && value_proof_.overflow_value()) {
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  }
  saturate(operation.mode);
  store_accumulator(destination_b, result);
  arithmetic_flags(operation.mode, true, result);
}

void BlockEmitter::arithmetic_shift(const BlockInstruction &operation) {
  if ((operation.instruction.opcode & operation_mask) == left_shift_code) {
    left_shift(operation);
    return;
  }
  const bool destination_b = (operation.instruction.opcode & 8U) != 0;
  const auto result = result_register(operation);
  value(result, destination_b ? 1U : 0U, has_moves(operation));
  // A proved zero guest LSB keeps the padding canonical after SAR 1.
  // Otherwise shift past padding, capture guest C and restore alignment.
  const bool aligned = value_proof_.input_zero_bits != 0;
  code_.shift(arithmetic_right, result, aligned ? 1U : padding_bits + 1);
  if ((live_flags_ & carry) != 0) { code_.set(condition_carry, Reg::dx); }
  if (!aligned) { code_.shift(shift_left, result, padding_bits); }
  if (value_proof_.overflow_value()) {
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  }
  saturate(operation.mode);
  store_accumulator(destination_b, result);
  arithmetic_flags(operation.mode, true, result);
}

// Keep lowering admission separable for local matched performance controls.
bool BlockEmitter::is_arithmetic_shift(const BlockInstruction &operation) {
  const auto code = operation.instruction.opcode & operation_mask;
  return operation.instruction.operation == Operation::parallel &&
         (code == right_shift_code || code == left_shift_code);
}
} // namespace dsp56300::detail
