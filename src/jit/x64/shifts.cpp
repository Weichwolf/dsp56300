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
constexpr unsigned condition_above = 0x97, add_immediate = 0, compare_immediate = 7;
constexpr unsigned count_mask = 63, source_bit = 7, direction_bit = 8;
}

void BlockEmitter::immediate_shift(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto count = (opcode >> 1) & count_mask;
  const bool left = (opcode & (1U << direction_bit)) != 0;
  current_value(Reg::ax, (opcode & (1U << source_bit)) != 0 ? 1U : 0U);
  code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  if (count == 0) {
    if ((live_flags_ & carry) != 0) {
      code_.binary(op_xor, {.destination = Reg::dx, .source = Reg::dx}, Width::dword);
    }
  } else if (left) {
    // OF after a multi-bit host shift is undefined. The lost signed prefix
    // overflows exactly when it contains a value other than zero or minus one.
    code_.mov({.destination = Reg::r10, .source = Reg::ax});
    code_.shift(arithmetic_right, Reg::r10, host_sign_bit - count);
    code_.immediate(add_immediate, Reg::r10, 1);
    code_.immediate(compare_immediate, Reg::r10, 1);
    code_.set(condition_above, Reg::r10);
    code_.shift(shift_left, Reg::ax, count);
    if ((live_flags_ & carry) != 0) { code_.set(condition_carry, Reg::dx); }
  } else {
    code_.shift(arithmetic_right, Reg::ax, count + padding_bits);
    if ((live_flags_ & carry) != 0) { code_.set(condition_carry, Reg::dx); }
    code_.shift(shift_left, Reg::ax, padding_bits);
  }
  // Immediate forms copy either old accumulator and never apply SM limiting.
  store_accumulator((opcode & 1U) != 0, Reg::ax);
  arithmetic_flags(operation.mode, true);
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
  if (operation.instruction.operation == Operation::arithmetic_shift_immediate) {
    immediate_shift(operation);
    return;
  }
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
  if (operation.instruction.operation == Operation::arithmetic_shift_immediate) {
    return ((operation.instruction.opcode >> 1) & count_mask) < accumulator_bits;
  }
  const auto code = operation.instruction.opcode & operation_mask;
  return operation.instruction.operation == Operation::parallel &&
         (code == right_shift_code || code == left_shift_code);
}
} // namespace dsp56300::detail
