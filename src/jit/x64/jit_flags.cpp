// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "value_ranges.hpp"
#include "x64.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr Word arithmetic_changed = overflow | zero | negative | unnormalized | extension;
constexpr Word observed_flags = arithmetic_changed | carry;
constexpr Word alu_mask = 0xf7, multiply_bit = 0x80;
constexpr unsigned op_or = 0x09, op_xor = 0x31, op_test = 0x85;
constexpr unsigned group_sub = 5, group_cmp = 7, group_and = 4, group_xor = 6;
constexpr unsigned shift_left = 4, shift_right = 5;
constexpr unsigned condition_zero = 0x94, condition_below = 0x92;
constexpr unsigned normal_integer_shift = 55, negative_shift = 60;

enum TransferCode : std::uint8_t {
  tfr_other = 0x01,
  tfr_x0 = 0x41,
  tfr_y0 = 0x51,
  tfr_x1 = 0x61,
  tfr_y1 = 0x71
};

bool reads_carry(const NativeOp &operation) {
  if (operation.instruction.operation != Operation::parallel) { return false; }
  const auto opcode = operation.instruction.opcode;
  const auto group = (opcode >> 4) & 7U;
  const auto kind = opcode & 7U;
  return (opcode & multiply_bit) == 0 && (group == 2 || group == 3) && (kind == 1 || kind == 5);
}
}

Word JitEmitter::written_flags(const NativeOp &operation) const {
  if (operation.instruction.operation == Operation::nop ||
      operation.instruction.operation == Operation::jump_short ||
      operation.instruction.operation == Operation::loop_immediate) {
    return 0;
  }
  if (is_logic(operation)) { return overflow | zero | negative; }
  if (is_addition(operation) || is_right_shift(operation)) { return observed_flags; }
  const auto alu = operation.instruction.opcode & byte_mask;
  const auto kind = alu & alu_mask;
  if (alu == 0 || kind == tfr_other || kind == tfr_x0 || kind == tfr_y0 || kind == tfr_x1 ||
      kind == tfr_y1) {
    return 0;
  }
  return arithmetic_changed;
}

std::vector<Word> JitEmitter::plan_flags(const std::span<const NativeOp> operations) const {
  std::vector<Word> result(operations.size(), observed_flags);
  Word live = observed_flags;
  for (std::size_t i = operations.size(); i != 0; --i) {
    const auto &operation = operations[i - 1];
    if (!lowerable(operation)) {
      live = observed_flags;
      continue;
    }
    // Moves may call observers/fault or invalidate code after their ALU. A
    // loop-ending guard may exit before its ALU. Neither boundary hides flags.
    const bool boundary = observable_moves(operation) || at_loop_end(operation) ||
                          operation.instruction.operation == Operation::loop_immediate;
    if (boundary) { live = observed_flags; }
    result[i - 1] = live;
    live &= ~written_flags(operation);
    if (reads_carry(operation)) { live |= carry; }
    if (boundary) { live = observed_flags; }
  }
  return result;
}

void JitEmitter::prepare_value_proof(const NativeOp &operation, NativeValueRanges &ranges) {
  value_proof_ = {};
  if ((operation.mode & arithmetic_saturation) == 0) {
    value_proof_.saturation = SaturationProof::none;
  }
  if (range_mode_ == RangeMode::automatic) {
    value_proof_ = ranges.advance(operation, lowerable(operation), observable_moves(operation));
  }
}

void JitEmitter::merge_flag(const Reg value, bool &initialized) {
  if (initialized) {
    code_.binary(op_or, {.destination = Reg::r8, .source = value}, Width::dword);
  } else {
    code_.mov({.destination = Reg::r8, .source = value}, Width::dword);
    initialized = true;
  }
}

void JitEmitter::zero_negative_flags(bool &initialized) {
  if ((live_flags_ & zero) != 0) {
    if (value_proof_.width == 0) {
      code_.constant(Reg::r8, zero);
    } else {
      code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax});
      code_.set(condition_zero, Reg::r8);
      code_.shift(shift_left, Reg::r8, 2, Width::dword);
    }
    initialized = true;
  }
  if ((live_flags_ & negative) != 0 && value_proof_.width != 0) {
    code_.mov({.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_right, Reg::r9, negative_shift);
    code_.immediate(group_and, Reg::r9, negative, Width::dword);
    merge_flag(Reg::r9, initialized);
  }
}

void JitEmitter::normalization_flags(const Word mode, bool &initialized) {
  const unsigned shift = normal_integer_shift + ((mode & scale_down) != 0 ? 1U : 0U) -
                         ((mode & scale_up) != 0 ? 1U : 0U);
  const auto position = shift - padding_bits;
  if ((live_flags_ & extension) != 0 && value_proof_.width > position + 1) {
    code_.mov({.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_right, Reg::r9, shift);
    code_.immediate(group_sub, Reg::r9, 1);
    const auto maximum_integer = static_cast<Word>(~std::uint64_t{0} >> shift);
    code_.immediate(group_cmp, Reg::r9, maximum_integer - 1);
    code_.set(condition_below, Reg::r9);
    code_.shift(shift_left, Reg::r9, 5, Width::dword);
    merge_flag(Reg::r9, initialized);
  }
  if ((live_flags_ & unnormalized) != 0) {
    if (value_proof_.width <= position) {
      code_.constant(Reg::r9, unnormalized);
    } else {
      code_.mov({.destination = Reg::r9, .source = Reg::ax});
      code_.shift(shift_left, Reg::r9, 1);
      code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::ax});
      code_.shift(shift_right, Reg::r9, shift);
      code_.immediate(group_xor, Reg::r9, 1, Width::dword);
      code_.immediate(group_and, Reg::r9, 1, Width::dword);
      code_.shift(shift_left, Reg::r9, 4, Width::dword);
    }
    merge_flag(Reg::r9, initialized);
  }
}

void JitEmitter::arithmetic_flags(const Word mode, const bool carry_changed) {
  bool initialized = false;
  zero_negative_flags(initialized);
  normalization_flags(mode, initialized);
  // Preserve every possible raw, rounding or limiting event in sticky L.
  if (value_proof_.overflow_value()) {
    code_.constant(Reg::r11, limit | (live_flags_ & overflow));
    code_.multiply({.destination = Reg::r10, .source = Reg::r11});
    merge_flag(Reg::r10, initialized);
  }
  if (carry_changed && (live_flags_ & carry) != 0) { merge_flag(Reg::dx, initialized); }
  const auto changed = live_flags_ & (arithmetic_changed | (carry_changed ? carry : 0));
  if (!initialized) {
    if (changed == 0) { return; }
    code_.constant(Reg::r8, 0);
  }
  merge_status(Reg::r8, changed, Reg::r9);
}
} // namespace dsp56300::detail
