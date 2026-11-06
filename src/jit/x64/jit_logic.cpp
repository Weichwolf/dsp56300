// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr unsigned op_or = 0x09, op_and = 0x21, op_xor = 0x31, op_test = 0x85;
constexpr unsigned group_and = 4, group_xor = 6, shift_left = 4, shift_right = 5;
constexpr unsigned condition_zero = 0x94, middle_shift = word_bits + padding_bits;
constexpr unsigned negative_to_flag = word_bits - 4;
constexpr Word complement = 0x17, alu_mask = 0xf7, product_bit = 0x80;
constexpr Word logical_changed = negative | zero | overflow;
}

bool JitEmitter::is_logic(const NativeOp &operation) {
  if (operation.instruction.operation != Operation::parallel) { return false; }
  const auto alu = operation.instruction.opcode & byte_mask;
  if ((alu & product_bit) != 0) { return false; }
  const auto group = (alu >> 4) & 7U;
  const auto kind = alu & 7U;
  return (alu & alu_mask) == complement || (group >= 4 && (kind == 2 || kind == 3 || kind == 6));
}

void JitEmitter::logical_flags(const Reg word) {
  const auto changed = live_flags_ & logical_changed;
  if (changed == 0) { return; }
  code_.constant(Reg::r8, 0);
  if ((changed & zero) != 0) {
    code_.binary(op_test, {.destination = word, .source = word}, Width::dword);
    code_.set(condition_zero, Reg::r8);
    code_.shift(shift_left, Reg::r8, 2, Width::dword);
  }
  if ((changed & negative) != 0) {
    code_.mov({.destination = Reg::r9, .source = word}, Width::dword);
    code_.shift(shift_right, Reg::r9, negative_to_flag, Width::dword);
    code_.immediate(group_and, Reg::r9, negative, Width::dword);
    code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  }
  // Only N/Z/V change. Keep the post-MOVE accumulator in RAX intact.
  merge_status(Reg::r8, changed, Reg::r10);
}

void JitEmitter::logic(const NativeOp &operation) {
  // RCX may retain a reused product operand across neighboring instructions.
  const auto opcode = operation.instruction.opcode;
  const bool destination_b = (opcode & 8U) != 0;
  const auto index = destination_b ? 1U : 0U;
  current_value(Reg::ax, index);
  if ((dataflow_.old_inputs & (Word{1} << index)) != 0) {
    value(Reg::r11, index, true);
  } else {
    code_.mov({.destination = Reg::r11, .source = Reg::ax});
  }
  code_.shift(shift_right, Reg::r11, middle_shift);
  code_.immediate(group_and, Reg::r11, word_mask, Width::dword);
  if ((opcode & alu_mask) == complement) {
    code_.immediate(group_xor, Reg::r11, word_mask, Width::dword);
  } else {
    const auto source = (opcode >> 4) & 7U;
    value(Reg::dx, (source & 1U) + 2, has_moves(operation));
    if (source >= 6) { code_.shift(shift_right, Reg::dx, word_bits); }
    code_.immediate(group_and, Reg::dx, word_mask, Width::dword);
    const auto kind = opcode & 7U;
    unsigned function = op_and;
    if (kind == 2) {
      function = op_or;
    } else if (kind == 3) {
      function = op_xor;
    }
    code_.binary(function, {.destination = Reg::r11, .source = Reg::dx}, Width::dword);
  }
  logical_flags(Reg::r11);
  constexpr auto middle = std::uint64_t{word_mask} << middle_shift;
  code_.constant(Reg::dx, ~middle);
  code_.binary(op_and, {.destination = Reg::ax, .source = Reg::dx});
  code_.shift(shift_left, Reg::r11, middle_shift);
  code_.binary(op_or, {.destination = Reg::ax, .source = Reg::r11});
  store_accumulator(destination_b, Reg::ax);
}
} // namespace dsp56300::detail
