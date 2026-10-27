// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <cstddef>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr auto status_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
constexpr unsigned op_add = 0x01, op_sub = 0x29, op_or = 0x09, op_adc = 0x11, op_sbb = 0x19;
constexpr unsigned group_and = 4, shift_right = 5;
constexpr unsigned condition_overflow = 0x90, condition_below = 0x92;
}

bool JitEmitter::is_addition(const NativeOp &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto kind = opcode & 7U;
  if (operation.instruction.operation == Operation::immediate_alu) {
    return kind == 0 || kind == 4 || kind == 5;
  }
  if (operation.instruction.operation != Operation::parallel || (opcode & 0x80U) != 0) {
    return false;
  }
  const auto group = (opcode >> 4) & 7U;
  const bool carry_form = (group == 2 || group == 3) && (kind == 1 || kind == 5);
  return (group >= 1 && (kind == 0 || kind == 4)) || carry_form ||
         (kind == 5 && (group == 0 || group >= 4));
}

void JitEmitter::addition(const NativeOp &operation) {
  const auto &decoded = operation.instruction;
  const auto opcode = decoded.opcode;
  const auto kind = opcode & 7U;
  const auto group = (opcode >> 4) & 7U;
  const bool immediate = decoded.operation == Operation::immediate_alu;
  const bool with_carry = !immediate && (group == 2 || group == 3) && (kind == 1 || kind == 5);
  const bool compare = kind == 5 && !with_carry;
  const bool subtract = kind == 4 || kind == 5;
  const bool destination_b = (opcode & 8U) != 0;
  value(Reg::ax, destination_b ? 1U : 0U, has_moves(operation));
  if (immediate) {
    const auto operand = decoded.words == 2 ? decoded.extension : (opcode >> 8) & six_bit_mask;
    code_.constant(Reg::dx, aligned_word(operand));
  } else {
    input(Reg::dx, group, destination_b, has_moves(operation));
  }
  if (with_carry) {
    code_.load(Reg::r11, Reg::r12, status_offset, Width::dword);
    code_.immediate(group_and, Reg::r11, carry, Width::dword);
    code_.unary(3, Reg::r11); // NEG: 0 or all ones.
    code_.immediate(group_and, Reg::r11, byte_mask, Width::dword);
    // C=1 fills eight padding bits, then ADC/SBB adds the ninth bit. Thus the
    // carry acts at DSP bit zero while x64 OF/CF describe the full operation.
    code_.binary(op_or, {.destination = subtract ? Reg::dx : Reg::ax, .source = Reg::r11});
    code_.shift(shift_right, Reg::r11, 1, Width::dword); // CF = original guest C.
    code_.binary(subtract ? op_sbb : op_adc, {.destination = Reg::ax, .source = Reg::dx});
  } else {
    code_.binary(subtract ? op_sub : op_add, {.destination = Reg::ax, .source = Reg::dx});
  }
  code_.set(condition_overflow, Reg::r10);
  code_.set(condition_below, Reg::dx);
  if (!compare) {
    code_.store(Reg::r12,
                offsetof(Cpu, reg) +
                    (destination_b ? offsetof(Registers, b) : offsetof(Registers, a)),
                Reg::ax);
  }
  arithmetic_flags(operation.mode, true);
}

} // namespace dsp56300::detail
