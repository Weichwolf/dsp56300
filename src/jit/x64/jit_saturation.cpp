// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "value_ranges.hpp"
#include "x64.hpp"

#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr Word alu_mask = 0xf7;
constexpr unsigned op_xor = 0x31, op_or = 0x09, op_test = 0x85;
constexpr unsigned group_and = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_negative = 0x48, condition_not_zero = 0x45, set_not_zero = 0x95;
constexpr unsigned msp_sign = 47 + padding_bits;
constexpr std::uint64_t positive = 0x007fffffffffff00, negative_limit = 0xff80000000000000;
constexpr std::uint64_t rounded_positive = 0x007fffff00000000;

enum Alu : std::uint8_t {
  alu_tfr_other = 0x01,
  alu_tfr_x0 = 0x41,
  alu_tfr_y0 = 0x51,
  alu_tfr_x1 = 0x61,
  alu_tfr_y1 = 0x71,
  alu_clear = 0x13,
  alu_round = 0x11,
  alu_absolute = 0x26,
  alu_negate = 0x36
};
}

bool JitEmitter::saturation_supported(const NativeOp &operation) {
  const auto &decoded = operation.instruction;
  const auto opcode = decoded.opcode;
  const auto kind = opcode & 7U;
  if (decoded.operation == Operation::immediate_alu) { return kind == 0 || kind == 4; }
  if (decoded.operation != Operation::parallel) { return false; }
  const auto alu = opcode & byte_mask;
  const auto form = alu & alu_mask;
  const auto group = (alu >> 4) & 7U;
  const bool sum = group >= 1 && (kind == 0 || kind == 4);
  const bool carry_sum = (group == 2 || group == 3) && (kind == 1 || kind == 5);
  return alu == 0 || (alu & 0x80U) != 0 || sum || carry_sum || form == alu_tfr_other ||
         form == alu_tfr_x0 || form == alu_tfr_y0 || form == alu_tfr_x1 || form == alu_tfr_y1 ||
         form == alu_clear || form == alu_round || form == alu_absolute || form == alu_negate;
}

void JitEmitter::saturate(const Word mode, const bool rounded) {
  if ((mode & arithmetic_saturation) == 0) { return; }
  if (value_proof_.saturation == SaturationProof::none) { return; }
  if (value_proof_.saturation == SaturationProof::canonical) {
    saturate_canonical(rounded);
    return;
  }
  // Only EXT[7], EXT[0], MSP[23] participate. Preserve raw C in EDX and OF in R10D.
  code_.mov({.destination = Reg::r8, .source = Reg::ax});
  code_.shift(shift_arithmetic, Reg::r8, host_sign_bit);
  code_.mov({.destination = Reg::r9, .source = Reg::ax});
  code_.shift(shift_right, Reg::r9, msp_sign);
  code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::r8});
  code_.immediate(group_and, Reg::r9, 3, Width::dword);
  code_.set(set_not_zero, Reg::r9);
  code_.binary(op_or, {.destination = Reg::r10, .source = Reg::r9}, Width::dword);
  code_.constant(Reg::r11, rounded ? rounded_positive : positive);
  code_.constant(Reg::r8, negative_limit);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax});
  code_.move_if(condition_negative, {.destination = Reg::r11, .source = Reg::r8});
  code_.binary(op_test, {.destination = Reg::r9, .source = Reg::r9}, Width::dword);
  code_.move_if(condition_not_zero, {.destination = Reg::ax, .source = Reg::r11});
}

void JitEmitter::saturate_canonical(const bool rounded) {
  constexpr unsigned op_cmp = 0x39;
  constexpr unsigned condition_greater = 0x4f;
  constexpr unsigned condition_less = 0x4c;
  code_.mov({.destination = Reg::r8, .source = Reg::ax});
  code_.constant(Reg::r11, positive);
  code_.binary(op_cmp, {.destination = Reg::ax, .source = Reg::r11});
  if (rounded) { code_.constant(Reg::r9, rounded_positive); }
  code_.move_if(condition_greater,
                {.destination = Reg::ax, .source = rounded ? Reg::r9 : Reg::r11});
  code_.constant(Reg::r11, negative_limit);
  code_.binary(op_cmp, {.destination = Reg::ax, .source = Reg::r11});
  code_.move_if(condition_less, {.destination = Reg::ax, .source = Reg::r11});
  code_.binary(op_cmp, {.destination = Reg::ax, .source = Reg::r8});
  code_.set(set_not_zero, Reg::r9);
  code_.binary(op_or, {.destination = Reg::r10, .source = Reg::r9}, Width::dword);
}
} // namespace dsp56300::detail
