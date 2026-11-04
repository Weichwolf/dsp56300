// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr unsigned op_or = 0x09, op_xor = 0x31, op_cmp = 0x39;
constexpr unsigned group_and = 4, shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_greater = 0x9f, condition_less = 0x9c;
constexpr unsigned move_greater = 0x4f, move_less = 0x4c;
constexpr unsigned aligned_growth_shift = 53, low_word_shift = 40;

std::size_t bank_offset(const unsigned reg) {
  if (reg >= a_register) {
    return offsetof(Cpu, reg) +
           (reg == a_register ? offsetof(Registers, a) : offsetof(Registers, b));
  }
  return offsetof(Cpu, reg) +
         (reg <= x1_register ? offsetof(Registers, x) : offsetof(Registers, y));
}
}

void JitEmitter::move_read_register(const Reg value, const NativeMove &move, const Word mode) {
  const auto reg = move.reg;
  if (reg >= a0_register && reg <= b1_register) {
    constexpr std::array shifts{padding_bits, accumulator_bits, main_shift};
    current_value(value, reg & 1U);
    const bool sign_extended = reg == a2_register || reg == b2_register;
    code_.shift(
        sign_extended ? shift_arithmetic : shift_right, value, shifts.at((reg - a0_register) / 2));
    code_.immediate(group_and, value, word_mask, Width::dword);
    return;
  }
  if (reg >= a_register) {
    current_value(value, reg == a_register ? 0U : 1U);
  } else {
    current_value(value, reg <= x1_register ? 2U : 3U);
  }
  if (reg < a_register) {
    if ((reg & 1U) != 0) { code_.shift(shift_right, value, word_bits); }
    code_.immediate(group_and, value, word_mask, Width::dword);
    return;
  }
  const unsigned scaling = ((mode & scale_down) != 0 ? 1U : 0U);
  const unsigned up = ((mode & scale_up) != 0 ? 1U : 0U);
  const unsigned growth_shift = aligned_growth_shift + scaling - up;
  code_.mov({.destination = Reg::r9, .source = value});
  code_.shift(shift_right, Reg::r9, growth_shift);
  code_.mov({.destination = Reg::r10, .source = Reg::r9});
  code_.shift(shift_right, Reg::r10, 1);
  code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::r10});
  code_.immediate(group_and, Reg::r9, 1, Width::dword);
  code_.shift(shift_left, Reg::r9, 7, Width::dword);
  merge_status(Reg::r9);
  code_.shift(shift_arithmetic, value, main_shift + scaling - up);
  code_.constant(Reg::r9, static_cast<std::uint64_t>(maximum_word));
  code_.binary(op_cmp, {.destination = value, .source = Reg::r9});
  code_.set(condition_greater, Reg::r10);
  code_.move_if(move_greater, {.destination = value, .source = Reg::r9});
  code_.constant(Reg::r9, static_cast<std::uint64_t>(std::int64_t{minimum_word}));
  code_.binary(op_cmp, {.destination = value, .source = Reg::r9});
  code_.set(condition_less, Reg::r11);
  code_.move_if(move_less, {.destination = value, .source = Reg::r9});
  code_.binary(op_or, {.destination = Reg::r10, .source = Reg::r11}, Width::dword);
  code_.shift(shift_left, Reg::r10, 6, Width::dword);
  merge_status(Reg::r10);
  code_.immediate(group_and, value, word_mask, Width::dword);
}

void JitEmitter::move_write_register(const Reg value, const unsigned reg) {
  if (reg >= a_register) {
    code_.mov({.destination = Reg::ax, .source = value});
    code_.shift(shift_left, Reg::ax, low_word_shift);
    code_.shift(shift_arithmetic, Reg::ax, padding_bits);
    if (loop_residency_) {
      store_accumulator(reg == b_register, Reg::ax);
      return;
    }
  } else if ((reg & 1U) != 0) {
    if (loop_bank_cache_) {
      current_value(Reg::ax, reg <= x1_register ? 2U : 3U);
    } else {
      code_.load(Reg::ax, Reg::r12, bank_offset(reg), Width::dword);
    }
    code_.immediate(group_and, Reg::ax, word_mask, Width::dword);
    code_.mov({.destination = Reg::r9, .source = value});
    code_.shift(shift_left, Reg::r9, word_bits);
    code_.binary(op_or, {.destination = Reg::ax, .source = Reg::r9});
  } else {
    current_value(Reg::ax, reg <= x1_register ? 2U : 3U);
    code_.immediate(group_and, Reg::ax, ~word_mask);
    code_.binary(op_or, {.destination = Reg::ax, .source = value});
  }
  if (loop_bank_cache_) {
    code_.packed_from<64>(reg <= x1_register ? x64::Xmm::x5 : x64::Xmm::x6, Reg::ax);
  } else {
    code_.store(Reg::r12, bank_offset(reg), Reg::ax);
  }
}
} // namespace dsp56300::detail
