// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "move.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr unsigned op_and = 0x21, op_or = 0x09, op_xor = 0x31;
constexpr unsigned group_add = 0, group_and = 4, group_xor = 6, group_cmp = 7;
constexpr unsigned shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned aligned_growth_shift = 53, low_word_shift = 40;

std::size_t bank_offset(const unsigned reg) {
  if (reg >= r_register_base && reg < m_register_base) {
    const bool offset = reg >= n_register_base;
    const auto index = reg - (offset ? n_register_base : r_register_base);
    return offsetof(Cpu, reg) + (offset ? offsetof(Registers, n) : offsetof(Registers, r)) +
           (index * sizeof(Word));
  }
  if (reg >= a_register) {
    return offsetof(Cpu, reg) +
           (reg == a_register ? offsetof(Registers, a) : offsetof(Registers, b));
  }
  return offsetof(Cpu, reg) +
         (reg <= x1_register ? offsetof(Registers, x) : offsetof(Registers, y));
}
}

void BlockEmitter::move_read_register(const Reg value, const NativeMove &move, const Word mode) {
  const auto reg = move.reg;
  if (reg >= r_register_base && reg < m_register_base) {
    code_.load(value, Reg::r12, bank_offset(reg), Width::dword);
    return;
  }
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
  const auto width = value_proof_.transfer_widths.at(reg == a_register ? 0U : 1U);
  constexpr unsigned growth_width = aligned_growth_shift - padding_bits + 1;
  if (width > growth_width + scaling - up) {
    code_.mov({.destination = Reg::r9, .source = value});
    if (deferred_growth_shift_ != 0) {
      // Helper-free regions observe S only at exit. OR adjacent-bit changes
      // across transfers before extracting the region's fixed scaling bit.
      code_.shift(shift_right, Reg::r9, 1);
      code_.binary(op_xor, {.destination = Reg::r9, .source = value});
      code_.packed_from<64>(x64::Xmm::x7, Reg::r9);
      code_.packed(x64::Packed::bit_or, x64::Xmm::x12, x64::Xmm::x7);
    } else {
      code_.shift(shift_right, Reg::r9, growth_shift);
      code_.mov({.destination = Reg::r10, .source = Reg::r9});
      code_.shift(shift_right, Reg::r10, 1);
      code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::r10});
      code_.immediate(group_and, Reg::r9, 1, Width::dword);
      code_.shift(shift_left, Reg::r9, 7, Width::dword);
      merge_status(Reg::r9);
    }
  }
  code_.shift(shift_arithmetic, value, main_shift + scaling - up);
  // This is the pre-transfer accumulator width, before any parallel ALU/write.
  // Growth-S above is independent; a proved signed24 result cannot set limit-L.
  if (width <= double_word_bits + scaling - up) {
    code_.immediate(group_and, value, word_mask, Width::dword);
    return;
  }
  x64::Label fits;
  // Unsigned (value + 2^23) <= 2^24-1 tests both signed24 bounds at once.
  // Only a clipped transfer changes sticky L; the fitting path needs no merge.
  code_.mov({.destination = Reg::r9, .source = value});
  code_.immediate(group_add, Reg::r9, sign24_bit);
  code_.immediate(group_cmp, Reg::r9, word_mask);
  code_.jump_if(x64::Branch::below_equal, fits);
  code_.mov({.destination = Reg::r9, .source = value});
  code_.shift(shift_arithmetic, Reg::r9, host_sign_bit);
  code_.immediate(group_xor, Reg::r9, static_cast<Word>(maximum_word));
  code_.mov({.destination = value, .source = Reg::r9});
  code_.constant(Reg::r10, limit);
  merge_status(Reg::r10);
  code_.bind(fits);
  code_.immediate(group_and, value, word_mask, Width::dword);
}

void BlockEmitter::move_write_register(const Reg value, const unsigned reg) {
  if (reg >= r_register_base && reg < m_register_base) {
    code_.store(Reg::r12, bank_offset(reg), value, Width::dword);
    return;
  }
  if (reg >= a0_register && reg <= b1_register) {
    constexpr std::array shifts{padding_bits, accumulator_bits, main_shift};
    constexpr Word extension_mask = 0xff;
    const auto shift = shifts.at((reg - a0_register) / 2);
    const auto mask = reg == a2_register || reg == b2_register ? extension_mask : word_mask;
    code_.mov({.destination = Reg::r9, .source = value});
    code_.immediate(group_and, Reg::r9, mask, Width::dword);
    code_.shift(shift_left, Reg::r9, shift);
    current_value(Reg::ax, reg & 1U);
    code_.constant(Reg::r10, ~(std::uint64_t{mask} << shift));
    code_.binary(op_and, {.destination = Reg::ax, .source = Reg::r10});
    code_.binary(op_or, {.destination = Reg::ax, .source = Reg::r9});
    store_accumulator((reg & 1U) != 0, Reg::ax);
    return;
  }
  if (reg >= a_register) {
    code_.mov({.destination = Reg::ax, .source = value});
    code_.shift(shift_left, Reg::ax, low_word_shift);
    code_.shift(shift_arithmetic, Reg::ax, padding_bits);
    store_accumulator(reg == b_register, Reg::ax);
    return;
  }
  if ((reg & 1U) != 0) {
    code_.load(Reg::ax, Reg::r12, bank_offset(reg), Width::dword);
    code_.immediate(group_and, Reg::ax, word_mask, Width::dword);
    code_.mov({.destination = Reg::r9, .source = value});
    code_.shift(shift_left, Reg::r9, word_bits);
    code_.binary(op_or, {.destination = Reg::ax, .source = Reg::r9});
  } else {
    current_value(Reg::ax, reg <= x1_register ? 2U : 3U);
    code_.immediate(group_and, Reg::ax, ~word_mask);
    code_.binary(op_or, {.destination = Reg::ax, .source = value});
  }
  code_.store(Reg::r12, bank_offset(reg), Reg::ax);
}
} // namespace dsp56300::detail
