// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::transfer_word(const Reg value, const Word mode) {
  const unsigned shift =
      main_shift + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  constexpr unsigned normal_growth = 53;
  const auto growth_shift = shift + normal_growth - main_shift;
  code_.extract(Reg::x9, value, growth_shift, 1, Width::x64);
  code_.extract(Reg::x10, value, growth_shift + 1, 1, Width::x64);
  code_.alu(Alu::bit_xor, Reg::x9, Reg::x9, Reg::x10);
  code_.shift(Shift::left, Reg::x9, Reg::x9, 7);
  code_.alu(Alu::bit_or, status, status, Reg::x9);
  code_.shift(Shift::arithmetic, Reg::x0, value, shift, Width::x64);
  code_.constant(Reg::x1, static_cast<std::uint64_t>(maximum_word));
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x2, Condition::greater);
  code_.select(Reg::x0, Reg::x1, Reg::x0, Condition::greater, Width::x64);
  code_.constant(Reg::x1, static_cast<std::uint64_t>(static_cast<std::int64_t>(minimum_word)));
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x3, Condition::less);
  code_.select(Reg::x0, Reg::x1, Reg::x0, Condition::less, Width::x64);
  code_.alu(Alu::bit_or, Reg::x2, Reg::x2, Reg::x3);
  code_.shift(Shift::left, Reg::x2, Reg::x2, 6);
  code_.alu(Alu::bit_or, status, status, Reg::x2);
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
}

void Emitter::read_data(const unsigned number, const Word mode) {
  if (number <= y1_register) {
    code_.extract(Reg::x0,
                  number <= x1_register ? bank_x : bank_y,
                  (number & 1U) != 0 ? word_bits : 0,
                  word_bits,
                  Width::x64);
  } else if (number <= b1_register) {
    constexpr std::array shifts{padding_bits, accumulator_bits, main_shift};
    const bool sign = number == a2_register || number == b2_register;
    code_.extract(Reg::x0,
                  (number & 1U) == 0 ? accumulator_a : accumulator_b,
                  shifts.at((number - a0_register) / 2),
                  sign ? padding_bits : word_bits,
                  Width::x64,
                  sign);
    if (sign) { mask(Reg::x0, Reg::x0, word_mask, Width::w32); }
  } else {
    transfer_word(number == a_register ? accumulator_a : accumulator_b, mode);
  }
}

void Emitter::write_data(const unsigned number) {
  if (number == a_register || number == b_register) {
    code_.extract(Reg::x0, Reg::x0, 0, word_bits, Width::x64, true);
    code_.shift(Shift::left,
                number == a_register ? accumulator_a : accumulator_b,
                Reg::x0,
                main_shift,
                Width::x64);
    return;
  }
  Reg target = bank_x;
  unsigned shift = 0;
  unsigned bits = word_bits;
  if (number <= y1_register) {
    target = number <= x1_register ? bank_x : bank_y;
    shift = (number & 1U) != 0 ? word_bits : 0;
  } else {
    target = (number & 1U) == 0 ? accumulator_a : accumulator_b;
    constexpr std::array shifts{padding_bits, accumulator_bits, main_shift};
    shift = shifts.at((number - a0_register) / 2);
    if (shift == accumulator_bits) { bits = padding_bits; }
  }
  const auto field = ((std::uint64_t{1} << bits) - 1) << shift;
  mask(target, target, ~field);
  if (shift != 0) { code_.shift(Shift::left, Reg::x0, Reg::x0, shift, Width::x64); }
  code_.alu(Alu::bit_or, target, target, Reg::x0, Width::x64);
}
}
