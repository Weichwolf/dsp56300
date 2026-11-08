// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/value_ranges.hpp"

#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::round(const Word mode) {
  constexpr unsigned main_shift = word_bits + padding_bits;
  const unsigned cleared =
      main_shift + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  if (guarded() && value_proof_.rounding_zero_bits >= cleared - padding_bits) { return; }
  const auto half = std::uint64_t{1} << (cleared - 1);
  constexpr auto unit = std::uint64_t{1} << padding_bits;
  code_.constant(Reg::x5, (mode & rounding_mode) != 0 ? half : half - unit);
  if ((mode & rounding_mode) == 0) {
    code_.extract(Reg::x6, Reg::x0, cleared, 1, Width::x64);
    code_.shift(Shift::left, Reg::x6, Reg::x6, padding_bits, Width::x64);
    code_.alu(Alu::add, Reg::x5, Reg::x5, Reg::x6, Width::x64);
  }
  code_.alu(Alu::add_flags, Reg::x0, Reg::x0, Reg::x5, Width::x64);
  code_.set(Reg::x5, Condition::overflow);
  code_.alu(Alu::bit_xor, Reg::x3, Reg::x3, Reg::x5);
  mask(Reg::x0, Reg::x0, ~std::uint64_t{0} << cleared);
}

void Emitter::saturate(const Word mode, const bool rounded) {
  if ((mode & arithmetic_saturation) == 0 ||
      (guarded() && value_proof_.saturation == SaturationProof::none)) {
    return;
  }
  constexpr unsigned high_word_sign = double_word_bits - 1 + padding_bits;
  Label complete;
  code_.extract(Reg::x9, Reg::x0, host_sign_bit, 1, Width::x64);
  code_.constant(Reg::x17, 3);
  code_.multiply(Reg::x10, Reg::x9, Reg::x17);
  code_.extract(Reg::x11, Reg::x0, high_word_sign, 2, Width::x64);
  code_.compare(Reg::x10, Reg::x11);
  code_.branch_far(Condition::equal, complete);
  constexpr std::uint64_t positive = 0x007fffffffffff00;
  constexpr std::uint64_t negative_limit = 0xff80000000000000;
  constexpr std::uint64_t rounded_positive = 0x007fffff00000000;
  code_.constant(Reg::x10, rounded ? rounded_positive : positive);
  code_.constant(Reg::x11, negative_limit);
  code_.compare(Reg::x9, Reg::zr);
  code_.select(Reg::x0, Reg::x11, Reg::x10, Condition::not_equal, Width::x64);
  code_.constant(Reg::x3, 1);
  code_.bind(complete);
}

void Emitter::arithmetic_flags(const Word mode, const bool carry_changed) {
  constexpr unsigned normal_integer_shift = 55;
  const unsigned shift = normal_integer_shift + ((mode & scale_down) != 0 ? 1U : 0U) -
                         ((mode & scale_up) != 0 ? 1U : 0U);
  constexpr Word arithmetic_changed = negative | zero | extension | unnormalized | overflow;
  const auto changed = (arithmetic_changed | (carry_changed ? carry : 0)) & live_flags_;
  const auto width = guarded() ? value_proof_.width : accumulator_bits;
  const auto merge = [this](const Reg value, const unsigned position) {
    code_.shift(Shift::left, Reg::x9, value, position);
    code_.alu(Alu::bit_or, status, status, Reg::x9);
  };
  if (changed != 0) { mask(status, status, word_mask ^ changed, Width::w32); }
  if ((changed & negative) != 0 && width != 0) {
    code_.extract(Reg::x9, Reg::x0, host_sign_bit, 1, Width::x64);
    merge(Reg::x9, 3);
  }
  if ((changed & zero) != 0) {
    if (width == 0) {
      code_.constant(Reg::x9, 1);
    } else {
      code_.compare(Reg::x0, Reg::zr, Width::x64);
      code_.set(Reg::x9, Condition::equal);
    }
    merge(Reg::x9, 2);
  }
  if ((changed & extension) != 0 && width > shift - padding_bits + 1) {
    code_.shift(Shift::arithmetic, Reg::x9, Reg::x0, shift, Width::x64);
    code_.compare(Reg::x9, Reg::zr, Width::x64);
    code_.set(Reg::x10, Condition::not_equal);
    code_.constant(Reg::x17, ~std::uint64_t{0});
    code_.compare(Reg::x9, Reg::x17, Width::x64);
    code_.set(Reg::x9, Condition::not_equal);
    code_.alu(Alu::bit_and, Reg::x9, Reg::x9, Reg::x10);
    merge(Reg::x9, 5);
  }
  if ((changed & unnormalized) != 0) {
    if (width <= shift - padding_bits) {
      code_.constant(Reg::x9, 1);
    } else {
      code_.extract(Reg::x9, Reg::x0, shift, 1, Width::x64);
      code_.extract(Reg::x10, Reg::x0, shift - 1, 1, Width::x64);
      code_.compare(Reg::x9, Reg::x10);
      code_.set(Reg::x9, Condition::equal);
    }
    merge(Reg::x9, 4);
  }
  if (!guarded() || value_proof_.overflow_value()) {
    code_.constant(Reg::x17, limit | (changed & overflow));
    code_.multiply(Reg::x9, Reg::x3, Reg::x17);
    code_.alu(Alu::bit_or, status, status, Reg::x9);
  }
  if (carry_changed && (changed & carry) != 0) { code_.alu(Alu::bit_or, status, status, Reg::x4); }
}

void Emitter::logical_flags(const Reg value) {
  const auto changed = (negative | zero | overflow) & live_flags_;
  if (changed == 0) { return; }
  mask(status, status, word_mask ^ changed, Width::w32);
  if ((changed & negative) != 0) {
    code_.extract(Reg::x9, value, word_bits - 1, 1);
    code_.shift(Shift::left, Reg::x9, Reg::x9, 3);
    code_.alu(Alu::bit_or, status, status, Reg::x9);
  }
  if ((changed & zero) != 0) {
    code_.compare(value, Reg::zr);
    code_.set(Reg::x9, Condition::equal);
    code_.shift(Shift::left, Reg::x9, Reg::x9, 2);
    code_.alu(Alu::bit_or, status, status, Reg::x9);
  }
}

void Emitter::complete(const BlockInstruction &operation,
                       const bool compare,
                       const bool carried,
                       const bool rounded) {
  if (rounded) { round(operation.mode); }
  saturate(operation.mode, rounded);
  arithmetic_flags(operation.mode, carried);
  if (!compare) { code_.move(destination(operation.instruction.opcode), Reg::x0); }
}
}
