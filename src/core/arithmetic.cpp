// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "rounding.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <optional>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr unsigned main_word_shift = 32;
constexpr unsigned normal_integer_shift = 55;
constexpr unsigned normal_growth_shift = 53;
constexpr std::uint64_t main_word_mask = std::uint64_t{word_mask} << main_word_shift;
constexpr unsigned multiply_shift = padding_bits + 1;
constexpr unsigned operation_mask = 0xf7;

enum AluCode : std::uint8_t {
  op_clear = 0x13,
  op_round = 0x11,
  op_abs = 0x26,
  op_negate = 0x36,
  op_asr = 0x22,
  op_asl = 0x32,
  op_not = 0x17,
  op_tfr_x0 = 0x41,
  op_tfr_y0 = 0x51,
  op_tfr_x1 = 0x61,
  op_tfr_y1 = 0x71,
  op_max = 0x1d,
  op_maxm = 0x15,
};

bool signed_overflow(const std::uint64_t left,
                     const std::uint64_t right,
                     const std::uint64_t result) noexcept {
  return ((left ^ result) & (right ^ result)) >> host_sign_bit != 0;
}

unsigned integer_shift(const Word status) noexcept {
  return normal_integer_shift + ((status & scale_down) != 0 ? 1U : 0U) -
         ((status & scale_up) != 0 ? 1U : 0U);
}

std::uint64_t magnitude(const std::uint64_t value) noexcept {
  return (value >> host_sign_bit) != 0 ? wrapped_sub(0, value) : value;
}
}

void Core::Impl::arithmetic_flags(const std::uint64_t value, const bool overflowed) {
  const auto shift = integer_shift(cpu.reg.sr);
  const auto integer = value >> shift;
  const auto all_ones = ~std::uint64_t{0} >> shift;
  Word flags = value == 0 ? zero : 0;
  if ((value >> host_sign_bit) != 0) { flags |= negative; }
  if (integer != 0 && integer != all_ones) { flags |= extension; }
  if (((value >> shift) & 1) == ((value >> (shift - 1)) & 1)) { flags |= unnormalized; }
  if (overflowed) { flags |= overflow | limit; }
  constexpr Word changed = negative | zero | extension | unnormalized | overflow;
  mark_alu(RegisterStorage::sr, changed | (overflowed ? limit : 0));
  cpu.reg.sr = (cpu.reg.sr & ~changed) | flags;
}

void Core::Impl::logical_flags(const Word value) {
  Word flags = value == 0 ? zero : 0;
  if ((value & sign24_bit) != 0) { flags |= negative; }
  mark_alu(RegisterStorage::sr, negative | zero | overflow);
  cpu.reg.sr = (cpu.reg.sr & ~(negative | zero | overflow)) | flags;
}

Word Core::Impl::transfer_word(const std::uint64_t value,
                               const std::optional<unsigned> source_register) {
  const unsigned shift = main_word_shift + ((cpu.reg.sr & scale_down) != 0 ? 1U : 0U) -
                         ((cpu.reg.sr & scale_up) != 0 ? 1U : 0U);
  const auto result = std::bit_cast<std::int64_t>(value) >> shift;
  const auto growth_shift = normal_growth_shift + ((cpu.reg.sr & scale_down) != 0 ? 1U : 0U) -
                            ((cpu.reg.sr & scale_up) != 0 ? 1U : 0U);
  Word flags = 0;
  if ((((value >> growth_shift) ^ (value >> (growth_shift + 1))) & 1) != 0) { flags |= growth; }
  Word word = static_cast<Word>(result) & word_mask;
  if (result > maximum_word) {
    flags |= limit;
    word = static_cast<Word>(maximum_word);
  } else if (result < minimum_word) {
    flags |= limit;
    word = sign24_bit;
  }
  transfer_status(flags, source_register);
  return word;
}

void Core::Impl::round(std::uint64_t &value) {
  const auto result = round_value({.value = value, .status = cpu.reg.sr});
  value = result.value;
  arithmetic_flags(value, result.overflowed);
  if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(value, true); }
}

std::uint64_t Core::Impl::source(const unsigned selector, const bool destination_b) const {
  const auto x = latched[2];
  const auto y = latched[3];
  switch (selector) {
    case 0:
    case 1: return latched[destination_b ? 0 : 1];
    case 2:
      return ((x & (std::uint64_t{1} << (double_word_bits - 1))) != 0 ? x | ~long_mask : x)
             << padding_bits;
    case 3:
      return ((y & (std::uint64_t{1} << (double_word_bits - 1))) != 0 ? y | ~long_mask : y)
             << padding_bits;
    case 4: return aligned_word(static_cast<Word>(x) & word_mask);
    case 5: return aligned_word(static_cast<Word>(y) & word_mask);
    case 6: return aligned_word(static_cast<Word>(x >> word_bits));
    case 7: return aligned_word(static_cast<Word>(y >> word_bits));
    default: return 0;
  }
}

void Core::Impl::multiply(const Word opcode) {
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = latched[destination_b ? 1 : 0];
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const auto pair = pairs[(opcode >> 4) & 7];
  const std::array inputs{static_cast<Word>(latched[2]) & word_mask,
                          static_cast<Word>(latched[2] >> word_bits),
                          static_cast<Word>(latched[3]) & word_mask,
                          static_cast<Word>(latched[3] >> word_bits)};
  const auto product =
      static_cast<std::int64_t>(signed_word(inputs[pair[0]])) * signed_word(inputs[pair[1]]);
  auto term = static_cast<std::uint64_t>(product) << multiply_shift;
  if ((opcode & 4U) != 0) { term = wrapped_sub(0, term); }
  const auto initial = (opcode & 2U) != 0 ? before : 0;
  mark_alu_accumulator(destination_b);
  destination = wrapped_add(initial, term);
  const bool overflowed = signed_overflow(initial, term, destination);
  if ((opcode & 1U) != 0) {
    const auto result = round_value(
        {.value = destination, .status = cpu.reg.sr, .arithmetic_overflow = overflowed});
    destination = result.value;
    arithmetic_flags(destination, result.overflowed);
    if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(destination, true); }
  } else {
    arithmetic_flags(destination, overflowed);
    if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(destination); }
  }
}

bool Core::Impl::logic(const Word opcode) {
  const auto operation = opcode & operation_mask;
  const auto group = (opcode >> 4) & 7;
  const auto kind = opcode & 7;
  const bool binary = group >= 4 && (kind == 2 || kind == 3 || kind == 6);
  if (!binary && operation != op_not) { return false; }
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = latched[destination_b ? 1 : 0];
  auto word = static_cast<Word>(before >> main_word_shift) & word_mask;
  const auto input = static_cast<Word>(source(group, destination_b) >> main_word_shift) & word_mask;
  if (operation == op_not) {
    word ^= word_mask;
  } else if (kind == 2) {
    word |= input;
  } else if (kind == 3) {
    word ^= input;
  } else {
    word &= input;
  }
  mark_alu_accumulator(destination_b, main_word_mask >> detail::padding_bits);
  destination = (destination & ~main_word_mask) | (std::uint64_t{word} << main_word_shift);
  logical_flags(word);
  return true;
}

bool Core::Impl::unary(const Word opcode) {
  const auto operation = opcode & operation_mask;
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = latched[destination_b ? 1 : 0];
  switch (operation) {
    case 0: return true;
    case 1:
    case op_tfr_x0:
    case op_tfr_y0:
    case op_tfr_x1:
    case op_tfr_y1:
      mark_alu_accumulator(destination_b);
      destination = source((opcode >> 4) & 7, destination_b);
      return true;
    case op_clear:
      mark_alu_accumulator(destination_b);
      destination = 0;
      arithmetic_flags(0, false);
      return true;
    case 3: {
      auto result = before;
      arithmetic_flags(result, false);
      saturate_arithmetic(result);
      return true;
    }
    case op_round:
      mark_alu_accumulator(destination_b);
      destination = before;
      round(destination);
      return true;
    case op_abs:
      mark_alu_accumulator(destination_b);
      destination = magnitude(before);
      arithmetic_flags(destination, before == (std::uint64_t{1} << host_sign_bit));
      if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(destination); }
      return true;
    case op_negate:
      mark_alu_accumulator(destination_b);
      destination = wrapped_sub(0, before);
      arithmetic_flags(destination, before == (std::uint64_t{1} << host_sign_bit));
      if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(destination); }
      return true;
    case op_asr:
    case op_asl:
      arithmetic_shift(before, {.destination_b = destination_b, .left = operation == op_asl});
      return true;
    default: return word_shift(opcode);
  }
}

bool Core::Impl::addition(const Word opcode) {
  const unsigned group = (opcode >> 4) & 7;
  const unsigned kind = opcode & 7;
  const bool with_carry = (group == 2 || group == 3) && (kind == 1 || kind == 5);
  const bool compare_magnitude = kind == 7 && (group == 0 || group >= 4);
  const bool compare = compare_magnitude || (kind == 5 && (group == 0 || group >= 4));
  const bool add = (group >= 1 && kind == 0) || (with_carry && kind == 1);
  const bool subtract = (group >= 1 && kind == 4) || compare || (with_carry && kind == 5);
  if (!add && !subtract) { return false; }
  const bool destination_b = (opcode & 8U) != 0;
  const auto original = latched[destination_b ? 1 : 0];
  const auto input = source(group, destination_b);
  if (compare_magnitude) {
    const auto left = magnitude(original);
    const auto right = magnitude(input);
    auto result = wrapped_sub(left, right);
    // Magnitudes occupy [0,2^55], including the unsigned magnitude of min56.
    // Only a positive difference outside signed56 overflows; 0-|min56| fits.
    arithmetic_flags(result, left > right && (result >> host_sign_bit) != 0);
    if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(result); }
    mark_alu(RegisterStorage::sr, carry);
    cpu.reg.sr = (cpu.reg.sr & ~carry) | (left < right ? carry : 0);
    return true;
  }
  add_operands(
      {.left = original,
       .right = input,
       .carry_in = with_carry && (cpu.reg.sr & carry) != 0 ? std::uint64_t{1} << padding_bits : 0,
       .destination_b = destination_b,
       .subtract = subtract,
       .compare = compare});
  return true;
}

void Core::Impl::add_operands(const AddOperands input) {
  const auto before = input.left;
  const auto operand = input.right;
  const auto intermediate =
      input.subtract ? wrapped_sub(before, operand) : wrapped_add(before, operand);
  auto result = input.subtract ? wrapped_sub(intermediate, input.carry_in)
                               : wrapped_add(intermediate, input.carry_in);
  const bool overflowed = input.subtract
                              ? ((before ^ operand) & (before ^ result)) >> host_sign_bit != 0
                              : signed_overflow(before, operand, result);
  const bool carried = input.subtract ? (before < operand || intermediate < input.carry_in)
                                      : (intermediate < before || result < intermediate);
  arithmetic_flags(result, input.shift_overflow || overflowed);
  if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(result); }
  mark_alu(RegisterStorage::sr, carry);
  cpu.reg.sr = (cpu.reg.sr & ~carry) | (carried ? carry : 0);
  if (!input.compare) {
    mark_alu_accumulator(input.destination_b);
    (input.destination_b ? cpu.reg.b : cpu.reg.a) = result;
  }
}

bool Core::Impl::shifted_addition(const Word opcode) {
  const auto operation = opcode & 0xf7U;
  constexpr Word add_right = 0x02;
  constexpr Word subtract_right = 0x06;
  constexpr Word add_left = 0x12;
  constexpr Word subtract_left = 0x16;
  if (operation != add_right && operation != subtract_right && operation != add_left &&
      operation != subtract_left) {
    return false;
  }
  const bool destination_b = (opcode & 8U) != 0;
  const bool left = (opcode & 16U) != 0;
  const auto before = latched[destination_b ? 1 : 0];
  const auto shifted =
      left ? before << 1
           : static_cast<std::uint64_t>(std::bit_cast<std::int64_t>(before) >> 1) & padding_mask;
  add_operands({.left = shifted,
                .right = latched[destination_b ? 0 : 1],
                .destination_b = destination_b,
                .subtract = (opcode & 4U) != 0,
                .shift_overflow = left && ((before ^ shifted) >> host_sign_bit) != 0});
  return true;
}

void Core::Impl::alu(const Word opcode) {
  if (opcode == 8) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  if ((opcode & 0x80U) != 0) {
    multiply(opcode);
    return;
  }
  if (opcode == op_max || opcode == op_maxm) {
    const bool transfer = opcode == op_max ? std::bit_cast<std::int64_t>(latched[0]) >=
                                                 std::bit_cast<std::int64_t>(latched[1])
                                           : magnitude(latched[0]) >= magnitude(latched[1]);
    if (transfer) {
      mark_alu_accumulator(true);
      cpu.reg.b = latched[0];
    }
    mark_alu(RegisterStorage::sr, carry);
    cpu.reg.sr = (cpu.reg.sr & ~carry) | (transfer ? 0 : carry);
    return;
  }
  if (unary(opcode) || logic(opcode) || addition(opcode) || shifted_addition(opcode)) { return; }
  fail(FaultKind::unsupported_instruction, Space::program, current_pc, opcode);
}
} // namespace dsp56300
