// SPDX-License-Identifier: GPL-3.0-only
// Independent ring coordinates and serial MSB-first carry circuit for fixture generators.
#pragma once
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::address_model {
inline constexpr Word linear = 0xffff;

inline Word span(const Word size) {
  Word result = 1;
  while (result < size) { result *= 2; }
  return result;
}

inline Word wrap24(const std::int64_t value) {
  constexpr std::int64_t size = std::int64_t{1} << word_bits;
  return static_cast<Word>((value % size + size) % size);
}

struct ReverseInput {
  Word original = 0, operand = 0;
  bool subtract = false;
};

inline Word reverse_carry(const ReverseInput input) {
  Word result = 0;
  int carry = 0;
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const auto position = remaining - 1;
    const int left = static_cast<int>((input.original >> position) & 1);
    const int right = static_cast<int>((input.operand >> position) & 1);
    const int sum = input.subtract ? left - right - carry : left + right + carry;
    result |= static_cast<Word>((sum + 2) % 2) << position;
    carry = input.subtract ? static_cast<int>(sum < 0) : sum / 2;
  }
  return result;
}

struct Input {
  Word modifier = linear, cursor = 0, offset = 0;
  unsigned mode = 0, index = 0;
};

inline Word updated(const Input input) {
  if (input.mode == 4) { return input.cursor; }
  const bool subtract = input.mode == 0 || input.mode == 2 || input.mode == 7;
  const bool one = input.mode == 2 || input.mode == 3 || input.mode == 7;
  const Word operand = one ? 1 : input.offset;
  const auto modifier = input.modifier % 0x10000;
  if (modifier == 0) {
    return reverse_carry({.original = input.cursor, .operand = operand, .subtract = subtract});
  }
  const auto offset = (subtract ? -1 : 1) * (one ? std::int64_t{1} : signed24(operand));
  if (modifier == linear) { return wrap24(input.cursor + offset); }
  const bool multi = modifier >= 0x8000;
  const auto length = multi ? (modifier % 0x4000) + 1 : modifier + 1;
  const auto alignment = span(length);
  const auto magnitude = offset < 0 ? -offset : offset;
  // FM4.5.3: a positive integer number of aligned blocks changes the block
  // coordinate, including the one-block boundary of a power-of-two ring.
  const auto original = one ? std::int64_t{1} : signed24(operand);
  if (!multi && original > 0 && (original / alignment) * alignment == original) {
    return wrap24(input.cursor + offset);
  }
  if (!multi && magnitude > length) {
    if (offset % alignment != 0) { throw std::logic_error("Unspecified normal-modulo offset"); }
    return wrap24(input.cursor + offset);
  }
  const auto base = (input.cursor / alignment) * alignment;
  const auto coordinate = static_cast<std::int64_t>(input.cursor - base) + offset;
  return base + static_cast<Word>((coordinate % length + length) % length);
}

} // namespace dsp56300::test::address_model
