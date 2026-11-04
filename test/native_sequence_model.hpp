// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-56 signed arithmetic model for directed native sequences.
#pragma once
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::sequence_model {
constexpr unsigned sign48_position = 47;
constexpr Word arithmetic_changed = flag_v | flag_z | flag_n | flag_e | flag_u;
constexpr Word alu_mask = 0xf7, clear = 0x13, round = 0x11, absolute = 0x26, negate = 0x36;
constexpr Word right_shift = 0x22;
constexpr Word tfr_other = 0x01, tfr_x0 = 0x41, tfr_y0 = 0x51, tfr_x1 = 0x61, tfr_y1 = 0x71;

inline std::int64_t signed48(const std::uint64_t raw) {
  constexpr auto sign = std::uint64_t{1} << sign48_position;
  return raw < sign ? static_cast<std::int64_t>(raw)
                    : static_cast<std::int64_t>(raw) - static_cast<std::int64_t>(long_mask) - 1;
}

struct Arithmetic {
  std::int64_t full = 0;
  bool overflow = false, carry_changed = false, carry = false, compare = false;
};

inline bool overflowed(const std::int64_t full) {
  return full < -static_cast<std::int64_t>(sign56) || full >= static_cast<std::int64_t>(sign56);
}

inline std::int64_t source(const Registers &old, const Word alu) {
  const auto selector = (alu >> 4) & 7;
  if (selector < 2) { return signed56((alu & 8U) != 0 ? old.a : old.b); }
  if (selector < 4) { return signed48(selector == 2 ? old.x : old.y); }
  const auto bank = (selector & 1U) == 0 ? old.x : old.y;
  const auto word = static_cast<Word>(selector < 6 ? bank : bank >> word_bits) & word_mask;
  return signed24(word) * (std::int64_t{1} << word_bits);
}

inline void rounded(Arithmetic &result, const Word status) {
  const unsigned bits =
      word_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  const auto widened = result.full;
  auto quotient = widened / unit;
  auto remainder = widened % unit;
  if (remainder < 0) {
    --quotient;
    remainder += unit;
  }
  if (remainder > unit / 2 ||
      (remainder == unit / 2 && ((status & mode_rm) != 0 || quotient % 2 != 0))) {
    ++quotient;
  }
  result.full = quotient * unit;
  result.overflow = overflowed(result.full);
}

inline Arithmetic multiplication(const Registers &old, const Word alu) {
  // Selector coordinates are X0/X1/Y0/Y1, expressed without native bank masks.
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const std::array values{static_cast<Word>(old.x) & word_mask,
                          static_cast<Word>(old.x >> word_bits),
                          static_cast<Word>(old.y) & word_mask,
                          static_cast<Word>(old.y >> word_bits)};
  const auto pair = pairs.at((alu >> 4) & 7);
  Arithmetic result;
  result.full = 2 * signed24(values.at(pair[0])) * signed24(values.at(pair[1]));
  if ((alu & 4U) != 0) { result.full = -result.full; }
  if ((alu & 2U) != 0) { result.full += signed56((alu & 8U) != 0 ? old.b : old.a); }
  result.overflow = overflowed(result.full);
  if ((alu & 1U) != 0) { rounded(result, old.sr); }
  return result;
}

inline Arithmetic addition(const Registers &old, const Word alu) {
  const auto kind = alu & 7U;
  const auto selector = (alu >> 4) & 7;
  const bool with_carry = (selector == 2 || selector == 3) && (kind == 1 || kind == 5);
  const bool subtract = kind == 4 || kind == 5;
  const auto left = (alu & 8U) != 0 ? old.b : old.a;
  const auto right = source(old, alu);
  const auto carry = with_carry && (old.sr & flag_c) != 0 ? 1 : 0;
  const auto raw_right = static_cast<std::uint64_t>(right) & accumulator_mask;
  const auto extra = static_cast<std::uint64_t>(carry);
  Arithmetic result;
  result.full = signed56(left) + (subtract ? -right - carry : right + carry);
  result.overflow = overflowed(result.full);
  result.carry_changed = true;
  result.carry = subtract ? left < raw_right + extra : left + raw_right + extra > accumulator_mask;
  result.compare = kind == 5 && !with_carry;
  return result;
}

inline void flags(Registers &state, const Arithmetic result) {
  const auto raw = static_cast<std::uint64_t>(result.full) & accumulator_mask;
  const auto value = signed56(raw);
  const unsigned position = sign48_position + ((state.sr & mode_down) != 0 ? 1U : 0U) -
                            ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto boundary = std::int64_t{1} << position;
  Word next = value == 0 ? flag_z : 0;
  if (value < 0) { next |= flag_n; }
  if (value < -boundary || value >= boundary) { next |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { next |= flag_u; }
  if (result.overflow) { next |= flag_v | flag_l; }
  const auto changed = arithmetic_changed | (result.carry_changed ? flag_c : 0);
  if (result.carry_changed && result.carry) { next |= flag_c; }
  state.sr = (state.sr & ~changed) | next;
}

inline void alu(Registers &state, const Registers &old, const Word opcode) {
  const auto code = opcode & status_byte;
  if (code == 0) { return; }
  const auto kind = code & alu_mask;
  auto &destination = (code & 8U) != 0 ? state.b : state.a;
  if (kind == tfr_other || kind == tfr_x0 || kind == tfr_y0 || kind == tfr_x1 || kind == tfr_y1) {
    destination = static_cast<std::uint64_t>(source(old, code)) & accumulator_mask;
    return;
  }
  Arithmetic result;
  if ((code & 0x80U) != 0) {
    result = multiplication(old, code);
  } else if (kind == right_shift) {
    const auto raw = (code & 8U) != 0 ? old.b : old.a;
    const auto value = signed56(raw);
    result.full = value / 2 - (value % 2 < 0 ? 1 : 0);
    result.carry_changed = true;
    result.carry = raw % 2 != 0;
  } else if (kind == clear) {
    result.full = 0;
  } else if (kind == absolute || kind == negate || kind == round) {
    const auto value = signed56((code & 8U) != 0 ? old.b : old.a);
    result.full = -value;
    if (kind == absolute && value >= 0) { result.full = value; }
    if (kind == round) {
      result.full = value;
      rounded(result, old.sr);
    }
    result.overflow = overflowed(result.full);
  } else {
    result = addition(old, code);
  }
  if (!result.compare) { destination = static_cast<std::uint64_t>(result.full) & accumulator_mask; }
  flags(state, result);
}
} // namespace dsp56300::test::sequence_model
