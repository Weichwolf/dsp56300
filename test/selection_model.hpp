// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-56 comparisons, per-bit complement/status and condition truth table.
#pragma once
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::selection_model {
inline constexpr Word max_code = 0x1d, magnitude_code = 0x15, complement_code = 0x17;
inline constexpr unsigned integer_sign = (word_bits * 2) - 1;
enum class Kind : std::uint8_t { maximum, magnitude, complement, test };

struct Input {
  Kind kind = Kind::maximum;
  bool accumulator_b = false;
};

inline Word code(const Input input) {
  if (input.kind == Kind::maximum) { return max_code; }
  if (input.kind == Kind::magnitude) { return magnitude_code; }
  return (input.kind == Kind::complement ? complement_code : 3U) | (input.accumulator_b ? 8U : 0U);
}

struct ConditionInput {
  Word status = 0;
  unsigned selector = 0;
};

inline bool test_condition(const ConditionInput input) {
  const auto status = input.status;
  const bool carry = (status & flag_c) != 0;
  const bool zero = (status & flag_z) != 0;
  const bool less = ((status & flag_n) != 0) != ((status & flag_v) != 0);
  const bool normalized = zero || (status & (flag_u | flag_e)) == 0;
  const std::array table{!carry,
                         !less,
                         !zero,
                         (status & flag_n) == 0,
                         !normalized,
                         (status & flag_e) == 0,
                         (status & flag_l) == 0,
                         !zero && !less,
                         carry,
                         less,
                         zero,
                         (status & flag_n) != 0,
                         normalized,
                         (status & flag_e) != 0,
                         (status & flag_l) != 0,
                         zero || less};
  return table.at(input.selector);
}

inline void select(Registers &state, const Registers &old, const Kind kind) {
  const auto left = signed56(old.a);
  const auto right = signed56(old.b);
  const auto left_magnitude = left < 0 ? -left : left;
  const auto right_magnitude = right < 0 ? -right : right;
  const bool transfer = kind == Kind::maximum ? left >= right : left_magnitude >= right_magnitude;
  if (transfer) { state.b = old.a; }
  state.sr = (state.sr & ~flag_c) | (transfer ? 0U : flag_c);
}

inline void complement(Registers &state, const std::uint64_t value, const bool accumulator_b) {
  std::uint64_t result = 0;
  bool word_zero = true;
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const bool middle = bit >= word_bits && bit < word_bits * 2;
    const bool next = (((value >> bit) & 1U) != 0) != middle;
    if (middle && next) { word_zero = false; }
    if (next) { result += std::uint64_t{1} << bit; }
  }
  (accumulator_b ? state.b : state.a) = result;
  Word flags = word_zero ? flag_z : 0;
  if (((result >> integer_sign) & 1U) != 0) { flags |= flag_n; }
  state.sr = (state.sr & ~(flag_v | flag_n | flag_z)) | flags;
}

inline void test(Registers &state, const std::uint64_t value) {
  const unsigned boundary = integer_sign + ((state.sr & mode_down) != 0 ? 1U : 0U) -
                            ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto bit = [value](const unsigned position) { return (value >> position) % 2; };
  const auto sign = bit(accumulator_bits - 1);
  Word flags = value == 0 ? flag_z : 0;
  if (sign != 0) { flags |= flag_n; }
  if (bit(boundary) == bit(boundary - 1)) { flags |= flag_u; }
  for (unsigned position = boundary; position < accumulator_bits - 1; ++position) {
    if (bit(position) != sign) { flags |= flag_e; }
  }
  state.sr = (state.sr & ~(flag_v | flag_n | flag_z | flag_e | flag_u)) | flags;
}

inline void apply(Registers &state, const Registers &old, const Input input) {
  if (input.kind == Kind::maximum || input.kind == Kind::magnitude) {
    select(state, old, input.kind);
    return;
  }
  const auto value = input.accumulator_b ? old.b : old.a;
  if (input.kind == Kind::complement) {
    complement(state, value, input.accumulator_b);
  } else {
    test(state, value);
  }
}
} // namespace dsp56300::test::selection_model
