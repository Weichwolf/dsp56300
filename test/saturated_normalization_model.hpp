// SPDX-License-Identifier: GPL-3.0-only
// Independent raw56 bit-serial shifts and explicit old-CCR action selection.
#pragma once
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::normalization_model {
enum class Kind : std::uint8_t { iteration, fast };

struct Input {
  Kind kind = Kind::fast;
  unsigned selector = 0;
  bool destination_b = false;
};

inline constexpr std::array sources{12U, 13U, 4U, 5U, 6U, 7U};
inline constexpr std::array codes{2U, 3U, 4U, 6U, 5U, 7U};
inline constexpr int minimum_count = -55, maximum_count = 56;
inline constexpr Word iteration_base = 0x01d815, fast_base = 0x0c1e20;

inline Word opcode(const Input input) {
  if (input.kind == Kind::iteration) {
    return iteration_base | (input.selector << 8) | (input.destination_b ? 8U : 0U);
  }
  return fast_base | (codes.at(input.selector) * 2) | (input.destination_b ? 1U : 0U);
}

inline Word cycles(const Input input) {
  return input.kind == Kind::iteration ? 5U : 1U;
}

inline Word source(const Registers &state, const unsigned selector) {
  const std::array words{state.a >> word_bits,
                         state.b >> word_bits,
                         state.x,
                         state.x >> word_bits,
                         state.y,
                         state.y >> word_bits};
  return static_cast<Word>(words.at(selector) & word_mask);
}

inline void
shift(Registers &state, const int count, const bool destination_b, const bool saturate) {
  if (count < minimum_count || count > maximum_count) {
    throw std::logic_error("Unqualified normalization count");
  }
  const auto carry = state.sr & flag_c;
  if (count == maximum_count) {
    auto &value = destination_b ? state.b : state.a;
    value = value < sign56 ? 0 : accumulator_mask;
    sequence_model::flags(state, {.full = signed56(value)});
  } else {
    shift_model::apply(state,
                       state,
                       {.count = static_cast<unsigned>(count < 0 ? -count : count),
                        .left = count < 0,
                        .source_b = destination_b,
                        .destination_b = destination_b,
                        .saturate = saturate});
  }
  state.sr = (state.sr & ~flag_c) | carry;
}

inline void apply(Registers &state, const Input input) {
  if (input.kind == Kind::fast) {
    shift(state,
          static_cast<int>(signed24(source(state, input.selector))),
          input.destination_b,
          false);
    return;
  }
  // Index 4E+2U+Z: zero retains Rn, but still limits the result and refreshes flags.
  constexpr std::array actions{0, 0, -1, 0, 1, 1, 1, 1};
  const auto index = ((state.sr & flag_e) != 0 ? 4U : 0U) | ((state.sr & flag_u) != 0 ? 2U : 0U) |
                     ((state.sr & flag_z) != 0 ? 1U : 0U);
  const auto count = actions.at(index);
  if (count == 0) {
    shift(state, 0, input.destination_b, true);
    return;
  }
  shift(state, count, input.destination_b, true);
  auto &exponent = state.r.at(input.selector);
  constexpr Word compatibility = Word{1} << 13;
  const auto address_bits = (state.sr & compatibility) != 0 ? 16U : word_bits;
  const auto range = std::int64_t{1} << address_bits;
  exponent = static_cast<Word>((static_cast<std::int64_t>(exponent) + count + range) % range);
}
} // namespace dsp56300::test::normalization_model
