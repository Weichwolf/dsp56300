// SPDX-License-Identifier: GPL-3.0-only
// Independent algebraic word-quantized recurrence; no instruction decoder or core.
#pragma once
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::iir {
inline constexpr std::int32_t unit = std::int32_t{1} << (word_bits - 1);
using Coefficients = std::array<std::int32_t, 5>;
inline constexpr std::array<Coefficients, 4> designs{
    {{unit / 16, unit / 8, unit / 16, -unit, unit / 4},
     {unit / 16, -unit / 8, unit / 16, 3 * (unit / 4), unit / 8},
     {unit / 2, 0, 0, 0, unit / 4},
     {7 * (unit / 8), 7 * (unit / 8), 7 * (unit / 8), -7 * (unit / 8), unit / 2}}};

struct Section {
  Word first = 0, second = 0;
};

struct Quantization {
  Word mode = 0;

  Word operator()(const std::int64_t value) const {
    if (value < -static_cast<std::int64_t>(sign56) || value >= static_cast<std::int64_t>(sign56)) {
      throw std::logic_error("The declared IIR mathematical domain must fit signed56");
    }
    const auto bits =
        word_bits + ((mode & mode_down) != 0 ? 1U : 0U) - ((mode & mode_up) != 0 ? 1U : 0U);
    const auto quantum = std::int64_t{1} << bits;
    auto whole = value / quantum;
    auto remainder = value % quantum;
    if (remainder < 0) {
      --whole;
      remainder += quantum;
    }
    if (remainder > quantum / 2 ||
        (remainder == quantum / 2 && ((mode & mode_rm) != 0 || whole % 2 != 0))) {
      ++whole;
    }
    return static_cast<Word>(std::clamp(whole, -std::int64_t{unit}, std::int64_t{unit - 1})) &
           word_mask;
  }
};

inline Word sample(Section &state, const Word input, const Coefficients &weights, const Word mode) {
  constexpr auto state_unit = std::int64_t{1} << word_bits;
  const auto x = signed24(input);
  const Quantization quantize{.mode = mode};
  const auto y =
      quantize((signed24(state.first) * state_unit) + (std::int64_t{2} * weights[0] * x));
  const auto output = signed24(y);
  const auto first =
      quantize((signed24(state.second) * state_unit) + (std::int64_t{2} * weights[1] * x) -
               (std::int64_t{2} * weights[3] * output));
  const auto second =
      quantize((std::int64_t{2} * weights[2] * x) - (std::int64_t{2} * weights[4] * output));
  state = {.first = first, .second = second};
  return y;
}

enum class Signal : unsigned char { zero, impulse, dc, alternating, ramp, low_bits };

inline Word signal(const Signal kind, const std::size_t index) {
  constexpr unsigned ramp_length = 16;
  constexpr auto ramp_step = std::int64_t{unit} / 8;
  constexpr std::array<std::int32_t, 8> low_bits{0, 1, -1, 3, -3, 7, -7, 1};
  switch (kind) {
    case Signal::zero: return 0;
    case Signal::impulse: return index == 0 ? unit / 2 : 0;
    case Signal::dc: return 3 * (unit / 4);
    case Signal::alternating: return index % 2 == 0 ? unit - 1 : unit;
    case Signal::ramp:
      return static_cast<Word>((static_cast<std::int64_t>(index % ramp_length) - 8) * ramp_step) &
             word_mask;
    case Signal::low_bits: return static_cast<Word>(low_bits[index % low_bits.size()]) & word_mask;
    default: throw std::logic_error("Unknown original IIR signal");
  }
}

struct History {
  unsigned context = 0, section = 0;
};

inline Section seed(const History input) {
  const auto section = input.section;
  switch (input.context) {
    case 0: return {};
    case 1:
      return {.first = (section * 2) + 1,
              .second =
                  static_cast<Word>(-static_cast<std::int64_t>((section * 2) + 3)) & word_mask};
    case 2: {
      constexpr Word first_step = 37;
      constexpr Word second_step = 13;
      return {.first = static_cast<Word>(unit - 1) - (section * first_step),
              .second = unit + (section * second_step)};
    }
    default: throw std::logic_error("Unknown IIR history context");
  }
}
} // namespace dsp56300::test::iir
