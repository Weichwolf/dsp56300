// SPDX-License-Identifier: GPL-3.0-only
// Reusable independent integer FFT recurrence; no emulator execution or decoder.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fft_program.hpp"
#include "fft_twiddles.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::test::fft {
using Sample = std::array<std::int64_t, 2>;
inline constexpr std::int64_t half_scale = std::int64_t{1} << 23;
inline constexpr std::int64_t accumulator_unit = std::int64_t{1} << word_bits;
inline constexpr unsigned low_growth_bit = 45, high_growth_bit = 46, high_fraction_bit = 47;

struct Division {
  std::int64_t numerator = 0, divisor = 2;
  bool rm = false;
};

inline std::int64_t nearest(const Division input) {
  auto quotient = input.numerator / input.divisor;
  auto remainder = input.numerator % input.divisor;
  if (remainder < 0) {
    --quotient;
    remainder += input.divisor;
  }
  if (remainder > input.divisor / 2 ||
      (remainder == input.divisor / 2 && (input.rm || quotient % 2 != 0))) {
    ++quotient;
  }
  return quotient;
}

inline Sample coefficient(const Word index) {
  const auto phase = index % maximum_size;
  const auto position = 2 * (phase % (maximum_size / 2));
  const auto sign = phase >= maximum_size / 2 ? -1 : 1;
  return {sign * signed24(twiddles.at(position)), sign * signed24(twiddles.at(position + 1))};
}

struct Signal {
  Word size = 0;
  unsigned kind = 0;
};

inline std::vector<Sample> input_signal(const Signal input) {
  const auto size = input.size;
  const auto signal = input.kind;
  std::vector<Sample> result(size);
  constexpr unsigned tone = 3;
  constexpr unsigned real_multiplier = 37;
  constexpr unsigned imaginary_multiplier = 53;
  constexpr std::int64_t pattern_period = 257;
  constexpr std::int64_t pattern_center = 128;
  constexpr std::int64_t pattern_unit = 1024;
  for (Word index = 0; index < size; ++index) {
    switch (signal) {
      case 0: break;
      case 1: result[index] = {index == 0 ? half_scale / 4 : 0, 0}; break;
      case 2: result[index] = {half_scale / 32, -half_scale / 64}; break;
      case 3: {
        const auto phase = coefficient((index * tone * maximum_size / size) % maximum_size);
        result[index] = {phase[0] / 16, -phase[1] / 16};
        break;
      }
      case 4: {
        const auto value = index % 2 == 0 ? half_scale - 1 : -half_scale;
        result[index] = {value, -value - 1};
        break;
      }
      case 5: {
        constexpr std::array<std::int64_t, 4> corners{
            half_scale - 1, -half_scale, half_scale / 2, -half_scale / 2};
        result[index] = {corners[index % corners.size()], corners[(index / 3) % corners.size()]};
        break;
      }
      case 6:
        result[index] = {
            ((static_cast<std::int64_t>(index) * real_multiplier) % pattern_period -
             pattern_center) *
                pattern_unit,
            (((static_cast<std::int64_t>(index) * imaginary_multiplier) + (index / 3)) %
                 pattern_period -
             pattern_center) *
                pattern_unit};
        break;
      default: throw std::invalid_argument("Invalid FFT signal");
    }
  }
  return result;
}

inline Word word(const std::int64_t value) {
  return static_cast<Word>(value) & word_mask;
}

inline std::uint64_t raw_accumulator(const std::int64_t value) {
  return static_cast<std::uint64_t>(value * accumulator_unit) & accumulator_mask;
}

inline Word result_flags(const std::int64_t value) {
  const auto raw = raw_accumulator(value);
  Word flags = value == 0 ? flag_z : 0;
  if (value < 0) { flags |= flag_n; }
  if (value < -half_scale || value >= half_scale) { flags |= flag_e; }
  if ((raw >> high_fraction_bit) % 2 == (raw >> (high_fraction_bit - 1)) % 2) { flags |= flag_u; }
  return flags;
}

struct Model {
  std::vector<Sample> data;
  Word status = 0, last_inner_status = 0;
  std::int64_t a = 0, b = 0;
  std::uint64_t clips = 0, ties = 0;
  bool rm = false;

  std::int64_t round(const std::int64_t value, const std::int64_t divisor) {
    if (value % divisor == divisor / 2 || value % divisor == -(divisor / 2)) { ++ties; }
    return nearest({.numerator = value, .divisor = divisor, .rm = rm});
  }

  std::int64_t transfer(const std::int64_t value) {
    const auto raw = raw_accumulator(value);
    if ((raw >> low_growth_bit) % 2 != (raw >> high_growth_bit) % 2) { status |= flag_s; }
    if (value < -half_scale || value >= half_scale) {
      status |= flag_l;
      ++clips;
    }
    return std::clamp(value, -half_scale, half_scale - 1);
  }
};

struct Butterfly {
  Word left = 0, right = 0, twiddle = 0;
};

inline void butterfly_model(Model &model, const Butterfly input) {
  const auto left = model.data[input.left];
  const auto right = model.data[input.right];
  const auto factor = coefficient(input.twiddle);
  Sample difference{};
  for (std::size_t part = 0; part < 2; ++part) {
    const auto sum = model.round(left[part] + right[part], 2);
    const auto delta = model.round(left[part] - right[part], 2);
    model.data[input.left][part] = model.transfer(sum);
    difference[part] = model.transfer(delta);
  }
  model.a = model.round((difference[0] * factor[0]) - (difference[1] * factor[1]), half_scale);
  model.b = model.round((difference[0] * factor[1]) + (difference[1] * factor[0]), half_scale);
  model.data[input.right] = {model.transfer(model.a), model.transfer(model.b)};
  model.status = (model.status & (mode_rm | flag_s | flag_l)) | result_flags(model.b);
}

inline Word reversed(const Word value, const Word size) {
  Word result = 0;
  for (Word remaining = size; remaining > 1; remaining /= 2) {
    result = 2 * result + ((value / (size / remaining)) % 2);
  }
  return result;
}

inline Model transform(std::vector<Sample> input, const Word status) {
  const auto size = static_cast<Word>(input.size());
  Model model{.data = std::move(input), .status = status, .rm = (status & mode_rm) != 0};
  for (Word length = size; length > 1; length /= 2) {
    for (Word base = 0; base < size; base += length) {
      model.last_inner_status = model.status | loop_flag;
      for (Word index = 0; index < length / 2; ++index) {
        butterfly_model(model,
                        {.left = base + index,
                         .right = base + index + (length / 2),
                         .twiddle = index * maximum_size / length});
      }
    }
  }
  return model;
}

} // namespace dsp56300::test::fft
