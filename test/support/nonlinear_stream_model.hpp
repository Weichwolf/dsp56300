// SPDX-License-Identifier: GPL-3.0-only
// Numerical recurrence and quantization, independent of instruction execution.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "nonlinear_stream_program.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::nonlinear_stream {
inline constexpr std::int64_t unit = std::int64_t{1} << (word_bits - 1);
inline constexpr std::int64_t accumulator_unit = std::int64_t{1} << word_bits;
inline constexpr std::int64_t saturation_boundary = unit * accumulator_unit;

struct Weights {
  std::int64_t gain, feedback;
};

inline constexpr std::array<Weights, stages> weights{
    {{.gain = 3 * unit / 4, .feedback = 5 * unit / 8},
     {.gain = 7 * unit / 8, .feedback = -unit / 4},
     {.gain = 5 * unit / 8, .feedback = 3 * unit / 4},
     {.gain = 15 * unit / 16, .feedback = -3 * unit / 8}}};

struct Point {
  Word frame = 0, channel = 0;
};

inline Word signal(const Point input) {
  constexpr Word frequency_choices = 7;
  constexpr Word phase_step = 3;
  constexpr Word dither_step = 5;
  const auto frame = input.frame % signal_period;
  const auto phase =
      ((frame * (((input.channel % frequency_choices) * 2) + 1)) + (input.channel * phase_step)) %
      signal_period;
  const auto triangle = phase < signal_period / 2 ? phase : signal_period - phase;
  const auto tone = (static_cast<std::int64_t>(triangle) - (signal_period / 4)) * (unit / 12);
  const auto dither = static_cast<std::int64_t>((frame + input.channel * dither_step) % 7) - 3;
  return static_cast<Word>(tone + dither) & word_mask;
}

inline Word history(const Word channel, const Word stage) {
  constexpr Word period = 17;
  constexpr Word channel_step = 11;
  constexpr Word stage_step = 7;
  const auto value =
      static_cast<std::int64_t>(((channel * channel_step) + stage * stage_step) % period) - 8;
  return static_cast<Word>(value * (unit / 16)) & word_mask;
}

inline unsigned quantum_bits(const Word status) {
  return word_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
}

struct Rounded {
  std::int64_t value;
  bool tie = false;
};

struct RoundInput {
  std::int64_t value;
  Word status;
};

inline Rounded rounded(const RoundInput input) {
  const auto value = input.value;
  const auto status = input.status;
  const auto quantum = std::int64_t{1} << quantum_bits(status);
  auto quotient = value / quantum;
  auto remainder = value % quantum;
  if (remainder < 0) {
    --quotient;
    remainder += quantum;
  }
  const bool tie = remainder == quantum / 2;
  if (remainder > quantum / 2 || (tie && ((status & mode_rm) != 0 || quotient % 2 != 0))) {
    ++quotient;
  }
  return {.value = quotient * quantum, .tie = tie};
}

struct StageInput {
  Word sample = 0, previous = 0, status = 0;
  Weights coefficients;
};

struct Result {
  std::int64_t accumulator = 0;
  Word value = 0, status = 0;
  bool limited = false, transfer_limited = false, tie = false;
};

inline Word flags(const Result result) {
  constexpr Word changed = flag_v | flag_z | flag_n | flag_u | flag_e;
  const auto boundary = unit * (std::int64_t{1} << quantum_bits(result.status));
  const auto raw = static_cast<std::uint64_t>(result.accumulator) & accumulator_mask;
  Word next = result.accumulator == 0 ? flag_z : 0;
  if (result.accumulator < 0) { next |= flag_n; }
  if (result.accumulator < -boundary || result.accumulator >= boundary) { next |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { next |= flag_u; }
  if ((raw / (divisor / 4)) % 2 != (raw / (divisor / 2)) % 2) { next |= flag_s; }
  if (result.limited) { next |= flag_v | flag_l; }
  if (result.transfer_limited) { next |= flag_l; }
  return (result.status & ~changed) | next;
}

inline Result sample(const StageInput input) {
  const auto first = 2 * signed24(input.sample) * input.coefficients.gain;
  const auto sum = first + (2 * signed24(input.previous) * input.coefficients.feedback);
  if (first <= -saturation_boundary || first >= saturation_boundary ||
      sum <= -2 * saturation_boundary || sum >= 2 * saturation_boundary) {
    throw std::logic_error("Nonlinear stream exceeds its proved arithmetic domain");
  }
  const auto rounding = rounded({.value = sum, .status = input.status});
  const bool limited =
      rounding.value < -saturation_boundary || rounding.value >= saturation_boundary;
  auto value = rounding.value;
  if (limited) {
    value = rounding.value < 0 ? -saturation_boundary : saturation_boundary - accumulator_unit;
  }
  const auto quantum = std::int64_t{1} << quantum_bits(input.status);
  auto word = value / quantum;
  if (value % quantum < 0) { --word; }
  const bool transfer_limited = word < -unit || word >= unit;
  word = std::clamp(word, -unit, unit - 1);
  Result result{.accumulator = value,
                .value = static_cast<Word>(word) & word_mask,
                .status = input.status,
                .limited = limited,
                .transfer_limited = transfer_limited,
                .tie = rounding.tie};
  result.status = flags(result);
  return result;
}
} // namespace dsp56300::test::nonlinear_stream
