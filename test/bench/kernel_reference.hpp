// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

namespace core_kernels {
inline constexpr std::size_t taps = 64, frames = 1024, blocks = 256, sections = 4;
inline constexpr std::size_t samples = frames * blocks;
inline constexpr std::uint64_t run_cycles = 256, interrupt_cycles = 4096;
inline constexpr std::uint32_t seed = 0x243f6a88, multiplier = 1664525, increment = 1013904223;
inline constexpr std::size_t pulse_period = 97;
inline constexpr unsigned minimum_fraction_bits = 10, maximum_fraction_bits = 31;

template <unsigned Bits>
using Sample = std::conditional_t<(Bits <= 15), std::int16_t, std::int32_t>;
template <unsigned Bits> using Coefficients = std::array<std::array<Sample<Bits>, 5>, sections>;
template <unsigned Bits> using State = std::array<std::array<Sample<Bits>, 2>, sections>;
template <unsigned Bits> inline constexpr std::int64_t unit = std::int64_t{1} << Bits;

// Signed Q1.Bits, nearest-even rounding and saturation after each quantized node.
// These fixture coefficients bound every unquantized intermediate to int64_t.
template <unsigned Bits> Sample<Bits> quantize(const std::int64_t value) noexcept {
  static_assert(Bits >= minimum_fraction_bits && Bits <= maximum_fraction_bits);
  auto whole = value / unit<Bits>;
  auto remainder = value % unit<Bits>;
  if (remainder < 0) {
    --whole;
    remainder += unit<Bits>;
  }
  if (remainder > unit<Bits> / 2 || (remainder == unit<Bits> / 2 && whole % 2 != 0)) { ++whole; }
  return static_cast<Sample<Bits>>(std::clamp(whole, -unit<Bits>, unit<Bits> - 1));
}

template <unsigned Bits> std::vector<Sample<Bits>> signal(const std::size_t count) {
  static_assert(Bits >= minimum_fraction_bits && Bits <= maximum_fraction_bits);
  std::vector<Sample<Bits>> result(count);
  auto random = seed;
  for (std::size_t index = 0; index < count; ++index) {
    random =
        static_cast<std::uint32_t>((std::uint64_t{random} * multiplier + increment) & UINT32_MAX);
    auto value =
        (static_cast<std::int64_t>(random >> (maximum_fraction_bits - Bits)) - unit<Bits>) / 2;
    if (index % pulse_period == 0) { value = unit<Bits> - 1; }
    if (index % pulse_period == 1) { value = -unit<Bits>; }
    result[index] = static_cast<Sample<Bits>>(value);
  }
  return result;
}

template <unsigned Bits> std::array<Sample<Bits>, taps> fir_coefficients() noexcept {
  constexpr std::size_t coefficient_step = 17;
  constexpr std::size_t coefficient_period = 31;
  constexpr std::int64_t coefficient_center = 15;
  constexpr std::int64_t coefficient_divisor = 1024;
  std::array<Sample<Bits>, taps> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto numerator =
        static_cast<std::int64_t>((index * coefficient_step) % coefficient_period) -
        coefficient_center;
    result[index] = static_cast<Sample<Bits>>(numerator * (unit<Bits> / coefficient_divisor));
  }
  return result;
}

template <unsigned Bits> Coefficients<Bits> biquad_coefficients() noexcept {
  const auto u = unit<Bits>;
  const auto q = [](const std::int64_t value) { return static_cast<Sample<Bits>>(value); };
  // b0, b1, b2, a1, a2; denominator 1 + a1*z^-1 + a2*z^-2.
  return {{{q(u / 4), q(u / 8), q(u / 16), q(-u / 2), q(u / 8)},
           {q(u / 2), q(-u / 4), q(u / 8), q(u / 4), q(u / 8)},
           {q(u / 8), q(u / 4), q(u / 8), q(-3 * u / 4), q(u / 4)},
           {q(u / 2), q(u / 4), q(u / 8), q(-u / 2), q(u / 4)}}};
}

template <unsigned Bits> State<Bits> initial_state() noexcept {
  const auto u = unit<Bits>;
  const auto q = [](const std::int64_t value) { return static_cast<Sample<Bits>>(value); };
  return {
      {{q(u - 1), q(-u)}, {q(-u), q(u - 1)}, {q(3 * u / 4), q(-u / 2)}, {q(-3 * u / 4), q(u / 2)}}};
}

// Input includes taps-1 preceding history samples. Coefficients are in causal order.
template <unsigned Bits>
void fir(const std::span<const Sample<Bits>> input,
         const std::span<const Sample<Bits>, taps> coefficients,
         const std::span<Sample<Bits>> output) noexcept {
  assert(input.size() == output.size() + taps - 1);
  for (std::size_t frame = 0; frame < output.size(); ++frame) {
    std::int64_t sum = 0;
    for (std::size_t tap = 0; tap < taps; ++tap) {
      sum += std::int64_t{input[frame + taps - 1 - tap]} * coefficients[tap];
    }
    output[frame] = quantize<Bits>(sum);
  }
}

template <unsigned Bits>
void biquads(const std::span<const Sample<Bits>> input,
             const Coefficients<Bits> &coefficients,
             State<Bits> &state,
             const std::span<Sample<Bits>> output) noexcept {
  assert(input.size() == output.size());
  for (std::size_t frame = 0; frame < input.size(); ++frame) {
    auto value = input[frame];
    for (std::size_t stage = 0; stage < sections; ++stage) {
      const auto &c = coefficients[stage];
      auto &s = state[stage];
      const auto filtered =
          quantize<Bits>((std::int64_t{s[0]} * unit<Bits>)+(std::int64_t{c[0]} * value));
      const auto first =
          quantize<Bits>((std::int64_t{s[1]} * unit<Bits>)+(std::int64_t{c[1]} * value) -
                         (std::int64_t{c[3]} * filtered));
      const auto second =
          quantize<Bits>((std::int64_t{c[2]} * value) - (std::int64_t{c[4]} * filtered));
      s = {first, second};
      value = filtered;
    }
    output[frame] = value;
  }
}
}
