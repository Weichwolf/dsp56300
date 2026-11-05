// SPDX-License-Identifier: GPL-3.0-only
// Independent one-bit-at-a-time raw56 shifts and post-shift saturation.
#pragma once
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::shift_model {
struct Control {
  unsigned count = 1;
  bool left = false, source_b = false, destination_b = false;
  bool saturate = true;
};

inline void apply(Registers &state, const Registers &old, const Control input) {
  if (input.count >= accumulator_bits) { throw std::logic_error("Undefined shift count"); }
  const auto raw = input.source_b ? old.b : old.a;
  std::array<bool, accumulator_bits> bits{};
  for (std::size_t i = 0; i < bits.size(); ++i) { bits[i] = ((raw >> i) & 1) != 0; }
  bool carried = false;
  bool overflowed = false;
  for (unsigned step = 0; step < input.count; ++step) {
    const bool sign = bits.back();
    carried = input.left ? bits.back() : bits.front();
    if (input.left) {
      for (std::size_t i = bits.size() - 1; i != 0; --i) { bits[i] = bits[i - 1]; }
      bits.front() = false;
      overflowed = overflowed || sign != bits.back();
    } else {
      for (std::size_t i = 1; i < bits.size(); ++i) { bits[i - 1] = bits[i]; }
      bits.back() = sign;
    }
  }
  std::uint64_t result = 0;
  for (std::size_t i = 0; i < bits.size(); ++i) {
    if (bits[i]) { result |= std::uint64_t{1} << i; }
  }
  const auto status = input.saturate ? state.sr : state.sr & ~saturation_model::mode;
  const auto limited = saturation_model::apply(result, {.status = status});
  (input.destination_b ? state.b : state.a) = limited.value;
  sequence_model::flags(state,
                        {.full = signed56(limited.value),
                         .overflow = overflowed || limited.limited,
                         .carry_changed = true,
                         .carry = carried});
}

inline Word immediate(const Control input) {
  constexpr Word right = 0x0c1c00;
  constexpr Word left = 0x0c1d00;
  return (input.left ? left : right) | (input.source_b ? 0x80U : 0U) | (input.count * 2) |
         (input.destination_b ? 1U : 0U);
}

inline Word parallel(const Control input) {
  constexpr Word left = 0x32;
  constexpr Word right = 0x22;
  return parallel_none | (input.left ? left : right) | (input.destination_b ? 8U : 0U);
}
} // namespace dsp56300::test::shift_model
