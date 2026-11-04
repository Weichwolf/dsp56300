// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>

namespace dsp56300::detail {
struct RoundingInput {
  std::uint64_t value;
  Word status;
  bool arithmetic_overflow = false;
};

struct RoundedValue {
  std::uint64_t value;
  bool overflowed;
};

[[nodiscard]] inline RoundedValue round_value(const RoundingInput input) noexcept {
  const unsigned cleared = word_bits + padding_bits + ((input.status & scale_down) != 0 ? 1U : 0U) -
                           ((input.status & scale_up) != 0 ? 1U : 0U);
  const auto half = std::uint64_t{1} << (cleared - 1);
  const auto unit = std::uint64_t{1} << padding_bits;
  const auto bias = (input.status & rounding_mode) != 0
                        ? half
                        : half - unit + (((input.value >> cleared) & 1U) << padding_bits);
  const auto added = wrapped_add(input.value, bias);
  const bool rounding_overflow = ((input.value ^ added) & (bias ^ added)) >> host_sign_bit != 0;
  // A negative accumulation wrap and positive rounding wrap cancel in MACR's
  // widened result. Sticky L observes that final overflow, then any SM limiting.
  return {.value = added & ~((std::uint64_t{1} << cleared) - 1),
          .overflowed = input.arithmetic_overflow != rounding_overflow};
}
} // namespace dsp56300::detail
