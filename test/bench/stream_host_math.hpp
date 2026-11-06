// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstdint>

namespace dsp56300::test::host_math {
// Handwritten arithmetic primitive; the compiler schedules the surrounding bank.
struct Product {
  std::int64_t a, b;
};

inline std::int64_t mac(std::int64_t sum, const Product input) noexcept {
  auto a = input.a;
  const auto b = input.b;
  asm("imulq %[b], %[a]\n\tleaq (%[sum],%[a],2), %[sum]"
      : [sum] "+&r"(sum), [a] "+&r"(a)
      : [b] "r"(b)
      : "cc");
  return sum;
}

inline std::int64_t rounded(const std::int64_t value, const unsigned bits, const bool rm) noexcept {
  const auto whole = value >> bits;
  const auto residue = static_cast<std::uint64_t>(value) & ((std::uint64_t{1} << bits) - 1);
  const auto half = std::uint64_t{1} << (bits - 1);
  return whole +
         static_cast<std::int64_t>(residue > half || (residue == half && (rm || (whole & 1) != 0)));
}

inline Word word(const std::int64_t value) noexcept {
  constexpr auto unit = std::int64_t{1} << (word_bits - 1);
  return static_cast<Word>(std::clamp(value, -unit, unit - 1)) & word_mask;
}
} // namespace dsp56300::test::host_math
