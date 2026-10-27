// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "dsp56300/core.hpp"

namespace dsp56300::detail {
struct ConditionInput {
  Word status = 0;
  unsigned selector = 0;
};

// Family manual tables 12-17/18. Bit 3 selects the complementary condition.
[[nodiscard]] constexpr bool condition(const ConditionInput input) noexcept {
  const auto status = input.status;
  const bool signed_ge = ((status & negative) != 0) == ((status & overflow) != 0);
  bool selected = false;
  switch (input.selector & 7) {
    case 0: selected = (status & carry) == 0; break;
    case 1: selected = signed_ge; break;
    case 2: selected = (status & zero) == 0; break;
    case 3: selected = (status & negative) == 0; break;
    case 4: selected = (status & zero) == 0 && (status & (unnormalized | extension)) != 0; break;
    case 5: selected = (status & extension) == 0; break;
    case 6: selected = (status & limit) == 0; break;
    case 7: selected = signed_ge && (status & zero) == 0; break;
    default: return false;
  }
  return selected != ((input.selector & 8) != 0);
}
} // namespace dsp56300::detail
