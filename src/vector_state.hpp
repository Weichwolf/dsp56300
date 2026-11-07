// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail {
enum class VectorPhase : std::uint8_t { none, pending, first, second };

struct VectorState {
  Word resume = 0, address = 0;
  unsigned level = 0;
  VectorPhase phase = VectorPhase::none;
  std::array<Word, 2> words{};
  bool loaded = false;
};

} // namespace dsp56300::detail
