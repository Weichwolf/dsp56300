// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstdint>

namespace dsp56300::detail {
inline constexpr Word inactive_loop = word_mask + 1;

constexpr Word active_loop_address(const Registers &state) noexcept {
  return (state.sr & loop_flag) != 0 ? state.la : inactive_loop;
}

constexpr std::uint64_t translation_key(const Registers &state) noexcept {
  // Seven numeric mode bits, 24 PC bits and 25 active/inactive LA bits fit in 56.
  constexpr unsigned loop_shift = word_bits + 7;
  const auto status = state.sr;
  const Word mode = ((status >> 10) & 3U) | ((status >> 11) & 12U) | ((status >> 13) & 16U) |
                    ((status >> 15) & 96U);
  return state.pc | (std::uint64_t{mode} << word_bits) |
         (std::uint64_t{active_loop_address(state)} << loop_shift);
}
} // namespace dsp56300::detail
