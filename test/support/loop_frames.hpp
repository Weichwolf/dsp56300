// SPDX-License-Identifier: GPL-3.0-only
// Independent DO frame restoration after a fault-free final instruction.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

namespace dsp56300::test::loop_frames {
inline constexpr Word loop_bits = (1U << 15) | (1U << 16);
inline constexpr Word pointer_mask = 63;

inline bool admitted(const Registers &state) {
  return state.lc > 1 || (state.lc == 1 && state.sc >= 2 && (state.sr & (1U << 16)) == 0);
}

inline void complete(Registers &state) {
  const auto saved = state.stack[state.sp % stack_entries];
  const auto previous = (state.sp + pointer_mask) & pointer_mask;
  const auto frame = state.stack[previous % stack_entries];
  state.sp = (previous + pointer_mask) & pointer_mask;
  state.sc -= 2;
  state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
  state.la = static_cast<Word>(frame >> word_bits) & word_mask;
  state.lc = static_cast<Word>(frame) & word_mask;
}
}
