// SPDX-License-Identifier: GPL-3.0-only
// Independent per-bit stack-frame reconstruction, with raw architectural fields.
#pragma once
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::return_model {
inline constexpr Word rti = 4, rts = 12, pointer_range = 64, foreground = 384;
inline constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, distant_end = 0x700;

struct Frame {
  Word pc = foreground, status = 0, pointer = 1, count = 1;
};

inline Registers seed(const Frame frame) {
  auto state = register_model::seed(0);
  state.sr = frame.status ^ word_mask;
  state.la = distant_end;
  state.sp = frame.pointer;
  state.sc = frame.count;
  state.stack.at(frame.pointer % stack_entries) =
      (std::uint64_t{frame.pc} << word_bits) | frame.status;
  return state;
}

inline void apply(Registers &state, const bool restore_status = true) {
  const auto frame = state.stack.at(state.sp % stack_entries);
  Word pc = 0;
  Word status = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if ((frame >> (word_bits + bit)) % 2 != 0) { pc += Word{1} << bit; }
    if ((frame >> bit) % 2 != 0) { status += Word{1} << bit; }
  }
  state.pc = pc;
  if (restore_status) { state.sr = status; }
  state.sp = (state.sp + pointer_range - 1) % pointer_range;
  --state.sc;
}

struct Entry {
  Word target = foreground;
  unsigned level = 0;
};

inline void interrupt_entry(Registers &state, const Entry input) {
  const auto frame = (std::uint64_t{state.pc} << word_bits) | state.sr;
  state.sp = (state.sp + 1) % pointer_range;
  ++state.sc;
  state.stack.at(state.sp % stack_entries) = frame;
  constexpr Word cleared = mode_up | mode_down | lf | fv | 0x20000U | 0x300U;
  state.sr = (state.sr & ~cleared) | ((input.level + 1) << 8);
  state.pc = input.target;
}
} // namespace dsp56300::test::return_model
