// SPDX-License-Identifier: GPL-3.0-only
// Independent software-exception conditions, bit-built frames and vector coordinates.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "return_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::exception_model {
enum class Kind : std::uint8_t { illegal, trap, conditional };

struct Input {
  Kind kind = Kind::trap;
  unsigned condition = 0;
  Word status = 0, vba = 0, pointer = 0;
};

inline constexpr Word service = 768, call = 0x0d0000, load_immediate_x0 = 0x44f400;
inline constexpr Word priority_bits = 0x300;
inline constexpr Word datum = 0x923456, invalid_canary = 7;
inline constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, sa = Word{1} << 17;

inline Word opcode(const Input input) {
  if (input.kind == Kind::illegal) { return 5; }
  return input.kind == Kind::trap ? 6U : 16U + input.condition;
}

inline Word cycles(const Input input) {
  return input.kind == Kind::illegal ? 5U : 9U;
}

inline bool requested(const Input input, const Word status) {
  return input.kind != Kind::conditional ||
         selection_model::test_condition({.status = status, .selector = input.condition});
}

inline Word resume(const Input input, const Word pc) {
  return (pc + (input.kind == Kind::illegal ? 2U : 1U)) % (word_mask + 1);
}

inline Word vector(const Input input) {
  return ((input.vba / 256) * 256) + (input.kind == Kind::illegal ? 4U : 8U);
}

inline Registers seed(const Input input) {
  auto state = register_model::seed(0);
  state.sr = input.status;
  state.vba = input.vba;
  state.sp = input.pointer;
  state.sc = input.pointer;
  return state;
}

inline void enter(Registers &state, const Word return_pc) {
  std::uint64_t frame = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if ((return_pc >> bit) % 2 != 0) { frame += std::uint64_t{1} << (bit + word_bits); }
    if ((state.sr >> bit) % 2 != 0) { frame += std::uint64_t{1} << bit; }
  }
  ++state.sp;
  ++state.sc;
  state.stack.at(state.sp % stack_entries) = frame;
  constexpr Word cleared = mode_up | mode_down | lf | fv | sa | priority_bits;
  state.sr = (state.sr & ~cleared) | priority_bits;
  state.pc = service;
}

inline void leave(Registers &state) {
  return_model::apply(state);
}
} // namespace dsp56300::test::exception_model
