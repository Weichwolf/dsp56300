// SPDX-License-Identifier: GPL-3.0-only
// Independent software-debug conditions, visible state and per-bit entry frames.
#pragma once
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "exception_model.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::debug_model {
inline constexpr Word debug = 0x200, conditional = 0x300, vector_offset = 6;
inline constexpr unsigned forms = 17;
inline constexpr std::array modes{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Input {
  unsigned form = 0;
  Word status = 0, bank = 0, pointer = 0;
  bool interrupt = false;
};

inline Word opcode(const Input input) {
  return input.form == 0 ? debug : conditional + input.form - 1;
}

inline Word cycles(const Input input) {
  return input.form == 0 ? 1U : 5U;
}

inline bool taken(const Input input, const Word status) {
  return input.form == 0 ||
         selection_model::test_condition({.status = status, .selector = input.form - 1});
}

inline Registers seed(const Input input) {
  auto result = register_model::seed(0);
  result.sr = input.status;
  result.vba = input.bank;
  result.sp = result.sc = input.pointer;
  return result;
}

inline Word vector(const Input input) {
  return ((input.bank / 256) * 256) + vector_offset;
}

inline DebugStatus status(const Input input, const bool requested) {
  return {.interrupt_mode = input.interrupt, .software_occurrence = requested && !input.interrupt};
}

struct Seed {
  unsigned form = 0;
  Word flags = 0;
  bool interrupt = false;
};

inline Input input(const Seed source) {
  const auto flags = source.flags;
  return {.form = source.form,
          .status = flags | modes.at(flags % modes.size()) | ((flags % 4) * 256),
          .bank = (flags % 2) * 512,
          .pointer = static_cast<Word>(flags % (stack_entries - 1)),
          .interrupt = source.interrupt};
}

inline void enter(Registers &state) {
  exception_model::enter(state, state.pc);
}

inline void leave(Registers &state) {
  exception_model::leave(state);
}
} // namespace dsp56300::test::debug_model
