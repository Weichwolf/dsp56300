// SPDX-License-Identifier: GPL-3.0-only
// Independent processing-state and guest RESET callback expectations.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "exception_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::processing_model {
enum class Kind : std::uint8_t { reset, wait, stop };
enum class Reply : std::uint8_t { absent, accept, reject };
inline constexpr Word reset = 0x84, wait = 0x86, stop = 0x87, invalid = 7;
inline constexpr Word priority = 0x300;
inline constexpr std::array modes{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Input {
  Kind kind = Kind::reset;
  Reply reply = Reply::accept;
  Word status = 0;
};

inline Word opcode(const Kind kind) {
  if (kind == Kind::reset) { return reset; }
  return kind == Kind::wait ? wait : stop;
}

inline Word cycles(const Kind kind) {
  return kind == Kind::reset ? 7U : 10U;
}

inline ExecutionState state(const Kind kind) {
  if (kind == Kind::reset) { return ExecutionState::running; }
  return kind == Kind::wait ? ExecutionState::waiting : ExecutionState::stopped;
}

inline Registers seed(const Word status) {
  auto result = register_model::seed(0);
  result.sr = status;
  result.vba = 0;
  return result;
}

inline void interrupt_entry(Registers &state, const unsigned level) {
  exception_model::enter(state, state.pc);
  const Word mask = level < 3 ? level + 1 : 3;
  state.sr = (state.sr & ~priority) | (mask * 256);
}
} // namespace dsp56300::test::processing_model
