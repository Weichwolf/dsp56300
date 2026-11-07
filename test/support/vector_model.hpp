// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_alu_model.hpp"
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "vector_state.hpp"

#include <cstdint>

namespace dsp56300::test::vector_model {
using State = loop_model::State;
using Phase = detail::VectorPhase;
inline constexpr Word base = 0x80, foreground = 0x123456, resume = 0x654321;
inline constexpr Word parallel = 0x200000, short_call = 0x0d0000, effective_call = 0x0bc080;

inline State seed(const Word status, const Phase phase) {
  State result;
  result.reg = register_model::seed(status);
  result.reg.sr = status;
  result.reg.pc = foreground;
  result.reg.la = base;
  result.reg.lc = 1;
  result.vector = {.resume = resume,
                   .address = phase == Phase::second ? base - 1 : base,
                   .level = 3,
                   .phase = phase,
                   .words = {parallel, 0},
                   .loaded = true};
  return result;
}

inline bool begin(State &state, const a64::Instruction input, const bool permitted = true) {
  if (state.vector.phase == Phase::pending) {
    state.reg.pc = state.vector.resume;
    state.completed = false;
    state.vector.phase = Phase::first;
  }
  if (!permitted || (input.words == 2 && state.vector.phase == Phase::second)) {
    loop_model::fail(state, {.pc = input.pc}, FaultKind::unsupported_instruction, input.opcode);
    return false;
  }
  return true;
}

inline void finish(State &state, const a64::Instruction input) {
  if (input.words == 2 || state.vector.phase == Phase::second) {
    state.vector.phase = Phase::none;
    state.reg.pc = state.vector.resume;
    state.inhibited = true;
  } else {
    state.vector.phase = Phase::second;
  }
  state.boundary = detail::NativeBoundary::owner;
  loop_model::retire(state, input.cycles);
}

inline void alu(State &state, const a64::Instruction input) {
  if (!begin(state, input)) { return; }
  if (input.opcode != 0) {
    const auto code = input.opcode & status_byte;
    if (!loop_model::numeric(state, {.pc = input.pc})) { return; }
    if (code == 4 || code == 8 || code == 12) {
      constexpr Word sm = Word{1} << 20;
      if ((state.reg.sr & sm) != 0) {
        loop_model::fail(state, {.pc = input.pc}, FaultKind::unsupported_mode, state.reg.sr);
      } else {
        loop_model::fail(state,
                         {.pc = input.pc},
                         code == 8 ? FaultKind::invalid_instruction
                                   : FaultKind::unsupported_instruction,
                         code);
      }
      return;
    }
    a64_alu_model::apply(state.reg, code);
  }
  finish(state, input);
}

struct Call {
  unsigned mode = 4, index = 0;
  Word target = 768;
  bool effective = false;
};

inline a64::Instruction instruction(const Call form, const Word status) {
  return {.opcode = form.effective ? effective_call | (form.mode << 11) | (form.index << 8)
                                   : short_call | form.target,
          .extension = form.target,
          .pc = base,
          .mode = status,
          .words = form.effective && form.mode == 6 ? 2U : 1U,
          .cycles = form.effective && form.mode >= 5 ? 4U : 3U};
}

inline void call(State &state, const Call form, const a64::Instruction input) {
  if (!begin(state, input)) { return; }
  auto target = form.target;
  if (form.effective) {
    if (form.mode == 6) {
      if (form.index != 0) {
        loop_model::fail(state, {.pc = input.pc}, FaultKind::invalid_instruction, input.opcode);
        return;
      }
    } else {
      target = loop_model::effective(
          state, {.address = {.mode = form.mode, .index = form.index}}, {.pc = input.pc});
      if (state.fault.kind != FaultKind::none) { return; }
    }
  }
  loop_model::push(
      state, {.pc = input.pc}, (std::uint64_t{state.vector.resume} << word_bits) | state.reg.sr);
  if (state.fault.kind != FaultKind::none) { return; }
  constexpr Word cleared = mode_up | mode_down | (Word{1} << 17) | loop_model::loop_flags | 0x300U;
  const auto priority = state.vector.level == 3 ? 3U : state.vector.level + 1;
  state.reg.sr = (state.reg.sr & ~cleared) | (priority * 256);
  state.reg.pc = target;
  state.vector.phase = Phase::none;
  state.boundary = detail::NativeBoundary::owner;
  loop_model::retire(state, input.cycles);
}
}
