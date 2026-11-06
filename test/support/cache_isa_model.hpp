// SPDX-License-Identifier: GPL-3.0-only
// Independent cache-control encoding, addressing and base-cost expectations.
#pragma once
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::cache_isa_model {
inline constexpr Word enable = Word{1} << 19;
enum class Kind : std::uint8_t {
  flush,
  flush_unlocked,
  free,
  lock,
  unlock,
  lock_relative,
  unlock_relative
};

struct Form {
  Kind kind = Kind::flush;
  unsigned mode = 0, index = 0;
  Word extra = 0;
};

inline bool effective(const Form form) {
  return form.kind == Kind::lock || form.kind == Kind::unlock;
}

inline bool relative(const Form form) {
  return form.kind == Kind::lock_relative || form.kind == Kind::unlock_relative;
}

inline Word words(const Form form) {
  return relative(form) || (effective(form) && form.mode == 6) ? 2U : 1U;
}

inline Word opcode(const Form form) {
  if (form.kind == Kind::flush) { return 3; }
  if (form.kind == Kind::flush_unlocked) { return 1; }
  if (form.kind == Kind::free) { return 2; }
  if (relative(form)) { return form.kind == Kind::lock_relative ? 15U : 14U; }
  const Word prefix = form.kind == Kind::lock ? 0x0bc081U : 0x0ac081U;
  return prefix + (form.mode * 2048) + (form.index * 256);
}

inline Word cycles(const Form form) {
  if (relative(form)) { return 4; }
  return effective(form) ? 2U + (form.mode >= 5 ? 1U : 0U) : 1U;
}

inline void apply(Registers &state, const Form form) {
  state.pc += words(form);
  if ((state.sr & enable) == 0 || !effective(form) || form.mode == 6) { return; }
  const auto next = address_model::updated({.modifier = state.m[form.index],
                                            .cursor = state.r[form.index],
                                            .offset = state.n[form.index],
                                            .mode = form.mode});
  if (form.mode != 5) { state.r[form.index] = next; }
}

inline Registers seed(const Word flags, const bool enabled) {
  constexpr Word ignored_modifier = 0x550000;
  constexpr std::array modes{
      0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
  constexpr std::array cursors{word_mask, 0x800000U, 0xaaaaaaU, 268U, 0x103U, 0U, 0x123456U};
  constexpr std::array offsets{3U, 0x800000U, 0x555555U, 3U, 0x800000U, word_mask, 0x7fffffU};
  constexpr std::array modifiers{0xffffU, 0U, 0U, 12U, 0x8007U, 0xffffU, 0x8001U};
  auto state = register_model::seed(0);
  state.sr = flags | modes.at(flags % modes.size()) | ((flags % 4) * 256) | (enabled ? enable : 0U);
  state.vba = (flags % 2) * 512;
  state.sp = state.sc = flags % (stack_entries - 1);
  const auto scenario = flags % cursors.size();
  state.r.fill(cursors.at(scenario));
  state.n.fill(offsets.at(scenario));
  state.m.fill(modifiers.at(scenario) + ignored_modifier);
  return state;
}
} // namespace dsp56300::test::cache_isa_model
