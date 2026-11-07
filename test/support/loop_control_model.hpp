// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_loop_state.hpp"
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <bit>
#include <cstdint>

namespace dsp56300::test::loop_model {
inline constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, cm = Word{1} << 13;
inline constexpr Word loop_flags = lf | fv, pointer_range = 64, zero_repeat = 65536;
inline constexpr Word do_immediate = 0x060080, do_register = 0x06c000, do_short = 0x060000,
                      do_effective = 0x064000;
inline constexpr Word rep_immediate = 0x0600a0, rep_register = 0x06c020, rep_short = 0x060020,
                      rep_effective = 0x064020;
inline constexpr Word do_forever = 0x000203, dor_forever = 0x000202, enddo = 0x00008c,
                      break_base = 0x000210;
enum class Kind : std::uint8_t { absolute, relative, repeat, forever_absolute, forever_relative };
enum class Operand : std::uint8_t { immediate, reg, short_memory, effective };

struct Input {
  Kind kind = Kind::absolute;
  Operand operand = Operand::immediate;
  Word source = 3, extension = start + 8, memory_value = 3;
  address_model::Input address{
      .modifier = address_model::linear, .cursor = register_model::cursor, .offset = 1, .mode = 4};
  bool y = false;
};

inline bool forever(const Input input) {
  return input.kind == Kind::forever_absolute || input.kind == Kind::forever_relative;
}

inline bool relative(const Input input) {
  return input.kind == Kind::relative || input.kind == Kind::forever_relative;
}

inline Word opcode(const Input input) {
  if (forever(input)) { return relative(input) ? dor_forever : do_forever; }
  const bool repeat = input.kind == Kind::repeat;
  Word base = repeat ? rep_immediate : do_immediate;
  Word source = ((input.source % 256) << 8) | (input.source / 256);
  if (input.operand == Operand::reg) {
    base = repeat ? rep_register : do_register;
    source = input.source << 8;
  } else if (input.operand == Operand::short_memory) {
    base = repeat ? rep_short : do_short;
    source = (input.source << 8) | (input.y ? 0x40U : 0U);
  } else if (input.operand == Operand::effective) {
    base = repeat ? rep_effective : do_effective;
    source = (input.address.mode << 11) | (input.address.index << 8) | (input.y ? 0x40U : 0U);
  }
  return base | source | (relative(input) ? 16U : 0U);
}

inline unsigned words(const Input input) {
  return input.kind == Kind::repeat ? 1U : 2U;
}

inline unsigned cycles(const Input input) {
  if (forever(input)) { return 4; }
  return 5 + (input.operand == Operand::effective && input.address.mode >= 5 ? 1U : 0U);
}

struct Site {
  Word pc = start;
  unsigned words = 1;
};

inline void fail(State &state, const Site site, const FaultKind kind, const Word value) {
  if (state.fault.kind == FaultKind::none) {
    state.fault = {
        .kind = kind, .space = Space::program, .address = site.pc, .value = value, .pc = site.pc};
  }
  state.boundary = detail::NativeBoundary::owner;
}

inline void context(State &state) {
  if (state.boundary != detail::NativeBoundary::owner) {
    state.boundary = detail::NativeBoundary::context;
  }
}

inline void push(State &state, const Site site, const std::uint64_t frame) {
  if (state.reg.sc >= stack_entries - 1) {
    fail(state, site, FaultKind::stack, state.reg.sp);
    return;
  }
  state.reg.sp = (state.reg.sp + 1) % pointer_range;
  ++state.reg.sc;
  state.reg.stack.at(state.reg.sp % stack_entries) = frame;
}

inline std::uint64_t pop(State &state, const Site site) {
  if (state.reg.sc == 0) {
    fail(state, site, FaultKind::stack, state.reg.sp);
    return 0;
  }
  const auto frame = state.reg.stack.at(state.reg.sp % stack_entries);
  state.reg.sp = (state.reg.sp + pointer_range - 1) % pointer_range;
  --state.reg.sc;
  return frame;
}

inline void restore(State &state, const Site site) {
  const auto saved = pop(state, site);
  const auto outer = pop(state, site);
  state.reg.sr = (state.reg.sr & ~loop_flags) | (static_cast<Word>(saved) & loop_flags);
  state.reg.la = static_cast<Word>(outer >> word_bits);
  state.reg.lc = static_cast<Word>(outer) & word_mask;
  if (state.fault.kind == FaultKind::none) { context(state); }
}

inline void finish(State &state, const Site site) {
  const auto last = (site.pc + site.words - 1) % address_words;
  if ((state.reg.sr & lf) == 0 || last != state.reg.la ||
      state.reg.pc != (state.reg.la + 1) % address_words || state.repeat.remaining != 0) {
    return;
  }
  if ((state.reg.sr & cm) != 0) {
    fail(state, site, FaultKind::unsupported_mode, state.reg.sr);
    return;
  }
  if (state.reg.lc != 1 || (state.reg.sr & fv) != 0) {
    state.reg.lc = (state.reg.lc + address_words - 1) % address_words;
    state.reg.pc = static_cast<Word>(state.reg.stack.at(state.reg.sp % stack_entries) >> word_bits);
    return;
  }
  restore(state, site);
}

inline bool numeric(State &state, const Site site) {
  constexpr Word unsupported = 0x026000;
  if ((state.reg.sr & unsupported) == 0 &&
      (state.reg.sr & (mode_up | mode_down)) != (mode_up | mode_down)) {
    return true;
  }
  fail(state, site, FaultKind::unsupported_mode, state.reg.sr);
  return false;
}

inline Word effective(State &state, const Input input, const Site site) {
  const auto form = input.address;
  if (form.mode == 6) {
    fail(state, site, FaultKind::invalid_instruction, opcode(input));
    return 0;
  }
  if (form.mode == 4) { return state.reg.r.at(form.index); }
  const auto modifier = state.reg.m.at(form.index) % 65536;
  if (modifier >= 32768 && modifier != address_model::linear) {
    const auto length = (modifier % 16384) + 1;
    constexpr Word reserved = 49152;
    if (modifier >= reserved || length == 1 || !std::has_single_bit(length)) {
      fail(state, site, FaultKind::unsupported_mode, modifier);
      return 0;
    }
  }
  auto address = form;
  address.cursor = state.reg.r.at(form.index);
  address.offset = state.reg.n.at(form.index);
  address.modifier = state.reg.m.at(form.index);
  const auto next = address_model::updated(address);
  if (form.mode != 5) { state.reg.r.at(form.index) = next; }
  return form.mode == 5 || form.mode == 7 ? next : address.cursor;
}

inline Word count(State &state, const Input input, const Site site) {
  if (input.operand == Operand::immediate) { return input.source; }
  if (!numeric(state, site)) { return 0; }
  if (input.operand == Operand::reg) {
    if (input.kind != Kind::repeat && input.source == register_model::reg_ssh) {
      fail(state, site, FaultKind::invalid_instruction, opcode(input));
      return 0;
    }
    if (!register_model::valid(input.source)) {
      fail(state, site, FaultKind::invalid_instruction, input.source);
      return 0;
    }
    if (input.source == register_model::reg_ssh && state.reg.sc == 0) {
      fail(state, site, FaultKind::stack, state.reg.sp);
      return 0;
    }
    return register_model::read(state.reg, input.source);
  }
  const auto address =
      input.operand == Operand::effective ? effective(state, input, site) : input.source;
  if (state.fault.kind != FaultKind::none) { return 0; }
  if (address >= memory_words) {
    state.fault = {.kind = FaultKind::memory_read,
                   .space = input.y ? Space::y : Space::x,
                   .address = address,
                   .pc = site.pc};
    state.boundary = detail::NativeBoundary::owner;
    return 0;
  }
  return input.memory_value & word_mask;
}

inline Word last(const Input input, const Site site) {
  constexpr auto range = std::int64_t{1} << word_bits;
  return relative(input) ? static_cast<Word>((site.pc + signed24(input.extension) + range) % range)
                         : input.extension;
}

inline void setup(State &state, const Input input, const Site site = {.pc = start, .words = 2}) {
  state.reg.pc = (site.pc + words(input)) % address_words;
  state.inhibited = false;
  const bool repeat = input.kind == Kind::repeat;
  if (!repeat && (state.reg.sr & cm) != 0) {
    fail(state, site, FaultKind::unsupported_mode, state.reg.sr);
    return;
  }
  const auto saved_lc = state.reg.lc;
  if (!repeat) { push(state, site, (std::uint64_t{state.reg.la} << word_bits) | state.reg.lc); }
  if (state.fault.kind != FaultKind::none) { return; }
  const auto loaded = forever(input) ? state.reg.lc : count(state, input, site);
  if (state.fault.kind != FaultKind::none) { return; }
  if (repeat) {
    state.repeat.remaining = loaded == 0 ? zero_repeat : loaded;
    state.repeat.pc = state.reg.pc;
    state.repeat.lc = saved_lc;
    state.repeat.loaded = false;
    state.reg.lc = loaded;
    state.boundary = detail::NativeBoundary::owner;
  } else if (!forever(input) && loaded == 0) {
    static_cast<void>(pop(state, site));
    state.reg.pc = (last(input, site) + 1) % address_words;
  } else {
    state.reg.lc = loaded;
    push(state, site, (std::uint64_t{state.reg.pc} << word_bits) | state.reg.sr);
    if (state.fault.kind != FaultKind::none) { return; }
    state.reg.la = last(input, site);
    state.reg.sr = (state.reg.sr & ~fv) | lf | (forever(input) ? fv : 0U);
    context(state);
  }
  if (!repeat) { state.completed = false; }
  finish(state, {.pc = site.pc, .words = words(input)});
  if (state.fault.kind == FaultKind::none) { retire(state, cycles(input)); }
}

inline void end(State &state, const Site site, const int condition = -1) {
  state.reg.pc = (site.pc + 1) % address_words;
  state.inhibited = false;
  if (condition < 0 ||
      selection_model::test_condition(
          {.status = state.reg.sr, .selector = static_cast<unsigned>(condition)})) {
    if (condition >= 0) { state.reg.pc = (state.reg.la + 1) % address_words; }
    restore(state, site);
  }
  if (state.fault.kind != FaultKind::none) { return; }
  state.completed = false;
  finish(state, site);
  if (state.fault.kind == FaultKind::none) { retire(state, condition < 0 ? 1U : 5U); }
}
}
