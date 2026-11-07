// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::bit_model {
enum class Location : std::uint8_t { reg, short_address, effective, high, low };
enum class Flow : std::uint8_t { ordinary, jump, relative };

struct Input {
  Location location = Location::reg;
  Flow flow = Flow::ordinary;
  BitAction action = BitAction::test;
  unsigned selector = 4, bit = 0;
  Word extension = 0x234;
  bool call = false, wanted = false, y = false;
};

inline constexpr Word test_action = 0x10020, register_modify = 0xc040, register_jump = 0xc000;
inline constexpr Word relative_register = 0x4080, high_jump = 0x8080;
inline constexpr Word action_prefix = 0x0a0000, relative_prefix = 0x0c8000;
inline constexpr Word low_action = 0x010000, low_jump = 0x018080, low_relative = 0x048000;

inline Word action(const BitAction value) {
  switch (value) {
    case BitAction::clear: return 0;
    case BitAction::set: return 0x20;
    case BitAction::change: return 0x10000;
    case BitAction::test: return test_action;
    default: throw std::logic_error("Unknown authored bit action");
  }
}

inline Word location(const Input input) {
  if (input.flow == Flow::relative) {
    switch (input.location) {
      case Location::reg: return relative_register;
      case Location::short_address: return 0x80;
      case Location::effective: return 0;
      case Location::high: return 0x4000;
      case Location::low: return 0;
      default: throw std::logic_error("Unknown authored bit operand");
    }
  }
  switch (input.location) {
    case Location::reg: return input.flow == Flow::ordinary ? register_modify : register_jump;
    case Location::short_address: return input.flow == Flow::ordinary ? 0 : 0x80;
    case Location::effective: return input.flow == Flow::ordinary ? 0x4000 : relative_register;
    case Location::high: return input.flow == Flow::ordinary ? 0x8000 : high_jump;
    case Location::low: return 0;
    default: throw std::logic_error("Unknown authored bit operand");
  }
}

inline Word variant(const Input input) {
  const bool low = input.location == Location::low;
  if (input.flow == Flow::ordinary) {
    const auto value = action(input.action);
    return low ? (value & 0x20U) | ((value / 65536) * 16384) : value;
  }
  Word call = 0x10000;
  if (low) { call = input.flow == Flow::relative ? 0x80U : 0x4000U; }
  return (input.wanted ? 0x20U : 0U) | (input.call ? call : 0U);
}

inline Word opcode(const Input input) {
  Word prefix = input.flow == Flow::relative ? relative_prefix : action_prefix;
  if (input.location == Location::low) {
    prefix = input.flow == Flow::relative ? low_relative : low_jump;
    if (input.flow == Flow::ordinary) { prefix = low_action; }
  }
  return prefix | variant(input) | location(input) | (input.selector << 8) | input.bit |
         (input.location != Location::reg && input.y ? 0x40U : 0U);
}

inline unsigned words(const Input input) {
  return input.flow != Flow::ordinary ||
                 (input.location == Location::effective && input.selector / 8 == 6)
             ? 2U
             : 1U;
}

inline unsigned cycles(const Input input) {
  unsigned cost = 2;
  if (input.flow != Flow::ordinary) { cost = input.flow == Flow::jump ? 4U : 5U; }
  if (input.location == Location::effective && input.selector / 8 >= 5) { ++cost; }
  return cost;
}

struct Change {
  BitAction action;
  unsigned bit;
};

inline Word changed(const Word before, const Change input) {
  const auto unit = Word{1} << input.bit;
  const bool previous = (before / unit) % 2 != 0;
  if (previous && (input.action == BitAction::clear || input.action == BitAction::change)) {
    return before - unit;
  }
  if (!previous && (input.action == BitAction::set || input.action == BitAction::change)) {
    return before + unit;
  }
  return before;
}

inline Fault failure(const FaultKind kind, const Word value) {
  return {.kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

inline Fault finish(Registers &state, const Input input, const bool previous) {
  if (input.flow == Flow::ordinary) {
    if (input.location != Location::reg || input.selector != register_model::reg_sr ||
        input.action == BitAction::test) {
      state.sr = (state.sr & ~flag_c) | (previous ? flag_c : 0U);
    }
    return {};
  }
  if (previous != input.wanted) { return {}; }
  if (input.call) {
    if (state.sc >= stack_entries - 1) { return failure(FaultKind::stack, state.sp); }
    state.sp = (state.sp + 1) % 64;
    ++state.sc;
    state.stack.at(state.sp % stack_entries) = (std::uint64_t{state.pc} << word_bits) | state.sr;
  }
  state.pc = input.extension;
  if (input.flow == Flow::relative) {
    constexpr auto span = std::int64_t{1} << word_bits;
    state.pc = static_cast<Word>((start + signed24(input.extension) + span) % span);
  }
  return {};
}

inline bool owner(const Input input) {
  return input.flow == Flow::ordinary && input.location == Location::reg &&
         input.action != BitAction::test &&
         (input.selector == register_model::reg_sr || input.selector == register_model::reg_omr ||
          input.selector == register_model::reg_la);
}

inline Fault apply_register(Registers &state, const Input input) {
  state.pc += words(input);
  if (input.bit >= word_bits) { return failure(FaultKind::invalid_instruction, opcode(input)); }
  const auto number = input.selector;
  constexpr Word unsupported = 0x026000;
  if ((number == 14 || number == 15) &&
      ((state.sr & unsupported) != 0 ||
       (state.sr & (mode_up | mode_down)) == (mode_up | mode_down))) {
    return failure(FaultKind::unsupported_mode, state.sr);
  }
  if (!register_model::valid(number)) { return failure(FaultKind::invalid_instruction, number); }
  const auto access = input.flow == Flow::ordinary ? input.action : BitAction::test;
  const bool high = number == register_model::reg_ssh && access != BitAction::test;
  if (number == register_model::reg_ssh && !high && state.sc == 0) {
    return failure(FaultKind::stack, state.sp);
  }
  const auto before = high
                          ? static_cast<Word>(state.stack.at(state.sp % stack_entries) >> word_bits)
                          : register_model::read(state, number);
  if (access != BitAction::test) {
    const auto value = changed(before, {.action = access, .bit = input.bit});
    if (high) {
      auto &frame = state.stack.at(state.sp % stack_entries);
      frame = (std::uint64_t{value} << word_bits) | (frame % (std::uint64_t{1} << word_bits));
    } else {
      register_model::write(state, number, value);
    }
  }
  return finish(state, input, ((before / (Word{1} << input.bit)) % 2) != 0);
}
}
