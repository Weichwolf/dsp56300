// SPDX-License-Identifier: GPL-3.0-only
// Independent per-bit byte logic; no emulator implementation or register helpers.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::byte_model {
enum class Field : std::uint8_t { mr, ccr, com, eom };

struct Input {
  Field field = Field::ccr;
  Word immediate = 0;
  bool inclusive_or = false;
};

inline Word &target(Registers &state, const Field field) {
  return field == Field::mr || field == Field::ccr ? state.sr : state.omr;
}

inline unsigned first_bit(const Field field) {
  return field == Field::mr || field == Field::eom ? 8U : 0U;
}

inline void put(Registers &state, const Field field, const Word byte) {
  auto &word = target(state, field);
  const auto first = first_bit(field);
  Word result = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool selected = bit >= first && bit < first + 8;
    const auto source = selected ? (byte >> (bit - first)) % 2 : (word >> bit) % 2;
    result += source << bit;
  }
  word = result;
}

inline Registers seed(const Field field, const Word byte) {
  constexpr Word old_status = 0x21005a;
  constexpr Word old_omr = 0x5aa55a;
  constexpr Word distant_loop_end = 0x7fffff;
  auto state = register_model::seed(0);
  state.sr = old_status;
  state.omr = old_omr;
  state.la = distant_loop_end;
  put(state, field, byte);
  return state;
}

inline void apply(Registers &state, const Input input) {
  const auto old = target(state, input.field);
  const auto first = first_bit(input.field);
  Word byte = 0;
  for (unsigned bit = 0; bit < 8; ++bit) {
    const bool left = (old >> (first + bit)) % 2 != 0;
    const bool right = (input.immediate >> bit) % 2 != 0;
    const bool next = input.inclusive_or ? left || right : left && right;
    if (next) { byte += Word{1} << bit; }
  }
  put(state, input.field, byte);
}

inline Word opcode(const Input input) {
  constexpr Word and_base = 0xb8;
  constexpr Word or_base = 0xf8;
  return (input.inclusive_or ? or_base : and_base) | (input.immediate << 8) |
         static_cast<Word>(input.field);
}
} // namespace dsp56300::test::byte_model
