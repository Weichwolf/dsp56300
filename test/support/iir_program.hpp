// SPDX-License-Identifier: GPL-3.0-only
// Original transposed direct-form-II program, with declared instruction effects.
#pragma once
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <vector>

namespace dsp56300::test::iir {
inline constexpr Word state_base = 32, coefficient_base = 16;
inline constexpr unsigned coefficients_per_section = 5;
inline constexpr Word loop_flag = Word{1} << 15;
inline constexpr Word do_prefix = 0x060080;

enum class Action : unsigned char {
  input,
  output,
  state_load,
  state_store,
  coefficient,
  move,
  alu
};

struct Step {
  Word word = 0;
  Action action = Action::alu;
  unsigned reg = 0, slot = 0;
  bool completes_section = false;
};

inline Word single(const unsigned reg) {
  constexpr unsigned high_shift = 17;
  constexpr Word high_mask = 0x18;
  constexpr Word prefix = 0x400000;
  return prefix | ((reg & 7U) << 16) | ((reg & high_mask) << high_shift);
}

inline Step state_load(const unsigned slot, const unsigned reg) {
  constexpr Word load = 0x8000;
  return {.word = single(reg) | load | ((state_base + slot) << 8),
          .action = Action::state_load,
          .reg = reg,
          .slot = slot};
}

inline Step state_store(const unsigned slot) {
  return {
      .word = single(14) | ((state_base + slot) << 8), .action = Action::state_store, .slot = slot};
}

inline Step coefficient(const unsigned slot) {
  constexpr Word load_y = 0x088000;
  return {.word = single(6) | load_y | ((coefficient_base + slot) << 8),
          .action = Action::coefficient,
          .slot = slot};
}

inline Step move(const unsigned source, const unsigned destination) {
  return {.word = parallel_none | (source << 13) | (destination << 8),
          .action = Action::move,
          .reg = destination,
          .slot = source};
}

inline Step alu(const Word code) {
  return {.word = parallel_none | code};
}

struct Shape {
  unsigned sections = 1;
  bool scheduled = false;
};

struct SectionPlan {
  unsigned number = 0;
  bool scheduled = false;
};

inline Step parallel(Step transfer, const Word code) {
  transfer.word |= code;
  return transfer;
}

inline std::vector<Step> section(const SectionPlan input) {
  constexpr Word tfr_x1 = 0x61;
  constexpr Word tfr_y1 = 0x71;
  constexpr Word mac_x0 = 0xd2;
  constexpr Word subtract_x1 = 0xe6;
  constexpr Word round = 0x11;
  constexpr Word clear = 0x13;
  const auto state = input.number * 2;
  const auto weights = input.number * coefficients_per_section;
  auto output = move(5, 4);
  output.completes_section = true;
  if (input.scheduled) {
    return {state_load(state, 5),
            state_load(state + 1, 7),
            parallel(coefficient(weights), tfr_x1),
            parallel(coefficient(weights + 1), mac_x0),
            alu(round),
            parallel(move(14, 5), tfr_y1),
            parallel(coefficient(weights + 3), mac_x0),
            parallel(coefficient(weights + 2), subtract_x1),
            alu(round),
            parallel(state_store(state), clear),
            parallel(coefficient(weights + 4), mac_x0),
            alu(subtract_x1),
            alu(round),
            state_store(state + 1),
            output};
  }
  return {state_load(state, 5),
          state_load(state + 1, 7),
          coefficient(weights),
          alu(tfr_x1),
          alu(mac_x0),
          alu(round),
          move(14, 5),
          coefficient(weights + 1),
          alu(tfr_y1),
          alu(mac_x0),
          coefficient(weights + 3),
          alu(subtract_x1),
          alu(round),
          state_store(state),
          coefficient(weights + 2),
          alu(clear),
          alu(mac_x0),
          coefficient(weights + 4),
          alu(subtract_x1),
          alu(round),
          state_store(state + 1),
          output};
}

inline std::vector<Step> body(const Shape input) {
  constexpr Word load_input = 0x44d800;
  constexpr Word store_output = 0x445900;
  std::vector<Step> result{{.word = load_input, .action = Action::input}};
  for (unsigned index = 0; index < input.sections; ++index) {
    const auto steps = section({.number = index, .scheduled = input.scheduled});
    result.insert(result.end(), steps.begin(), steps.end());
  }
  result.push_back({.word = store_output, .action = Action::output});
  return result;
}

inline std::vector<Word> program(const Word samples, const std::vector<Step> &steps) {
  const auto last = start + 1 + static_cast<Word>(steps.size());
  std::vector<Word> words{do_prefix | ((samples & status_byte) << 8) | (samples >> 8), last};
  for (const auto &step : steps) { words.push_back(step.word); }
  return words;
}
} // namespace dsp56300::test::iir
