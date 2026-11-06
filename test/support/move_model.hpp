// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-field, ring-coordinate and signed quotient/remainder MOVE model.
#pragma once
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace dsp56300::test::move_model {
enum class Kind : std::uint8_t { immediate, long_word, mixed_one, mixed_two };

inline constexpr Word default_address = 384;

struct Form {
  Kind kind = Kind::long_word;
  unsigned reg = 4, selector = 0, mode = 4, index = 0;
  Word extension = default_address;
  bool load = true, y = false, source_b = false, second_high = false, short_form = false;
};

struct Data {
  Word high = 0x923456, low = 0x765432;
};

struct Output {
  Word address = 0, first = 0, second = 0;
  unsigned stores = 0;
  Space space = Space::x;
};

inline constexpr std::uint64_t long_mask = (std::uint64_t{1} << 48) - 1;
inline constexpr unsigned upper_selector_shift = 19, x_register_shift = 18,
                          x_accumulator_shift = 17;
inline constexpr std::array xregs{4U, 5U, 14U, 15U}, yregs{6U, 7U, 14U, 15U};

inline Word opcode(const Form form, const Word alu = 0) {
  if (form.kind == Kind::immediate) {
    return 0x200000U | (form.reg << 16) | (form.extension << 8) | alu;
  }
  if (form.kind == Kind::mixed_two) {
    return 0x080000U | (form.selector << 16) | (form.y ? 0x8000U : 0U) | (form.mode << 11) |
           (form.index << 8) | alu;
  }
  Word result = form.load ? 0x8000U : 0U;
  if (form.kind == Kind::long_word) {
    result |=
        0x400000U | ((form.selector % 4) << 16) | ((form.selector / 4) << upper_selector_shift);
    if (form.short_form) { return result | (form.extension << 8) | alu; }
    result |= 0x4000U;
  } else {
    result |=
        0x100000U | (form.y ? 0x4000U : 0U) | (form.selector << (form.y ? 16U : x_register_shift)) |
        (form.source_b ? Word{1} << (form.y ? upper_selector_shift : x_accumulator_shift) : 0U) |
        (form.second_high ? Word{1} << (form.y ? x_register_shift : 16U) : 0U);
  }
  return result | (form.mode << 11) | (form.index << 8) | alu;
}

inline Word words(const Form form) {
  return form.kind == Kind::immediate || form.short_form || form.mode != 6 ? 1U : 2U;
}

inline Word cycles(const Form form) {
  return form.kind != Kind::immediate && !form.short_form && form.mode >= 5 ? 2U : 1U;
}

inline Word address(Registers &state, const Form form) {
  if (form.short_form || form.mode == 6) { return form.extension; }
  auto &cursor = state.r.at(form.index);
  const auto original = cursor;
  const auto next = address_model::updated({.modifier = state.m.at(form.index),
                                            .cursor = cursor,
                                            .offset = state.n.at(form.index),
                                            .mode = form.mode});
  if (form.mode != 4 && form.mode != 5) { cursor = next; }
  return form.mode == 5 || form.mode == 7 ? next : original;
}

inline std::uint64_t long_bus(Registers &state, const std::uint64_t raw) {
  auto scaled = signed56(raw);
  if ((state.sr & mode_down) != 0) {
    scaled = scaled / 2 - (scaled % 2 < 0 ? 1 : 0);
  } else if ((state.sr & mode_up) != 0) {
    scaled *= 2;
  }
  const unsigned growth =
      45U + ((state.sr & mode_down) != 0 ? 1U : 0U) - ((state.sr & mode_up) != 0 ? 1U : 0U);
  if (((raw >> growth) % 2) != ((raw >> (growth + 1)) % 2)) { state.sr |= flag_s; }
  constexpr auto lower = -(std::int64_t{1} << 47);
  constexpr auto upper = (std::int64_t{1} << 47) - 1;
  if (scaled < lower || scaled > upper) { state.sr |= flag_l; }
  return static_cast<std::uint64_t>(std::clamp(scaled, lower, upper)) & long_mask;
}

inline void load_long(Registers &state, const unsigned selector, const Data data) {
  const auto raw = (std::uint64_t{data.high} << word_bits) | data.low;
  if (selector < 2) {
    auto &accumulator = selector == 0 ? state.a : state.b;
    accumulator = (accumulator / (long_mask + 1)) * (long_mask + 1) + raw;
  } else if (selector < 4) {
    (selector == 2 ? state.x : state.y) = raw;
  } else if (selector < 6) {
    const auto extended = raw + (data.high >= sign24 ? accumulator_mask - long_mask : 0);
    (selector == 4 ? state.a : state.b) = extended;
  } else {
    register_model::write(state, selector == 6 ? 14U : 15U, data.high);
    register_model::write(state, selector == 6 ? 15U : 14U, data.low);
  }
}

inline Data store_long(Registers &state, const unsigned selector) {
  if (selector >= 6) {
    const auto high = register_model::read(state, selector == 6 ? 14U : 15U);
    const auto low = register_model::read(state, selector == 6 ? 15U : 14U);
    return {.high = high, .low = low};
  }
  const std::array banks{state.a, state.b, state.x, state.y};
  auto raw = banks.at(selector < 4 ? selector : selector % 2);
  if (selector >= 4) { raw = long_bus(state, raw); }
  return {.high = static_cast<Word>((raw >> word_bits) & word_mask),
          .low = static_cast<Word>(raw & word_mask)};
}

inline Output mixed(Registers &state, const Form form, const Word location, const Data data) {
  const auto space = form.y ? Space::y : Space::x;
  if (form.kind == Kind::mixed_two) {
    const auto accumulator = 14U + form.selector;
    const auto value = register_model::read(state, accumulator);
    const auto old_input = register_model::read(state, form.y ? 6U : 4U);
    register_model::write(state, accumulator, old_input);
    return {.address = location, .first = value, .stores = 1, .space = space};
  }
  const auto secondary = register_model::read(state, form.source_b ? 15U : 14U);
  const auto primary = form.y ? yregs.at(form.selector) : xregs.at(form.selector);
  Word value = 0;
  if (form.load) {
    value = form.mode == 6 && form.index == 4 ? form.extension : data.high;
    register_model::write(state, primary, value);
  } else {
    value = register_model::read(state, primary);
  }
  register_model::write(state, (form.y ? 4U : 6U) + (form.second_high ? 1U : 0U), secondary);
  return {.address = location, .first = value, .stores = form.load ? 0U : 1U, .space = space};
}

inline Output apply(Registers &state, const Form form, const Data data = {}) {
  if (form.kind == Kind::immediate) {
    const bool fractional = form.reg < 8 || form.reg == 14 || form.reg == 15;
    register_model::write(state, form.reg, form.extension * (fractional ? 65536U : 1U));
    return {};
  }
  const auto location = address(state, form);
  if (form.kind != Kind::long_word) { return mixed(state, form, location, data); }
  if (form.load) {
    load_long(state, form.selector, data);
    return {.address = location};
  }
  const auto value = store_long(state, form.selector);
  return {.address = location, .first = value.high, .second = value.low, .stores = 2};
}
} // namespace dsp56300::test::move_model
