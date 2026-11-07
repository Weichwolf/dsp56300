// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "selection_model.hpp"
#include "shifted_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::a64_alu_model {
enum Byte : std::uint8_t {
  complement = 0x17,
  logical_right = 0x23,
  logical_left = 0x33,
  rotate_right = 0x27,
  rotate_left = 0x37,
  add_right = 0x02,
  subtract_right = 0x06,
  add_left = 0x12,
  subtract_left = 0x16,
  arithmetic_right = 0x22,
  arithmetic_left = 0x32
};

inline void magnitude_compare(Registers &state, const Registers &old, const Word code) {
  const auto destination = signed56((code & 8U) != 0 ? old.b : old.a);
  const auto source = sequence_model::source(old, code);
  const auto left = destination < 0 ? -destination : destination;
  const auto right = source < 0 ? -source : source;
  const auto difference = left - right;
  const auto limited = saturation_model::apply(
      static_cast<std::uint64_t>(difference) & accumulator_mask, {.status = state.sr});
  sequence_model::flags(state,
                        {.full = signed56(limited.value),
                         .overflow = sequence_model::overflowed(difference) || limited.limited,
                         .carry_changed = true,
                         .carry = left < right});
}

inline void bfu(Registers &state, const Registers &old, const Word code) {
  const auto kind = code & 0xf7U;
  bfu_model::Input input{.destination_b = (code & 8U) != 0};
  if (kind == complement) {
    input.function = bfu_model::Function::complement;
  } else if (kind == logical_right || kind == logical_left || kind == rotate_right ||
             kind == rotate_left) {
    constexpr std::array functions{bfu_model::Function::right,
                                   bfu_model::Function::left,
                                   bfu_model::Function::rotate_right,
                                   bfu_model::Function::rotate_left};
    input.function = functions.at(((code & 16U) != 0 ? 1U : 0U) + ((code & 4U) != 0 ? 2U : 0U));
  } else {
    if ((code & 7U) == 2) {
      input.function = bfu_model::Function::bit_or;
    } else if ((code & 7U) == 3) {
      input.function = bfu_model::Function::bit_xor;
    }
    constexpr std::array indices{0U, 2U, 1U, 3U};
    input.source = indices.at(((code >> 4) & 7U) - 4);
  }
  const auto post_transfer = input.destination_b ? state.b : state.a;
  bfu_model::apply(state, old, input);
  constexpr auto middle = std::uint64_t{word_mask} << word_bits;
  auto &destination = input.destination_b ? state.b : state.a;
  destination = (post_transfer & ~middle) | (destination & middle);
}

inline void apply(Registers &state, const Registers &old, const Word code) {
  const auto kind = code & 0xf7U;
  if (code == selection_model::magnitude_code || code == selection_model::max_code) {
    selection_model::select(state,
                            old,
                            code == selection_model::max_code ? selection_model::Kind::maximum
                                                              : selection_model::Kind::magnitude);
  } else if (kind == add_right || kind == subtract_right || kind == add_left ||
             kind == subtract_left) {
    auto shifted = old;
    shifted.sr = state.sr;
    static_cast<void>(shifted_model::apply(shifted, code));
    ((code & 8U) != 0 ? state.b : state.a) = (code & 8U) != 0 ? shifted.b : shifted.a;
    state.sr = shifted.sr;
  } else if (kind == arithmetic_right || kind == arithmetic_left) {
    const bool destination_b = (code & 8U) != 0;
    shift_model::apply(state,
                       old,
                       {.left = kind == arithmetic_left,
                        .source_b = destination_b,
                        .destination_b = destination_b});
  } else if (code < 0x80U && (kind == complement || kind == logical_right || kind == logical_left ||
                              kind == rotate_right || kind == rotate_left ||
                              (((code >> 4) & 7U) >= 4 &&
                               ((code & 7U) == 2 || (code & 7U) == 3 || (code & 7U) == 6)))) {
    bfu(state, old, code);
  } else if (code < 0x80U && (code & 7U) == 7) {
    magnitude_compare(state, old, code);
  } else {
    saturation_model::alu(state, old, code);
  }
}

inline void apply(Registers &state, const Word code) {
  const auto old = state;
  apply(state, old, code);
}
}
