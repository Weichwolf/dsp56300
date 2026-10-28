// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-width saturation decisions and signed arithmetic expectations.
#pragma once
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::saturation_model {
inline constexpr Word mode = Word{1} << 20;
inline constexpr unsigned extension_low = 48, sign_msp = 47;
inline constexpr std::uint64_t positive = 0x007fffffffffff, negative = 0xff800000000000;
inline constexpr std::uint64_t rounded_positive = 0x007fffff000000;

struct Result {
  std::uint64_t value;
  bool limited = false;
};

struct Control {
  Word status;
  bool rounded = false;
};

inline Result apply(const std::uint64_t raw, const Control input) {
  if ((input.status & mode) == 0) { return {.value = raw}; }
  // Raw numerical coordinates and an explicit action table are independent of
  // the implementation's aligned host-bit comparison.
  constexpr std::array<unsigned, 8> action{0, 1, 1, 1, 2, 2, 2, 0};
  const auto decision = (((raw / sign56) % 2) * 4) +
                        (((raw / (std::uint64_t{1} << extension_low)) % 2) * 2) +
                        ((raw / (std::uint64_t{1} << sign_msp)) % 2);
  if (action.at(decision) == 0) { return {.value = raw}; }
  if (action.at(decision) == 2) { return {.value = negative, .limited = true}; }
  return {.value = input.rounded ? rounded_positive : positive, .limited = true};
}

inline void alu(Registers &state, const Registers &old, const Word opcode) {
  const auto code = opcode & status_byte;
  if (code == 0) { return; }
  const auto operation = code & sequence_model::alu_mask;
  if (operation == sequence_model::tfr_other || operation == sequence_model::tfr_x0 ||
      operation == sequence_model::tfr_y0 || operation == sequence_model::tfr_x1 ||
      operation == sequence_model::tfr_y1) {
    sequence_model::alu(state, old, code);
    return;
  }
  const bool destination_b = (code & 8U) != 0;
  auto &destination = destination_b ? state.b : state.a;
  sequence_model::Arithmetic result;
  bool rounded = false;
  if ((code & 0x80U) != 0) {
    result = sequence_model::multiplication(old, code);
    rounded = (code & 1U) != 0;
  } else if (operation == sequence_model::clear) {
    result.full = 0;
  } else if (operation == sequence_model::round || operation == sequence_model::absolute ||
             operation == sequence_model::negate) {
    const auto before = signed56(destination_b ? old.b : old.a);
    result.full = operation == sequence_model::round ? before : -before;
    if (operation == sequence_model::absolute && before >= 0) { result.full = before; }
    rounded = operation == sequence_model::round;
    if (rounded) { sequence_model::rounded(result, old.sr); }
    result.overflow = result.overflow || sequence_model::overflowed(result.full);
  } else {
    result = sequence_model::addition(old, code);
    if (result.compare) { throw std::logic_error("SM compare is outside this qualified domain"); }
  }
  const auto limited = apply(static_cast<std::uint64_t>(result.full) & accumulator_mask,
                             {.status = state.sr, .rounded = rounded});
  destination = limited.value;
  result.full = signed56(limited.value);
  result.overflow = result.overflow || limited.limited;
  sequence_model::flags(state, result);
}

inline void unit(Registers &state, const bool subtract, const bool destination_b) {
  auto &destination = destination_b ? state.b : state.a;
  const auto before = destination;
  sequence_model::Arithmetic result{.full = signed56(before) + (subtract ? -1 : 1),
                                    .carry_changed = true,
                                    .carry = subtract ? before == 0 : before == accumulator_mask};
  result.overflow = sequence_model::overflowed(result.full);
  const auto limited =
      apply(static_cast<std::uint64_t>(result.full) & accumulator_mask, {.status = state.sr});
  destination = limited.value;
  result.full = signed56(limited.value);
  result.overflow = result.overflow || limited.limited;
  sequence_model::flags(state, result);
}
} // namespace dsp56300::test::saturation_model
