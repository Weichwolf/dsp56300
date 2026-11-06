// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::shifted_model {
// FM13-9/10/174/175: signed mathematical shift, then add/subtract the
// other accumulator. Division with an explicit remainder gives floor(D/2).
// Table3-1 limits the wrapped result; it does not replace full-width carry.
inline bool apply(Registers &state, const Word opcode) {
  const bool destination_b = (opcode & 8U) != 0;
  const bool left = (opcode & 16U) != 0;
  const bool subtract = (opcode & 4U) != 0;
  auto &destination = destination_b ? state.b : state.a;
  const auto source = destination_b ? state.a : state.b;
  const auto original = signed56(destination);
  auto shifted = original / 2;
  if (original < 0 && original % 2 != 0) { --shifted; }
  if (left) { shifted = original * 2; }
  const auto full = subtract ? shifted - signed56(source) : shifted + signed56(source);
  const auto raw_shifted = static_cast<std::uint64_t>(shifted) & accumulator_mask;
  const bool carry = subtract ? raw_shifted < source : raw_shifted + source > accumulator_mask;
  const bool shift_overflow = sequence_model::overflowed(shifted);
  const auto limited = saturation_model::apply(static_cast<std::uint64_t>(full) & accumulator_mask,
                                               {.status = state.sr});
  destination = limited.value;
  sequence_model::flags(
      state,
      {.full = signed56(limited.value),
       .overflow = shift_overflow || sequence_model::overflowed(full) || limited.limited,
       .carry_changed = true,
       .carry = carry});
  return !shift_overflow;
}
} // namespace dsp56300::test::shifted_model
