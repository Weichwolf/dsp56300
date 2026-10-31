// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::test::unit_publication_test {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
inline constexpr auto policy = InstructionPublication::unit_updates;
inline constexpr unsigned control_base = 32;
inline constexpr unsigned stack_pointer_mask = 63;
inline constexpr unsigned stack_counter_mask = 31;
inline constexpr std::uint64_t middle = std::uint64_t{word_mask} << word_bits;
inline constexpr std::uint64_t upper = accumulator_mask ^ long_mask;

inline RegisterUpdate field(const RegisterStorage storage,
                            const std::uint64_t mask,
                            const std::uint64_t before,
                            const std::uint64_t after,
                            const std::size_t index = 0) {
  return {.storage = storage,
          .index = static_cast<std::uint8_t>(index),
          .mask = mask,
          .previous = before & mask,
          .value = after & mask};
}

inline Registers seed() {
  auto state = register_model::seed(0);
  state.pc = boot;
  state.sr = 0;
  return state;
}

inline void held(Core &core, const Registers &state) {
  require(core.registers() == state && core.progress() == Progress{},
          "Native unit publication preserves every unrelated register and retirement");
}

inline void publish(Core &core, const UnitWriteback &expected, const Registers &state) {
  auto action = pending(core);
  require(action.effect.kind == EffectKind::unit_update && action.effect.unit_update == expected,
          "Native unit/opcode/first-written masks and values match independent expectations");
  action.effect.unit_update = UnitWriteback{};
  action.computed = {};
  require(!core.advance_instruction(action.ticket).retired,
          "Only the stored owned native result authorizes a unit publication");
  held(core, state);
  rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
}

RegisterUpdate destination(const Registers &before, const Registers &after, unsigned number);
void bits();
void controls();
void calculations();
void boundaries();
void ram_boundaries();
} // namespace dsp56300::test::unit_publication_test
