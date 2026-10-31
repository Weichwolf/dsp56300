// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::move_publication_test {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
inline constexpr auto phased = InstructionPublication::move_updates;
inline constexpr Word payload = 0x8642ac, data_address = 0x100;
inline constexpr unsigned register_count = 64, control_base = 32;
inline constexpr unsigned ssh = 60, ssl = 61, stack_pointer_mask = 63, stack_counter_mask = 31;
inline constexpr std::uint64_t main_mask = 0x00ffffff000000, high_mask = 0xff000000000000;

inline Registers seed() {
  auto result = register_model::seed(0);
  result.pc = boot;
  result.sr = 0;
  result.y = (result.y & main_mask) | payload;
  result.r[0] = data_address;
  return result;
}

inline void held(Core &core, const Registers &expected) {
  require(core.registers() == expected, "Native publication preserves every unrelated register");
  require(core.progress() == Progress{}, "MOVE publication never retires or advances CPU clocks");
}

inline RegisterUpdate field(const RegisterStorage storage,
                            const std::uint64_t mask,
                            const std::uint64_t before,
                            const std::uint64_t after,
                            const unsigned index = 0) {
  return {.storage = storage,
          .index = static_cast<std::uint8_t>(index),
          .mask = mask,
          .previous = before & mask,
          .value = after & mask};
}

inline MoveWriteback single(const unsigned selector,
                            const RegisterUpdate update,
                            const MoveProducer producer = MoveProducer::register_write) {
  return {.producer = producer,
          .selector = static_cast<std::uint8_t>(selector),
          .count = 1,
          .updates = {{update}}};
}

inline void publish(Core &core, const MoveWriteback &expected, const Registers &after) {
  auto action = pending(core);
  require(
      action.effect.kind == EffectKind::move_update && action.effect.move_update == expected &&
          !action.effect.address_update && action.effect.pc == boot,
      "Native MOVE producer, physical masks, old/new values and PC match independent expectations");
  action.effect.move_update = MoveWriteback{};
  action.computed = {};
  require(!core.advance_instruction(action.ticket).retired,
          "The stored native result, not a mutated observation, authorizes publication");
  held(core, after);
  rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
}

void scalars();
void parallel();
void composite();
void side_effects();
void restores();
} // namespace dsp56300::test::move_publication_test
