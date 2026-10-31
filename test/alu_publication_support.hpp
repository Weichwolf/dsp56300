// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::alu_publication_test {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
inline constexpr auto phased = InstructionPublication::alu_updates;
inline constexpr std::uint64_t main_mask = std::uint64_t{word_mask} << word_bits;
inline constexpr Word arithmetic_flags = flag_n | flag_z | flag_v | flag_e | flag_u;
inline constexpr Word logical_flags = flag_n | flag_z | flag_v;
inline constexpr Word sm = Word{1} << 20;

struct Expected {
  Registers state;
  std::uint64_t data_mask = accumulator_mask;
  Word status_mask = 0;
  bool write_data = true, destination_b = false, supported = true;
};

inline RegisterUpdate field(const RegisterStorage storage,
                            const std::uint64_t mask,
                            const std::uint64_t before,
                            const std::uint64_t value,
                            const unsigned index = 0) {
  return {.storage = storage,
          .index = static_cast<std::uint8_t>(index),
          .mask = mask,
          .previous = before & mask,
          .value = value & mask};
}

inline void held(Core &core, const Registers &state) {
  require(core.registers() == state && core.progress() == Progress{},
          "Unit results, PC and retirement remain held until their actual owned effect");
}

inline void publish(Core &core, const AluWriteback &expected, const Registers &after) {
  auto action = pending(core);
  require(action.effect.kind == EffectKind::alu_update && action.effect.alu_update == expected &&
              !action.effect.move_update && !action.effect.address_update,
          "Actual native unit opcode, first-written bits, final bits and masks match the "
          "independent expectation");
  action.effect.alu_update = AluWriteback{};
  action.computed = {};
  require(!core.advance_instruction(action.ticket).retired,
          "Only the stored native result authorizes publication, not a tampered observation");
  held(core, after);
  rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
}

[[nodiscard]] Registers seed(unsigned index);
[[nodiscard]] Expected expected(const Registers &before, Word code);
void matrix();
void controls();
void restores();
} // namespace dsp56300::test::alu_publication_test
