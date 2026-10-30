// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

namespace dsp56300::test::address_publication_test {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
using staged_test::seed;
inline constexpr auto phased = InstructionPublication::address_updates;

inline void unchanged(Core &core, const Registers &expected) {
  require(core.registers() == expected,
          "Only an accepted native address writeback can change early architectural state");
  require(core.progress() == Progress{}, "Publication never retires or advances base cycles");
}

inline void writeback(Core &core, const AddressWriteback expected, const Word address) {
  const auto action = pending(core);
  require(action.effect.kind == EffectKind::address_update &&
              action.effect.address_update == expected && action.effect.address == address &&
              action.effect.pc == boot,
          "Owned update preserves the independently expected old R, new R, index, EA and PC");
  require(action.computed.r.at(expected.index) == expected.value,
          "The publication originates in the actual native computation");
  require(!core.advance_instruction(action.ticket).retired,
          "Accepting native R writeback does not retire the instruction");
  rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
}

void paired_restore();
void controls();
} // namespace dsp56300::test::address_publication_test
