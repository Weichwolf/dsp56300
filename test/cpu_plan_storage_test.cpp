// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged.hpp"
#include "support.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
void bounded_storage() {
  detail::StagedState work;
  work.active = work.computing = work.planning = true;
  Registers registers;
  require(work.plan.size() == 8 && work.records.size() == 64,
          "CPU planning and accepted logs retain explicit bounded storage");
  for (unsigned index = 0; index < 8; ++index) {
    registers.r[0] = index + 1;
    require(work.sample({.kind = EffectKind::address_update,
                         .address_update = AddressWriteback{.previous = index, .value = index + 1}},
                        registers) == 0,
            "CPU publications need no external response operand");
  }
  const InstructionEffect ninth{.kind = EffectKind::pcu_update, .pcu_update = PcuWriteback{}};
  bool paused = false;
  try {
    static_cast<void>(work.sample(ninth, registers));
  } catch (const detail::InstructionPause &) { paused = true; }
  require(paused && work.pending == ninth && work.used == 0 && work.planned == 8,
          "A full plan stops at the next effect without accepting the prefix or faulting early");
  for (unsigned index = 0; index < 8; ++index) {
    require(work.plan.at(index).computed.r[0] == index + 1,
            "Bounded storage retains every distinct intermediate observation in order");
  }
  constexpr unsigned accepted_limit = 64;
  work.used = work.cursor = accepted_limit - 1;
  work.planned = 0;
  work.pending.reset();
  require(work.sample(ninth, registers) == 0 && work.planned == 1,
          "The final remaining accepted-log slot can hold one private CPU writer");
  paused = false;
  try {
    static_cast<void>(work.sample(ninth, registers));
  } catch (const detail::InstructionPause &) { paused = true; }
  require(paused && work.used == accepted_limit - 1,
          "A private plan cannot consume the last accepted slot or expose overflow early");
  work.used = work.cursor = accepted_limit;
  bool overflow = false;
  try {
    static_cast<void>(work.sample(ninth, registers));
  } catch (const std::overflow_error &) { overflow = true; }
  require(overflow, "Only an already full accepted log reports its existing capacity failure");
  work.plan_index = work.planned;
  work.following = ninth;
  require(work.capacity_replay(),
          "The last accepted writer must diagnose an extra effect before exposing another ticket");
  work.following = InstructionEffect{.kind = EffectKind::retire};
  require(!work.capacity_replay(),
          "An exactly full accepted log can still retire without a new operation");
  work.following = ninth;
  --work.used;
  require(!work.capacity_replay(), "A remaining accepted slot permits the original next boundary");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::bounded_storage();
    std::cout << dsp56300::test::checks << " bounded CPU plan storage checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
