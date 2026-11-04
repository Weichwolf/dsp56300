// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
using external_run::Bus;
using external_run::ce;
using external_run::origin;
constexpr Word add = 0x200040, movec_sr = 0x05f439;
constexpr auto policy = InstructionPublication::pcu_updates;

struct Writer {
  EffectKind kind;
  Registers visible;
};

void publications(Core &core,
                  const Registers &computed,
                  const std::span<const Writer> writers,
                  const Word cycles) {
  const auto base = core.progress();
  auto visible = core.registers();
  for (const auto &writer : writers) {
    auto action = staged_test::pending(core);
    require(action.effect.kind == writer.kind && action.computed == computed &&
                core.registers() == visible && core.progress() == base,
            "Captured cache modes retain each literal held writer and complete observation");
    const auto old_ticket = action.ticket;
    core.restore(core.checkpoint());
    staged_test::rejected([&] { static_cast<void>(core.advance_instruction(old_ticket)); });
    action = staged_test::pending(core);
    require(action.effect.kind == writer.kind && action.computed == computed,
            "Restored cache modes retain the next distinct computation without publishing it");
    action.effect = {};
    action.computed = {};
    require(!core.advance_instruction(action.ticket).retired &&
                core.registers() == writer.visible && core.progress() == base,
            "Owned CPU publications ignore modified caller observations and retain accounting");
    visible = writer.visible;
  }
  const auto action = staged_test::pending(core);
  require(action.effect.kind == EffectKind::retire && action.computed == computed &&
              core.registers() == visible,
          "The complete cache-mode result remains behind a genuine retirement ticket");
  require(core.advance_instruction(action.ticket).retired && core.registers() == computed &&
              core.progress() == Progress{.instructions = base.instructions + 1,
                                          .estimated_cycles = base.estimated_cycles + cycles},
          "Captured ADD and immediate MOVEC retain their independent one/two-clock base costs");
}

void arithmetic(Core &core) {
  const auto before = core.registers();
  auto computed = before;
  ++computed.pc;
  computed.a = 0x1000000;
  computed.sr |= flag_u;
  auto visible = computed;
  visible.pc = before.pc;
  const std::array writers{Writer{.kind = EffectKind::alu_update, .visible = visible},
                           Writer{.kind = EffectKind::pcu_update, .visible = computed}};
  static_cast<void>(core.begin_instruction({.address = before.pc, .opcode = add}, policy));
  publications(core, computed, writers, 1);
}

void status(Core &core, const Word value) {
  const auto before = core.registers();
  auto computed = before;
  computed.pc += 2;
  computed.sr = value;
  auto visible = computed;
  visible.pc = before.pc;
  const std::array writers{Writer{.kind = EffectKind::move_update, .visible = visible},
                           Writer{.kind = EffectKind::pcu_update, .visible = computed}};
  static_cast<void>(core.begin_instruction(
      {.address = before.pc, .opcode = movec_sr, .extension = value}, policy));
  publications(core, computed, writers, 2);
}

void cache_modes(const Backend backend,
                 const bool cacheable,
                 const bool enabled,
                 const bool prime) {
  Bus bus;
  bus.program(origin, {0}); // A real NOP cache fill, distinct from the captured ADD.
  const Options options{.backend = backend,
                        .cacheable_begin = cacheable ? origin : static_cast<Word>(address_words),
                        .cacheable_end = cacheable ? origin + 8 : static_cast<Word>(address_words)};
  Core core(bus, options);
  auto initial = initial_state();
  initial.x = 1;
  initial.sr = enabled ? ce : 0U;
  core.set_registers(initial);
  if (prime) {
    require(core.run(1).dispatches == 1 && bus.reads == 1,
            "The cache control starts from one actually executed and fetched NOP");
    core.set_registers(initial);
  }
  const auto reads = bus.reads;
  arithmetic(core);
  status(core, enabled ? 0U : ce);
  status(core, enabled ? ce : 0U);
  require(bus.reads == reads && bus.writes == 0 && bus.bits == 0 && bus.resets == 0,
          "Captured CPU writers and guest CE transitions invent no external operation");
  // Both transitions include a real CE clear, which invalidates any earlier line.
  if (cacheable && enabled) {
    auto before = core.registers();
    before.pc = origin;
    core.set_registers(before);
    const auto result = core.run(1);
    ++before.pc;
    require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                core.registers() == before && bus.reads == reads + 1,
            "Guest CE clear/re-enable invalidates the old line and fetches the real NOP again");
  }
}
}
}

int main() {
  try {
    for (const auto backend : dsp56300::test::external_run::backends) {
      for (const bool cacheable : {false, true}) {
        for (const bool enabled : {false, true}) {
          dsp56300::test::cache_modes(backend, cacheable, enabled, false);
        }
      }
      dsp56300::test::cache_modes(backend, true, true, true);
    }
    std::cout << dsp56300::test::checks << " captured cache-mode planning checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
