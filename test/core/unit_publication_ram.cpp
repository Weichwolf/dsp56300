// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>

namespace dsp56300::test::unit_publication_test {
namespace {
constexpr Word ram_address = staged_test::data_address;
constexpr Word raw_prefix = 0xab000000;
constexpr Word old_set = 0x800001;
constexpr Word old_clear = 0x800000;
constexpr Word y_space_bit = Word{1} << 6;

struct RamCase {
  Word opcode, before, after;
  bool modify = true;
  Space space = Space::x;
};

unsigned carry_phase(const RamCase input) {
  return input.modify ? 3U : 2U;
}

Registers ram_state(const RamCase input, const unsigned phase) {
  auto state = seed();
  state.r[0] = ram_address + (phase > 0 ? 1U : 0U);
  if (phase > carry_phase(input)) { state.sr = input.before & flag_c; }
  return state;
}

void ram_phase(Core &core, const RamCase input, const unsigned phase) {
  auto state = ram_state(input, phase);
  held(core, state);
  const auto action = pending(core);
  if (phase == 0) {
    require(action.effect.kind == EffectKind::address_update &&
                action.effect.address == ram_address &&
                action.effect.address_update ==
                    AddressWriteback{.previous = ram_address, .value = ram_address + 1},
            "Real RAM bit addressing publishes its native automatic pointer first");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (phase == 1 || (phase == 2 && input.modify)) {
    const auto kind = phase == 1 ? EffectKind::read : EffectKind::write;
    require(action.effect.kind == kind && action.effect.space == input.space &&
                action.effect.address == ram_address &&
                (kind != EffectKind::write || action.effect.value == input.after),
            "RAM bit execution owns distinct real reads and exact masked native writes");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (phase == carry_phase(input)) {
    const UnitWriteback result{
        .unit = RegisterUnit::bit,
        .opcode = input.opcode,
        .count = 1,
        .updates = {{field(RegisterStorage::sr, flag_c, 0, input.before & flag_c)}}};
    state.sr = input.before & flag_c;
    publish(core, result, state);
  } else {
    retire(core);
    ++state.pc;
    require(core.registers() == state && core.progress().instructions == 1,
            "RAM bit retirement retains actual read/write and native address/carry prefixes");
  }
}

Bus ram_bus(const RamCase input) {
  Bus bus;
  bus.events = {{.space = input.space, .value = raw_prefix | input.before}};
  if (input.modify) {
    bus.events.push_back({.kind = EffectKind::write, .space = input.space, .value = input.after});
  }
  return bus;
}

struct BackendPair {
  Backend from, to;
};

void ram_restore(const BackendPair backend, const RamCase input, const unsigned cut) {
  auto bus = ram_bus(input);
  Core original(bus, {.backend = backend.from});
  original.set_registers(ram_state(input, 0));
  static_cast<void>(original.begin_instruction({.address = boot, .opcode = input.opcode}, policy));
  for (unsigned phase = 0; phase < cut; ++phase) { ram_phase(original, input, phase); }
  // Accepted RAM samples belong to the CPU continuation, even if host RAM changes.
  if (cut > 1) { bus.events[0].value = raw_prefix | (input.before ^ flag_c); }
  const auto saved = original.checkpoint();
  const auto stale = pending(original);
  auto restored_bus = bus;
  Core restored(restored_bus, {.backend = backend.to});
  restored.restore(saved);
  rejected([&] { static_cast<void>(restored.advance_instruction(stale.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(stale.ticket)); });
  for (unsigned phase = cut; phase <= carry_phase(input) + 1; ++phase) {
    ram_phase(original, input, phase);
    ram_phase(restored, input, phase);
  }
  const auto count = input.modify ? 2U : 1U;
  require(bus.calls == count && restored_bus.calls == count && bus.accepted == count &&
              restored_bus.accepted == count && bus.exact && restored_bus.exact,
          "Restored RAM RMW/TEST never repeat a read/write or invoke peripheral bit callbacks");
}

void ram_refusal(const Backend backend, const bool writing, const Space space) {
  const RamCase input{.opcode = 0x0a5800 | (space == Space::y ? y_space_bit : 0U),
                      .before = old_set,
                      .after = old_clear,
                      .space = space};
  auto bus = ram_bus(input);
  bus.events[writing ? 1U : 0U].accept = false;
  Core core(bus, {.backend = backend});
  core.set_registers(ram_state(input, 0));
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = input.opcode}, policy));
  ram_phase(core, input, 0);
  if (writing) { ram_phase(core, input, 1); }
  const auto request = pending(core);
  static_cast<void>(core.advance_instruction(request.ticket));
  require(core.fault() == Fault{.kind = writing ? FaultKind::memory_write : FaultKind::memory_read,
                                .space = space,
                                .address = ram_address,
                                .value = writing ? old_clear : 0U,
                                .pc = boot} &&
              !core.pending_instruction() && bus.calls == (writing ? 2U : 1U) &&
              bus.accepted == (writing ? 1U : 0U) && bus.exact,
          "Real RAM refusals retain their native memory class/address/value and accepted prefix");
  held(core, ram_state(input, 1));
}

void ram_prior(const Backend backend, const InstructionPublication selected) {
  const RamCase input{.opcode = 0x0a5820, .before = old_set, .after = old_set};
  auto bus = ram_bus(input);
  Core core(bus, {.backend = backend});
  auto state = ram_state(input, 0);
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = input.opcode}, selected));
  if (selected != InstructionPublication::retirement) {
    const auto address = pending(core);
    require(address.effect.kind == EffectKind::address_update,
            "Older CPU policies retain their actual automatic address publication");
    static_cast<void>(core.advance_instruction(address.ticket));
    ++state.r[0];
  }
  for (const auto kind : {EffectKind::read, EffectKind::write}) {
    held(core, state);
    const auto action = pending(core);
    require(action.effect.kind == kind && action.effect.address == ram_address &&
                (kind != EffectKind::write || action.effect.value == old_set),
            "Every prior policy uses genuine RAM RMW even for unchanged destination bits");
    static_cast<void>(core.advance_instruction(action.ticket));
  }
  held(core, state);
  retire(core);
  state.r[0] = ram_address + 1;
  state.sr = flag_c;
  ++state.pc;
  require(core.registers() == state && bus.calls == 2 && bus.exact,
          "Correct RAM transport preserves earlier CPU policies and exact final native results");
}
}

namespace {
void ram_restore_space(const Backend from, const Space space) {
  constexpr std::array actions{
      RamCase{.opcode = 0x0a5800, .before = old_set, .after = old_clear},
      RamCase{.opcode = 0x0a5820, .before = old_clear, .after = old_set},
      RamCase{.opcode = 0x0b5800, .before = old_set, .after = old_clear},
      RamCase{.opcode = 0x0b5820, .before = old_set, .after = old_set, .modify = false}};
  for (const auto to : backends) {
    for (auto input : actions) {
      input.space = space;
      if (space == Space::y) { input.opcode |= y_space_bit; }
      for (unsigned cut = 0; cut <= carry_phase(input) + 1; ++cut) {
        ram_restore({.from = from, .to = to}, input, cut);
      }
    }
  }
}
}

void ram_boundaries() {
  constexpr std::array prior{InstructionPublication::retirement,
                             InstructionPublication::address_updates,
                             InstructionPublication::move_updates,
                             InstructionPublication::alu_updates};
  for (const auto from : backends) {
    for (const auto space : {Space::x, Space::y}) {
      for (const bool writing : {false, true}) { ram_refusal(from, writing, space); }
      ram_restore_space(from, space);
    }
    for (const auto selected : prior) { ram_prior(from, selected); }
  }
}
} // namespace dsp56300::test::unit_publication_test
