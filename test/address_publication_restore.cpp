// SPDX-License-Identifier: GPL-3.0-only
#include "address_publication_support.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::address_publication_test {
namespace {
constexpr Word paired_load = 0xf09800, x_address = 0x100, y_address = 0x200;
constexpr Word x_value = 0x123456, y_value = 0x654321;

Registers paired_state() {
  auto state = seed();
  state.r[4] = y_address;
  return state;
}

void pair_boundary(Core &core, const unsigned ordinal) {
  auto expected = paired_state();
  if (ordinal > 0) { expected.r[0] = x_address + 1; }
  if (ordinal > 1) { expected.r[4] = y_address + 1; }
  unchanged(core, expected);
  if (ordinal < 2) {
    const auto index = ordinal == 0 ? 0U : 4U;
    const auto address = ordinal == 0 ? x_address : y_address;
    writeback(
        core,
        {.index = static_cast<std::uint8_t>(index), .previous = address, .value = address + 1},
        address);
    return;
  }
  const auto action = pending(core);
  if (ordinal < 4) {
    require(action.effect.kind == EffectKind::read &&
                action.effect.space == (ordinal == 2 ? Space::x : Space::y) &&
                action.effect.address == (ordinal == 2 ? x_address : y_address),
            "A paired load keeps distinct authentic X then Y bus effects");
    static_cast<void>(core.advance_instruction(action.ticket));
    return;
  }
  retire(core);
  expected.x = x_value;
  expected.y = y_value;
  expected.pc = boot + 1;
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Restored paired native retirement has independently expected registers and progress");
}

struct BackendPair {
  Backend from, to;
};

void restore_pair(const BackendPair selected, const unsigned cut) {
  Bus bus;
  bus.events = {{.space = Space::x, .address = x_address, .value = x_value},
                {.space = Space::y, .address = y_address, .value = y_value}};
  Core original(bus, {.backend = selected.from});
  original.set_registers(paired_state());
  static_cast<void>(original.begin_instruction({.address = boot, .opcode = paired_load}, phased));
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { pair_boundary(original, ordinal); }
  const auto saved = original.checkpoint();
  const auto old = pending(original);
  auto tampered = old;
  tampered.effect.address_update = AddressWriteback{.index = 7, .value = word_mask};
  tampered.computed.r.fill(word_mask);
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = selected.to});
  resumed.restore(saved);
  rejected([&] { static_cast<void>(resumed.advance_instruction(old.ticket)); });
  rejected([&] { static_cast<void>(resumed.advance_instruction(tampered.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(old.ticket)); });
  for (unsigned ordinal = cut; ordinal < 5; ++ordinal) {
    pair_boundary(resumed, ordinal);
    pair_boundary(original, ordinal);
  }
  require(bus.calls == 2 && resumed_bus.calls == 2 && bus.exact && resumed_bus.exact,
          "Paired external continuation never repeats an accepted X/Y access");
}

void fault_prefix(const Backend backend) {
  Bus bus;
  bus.events = {{.address = x_address, .accept = false}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  static_cast<void>(
      core.begin_instruction({.address = boot, .opcode = staged_test::read_x0}, phased));
  writeback(core, {.previous = x_address, .value = x_address + 1}, x_address);
  const auto saved = core.checkpoint();
  const auto access = pending(core);
  state.r[0] = x_address + 1;
  const auto failed = core.advance_instruction(access.ticket);
  require(failed.fault.kind == FaultKind::memory_read && failed.fault.address == x_address &&
              failed.fault.pc == boot && !failed.pending && !failed.retired,
          "Rejected bus access faults visibly after an already accepted native R update");
  unchanged(core, state);
  core.restore(saved);
  unchanged(core, state);
  rejected([&] { static_cast<void>(core.advance_instruction(access.ticket)); });
  const auto restored = pending(core);
  core.reset();
  require(!core.pending_instruction() && core.registers().r[0] == 0,
          "Reset cancels pending and published address continuation");
  rejected([&] { static_cast<void>(core.advance_instruction(restored.ticket)); });
}

void update_only(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  constexpr Word update_r0 = 0x205800;
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = update_r0}, phased));
  auto observed = pending(core);
  require(observed.effect.kind == EffectKind::address_update && observed.effect.address_update,
          "Pure U exposes an actual typed native address writeback");
  if (!observed.effect.address_update) { throw std::runtime_error("Missing native R writeback"); }
  observed.effect.address_update.value().value = word_mask;
  static_cast<void>(core.advance_instruction(observed.ticket));
  state.r[0] = x_address + 1;
  unchanged(core, state);
  require(bus.calls == 0, "Tampering with the observed copy cannot inject a native R value");
  retire(core);
  state.pc = boot + 1;
  require(core.registers() == state, "Pure U publishes R without a fictitious data MOVE");
}

void nonupdates(const Backend backend) {
  struct Input {
    Word opcode, address;
    bool extension;
  };

  constexpr std::array inputs{
      Input{.opcode = 0x44e000, .address = x_address, .extension = false},
      Input{.opcode = 0x44e800, .address = x_address + 3, .extension = false},
      Input{.opcode = 0x44f000, .address = x_address, .extension = true}};
  for (const auto input : inputs) {
    Bus bus;
    bus.events = {{.address = input.address}};
    Core core(bus, {.backend = backend});
    auto state = seed();
    state.n[0] = 3;
    core.set_registers(state);
    InstructionWords words{.address = boot, .opcode = input.opcode};
    if (input.extension) { words.extension = x_address; }
    static_cast<void>(core.begin_instruction(words, phased));
    unchanged(core, state);
    const auto access = pending(core);
    require(access.effect.kind == EffectKind::read && !access.effect.address_update &&
                access.effect.address == input.address,
            "Indirect, indexed and absolute-extension reads invent no R writeback");
    static_cast<void>(core.advance_instruction(access.ticket));
    retire(core);
    state.x = staged_test::datum;
    state.pc = boot + (input.extension ? 2U : 1U);
    require(core.registers() == state && bus.calls == 1 && bus.exact,
            "Non-updating modes retain their authentic native EA and final state");
  }
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  constexpr Word x0_to_r0 = 0x209000;
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = x0_to_r0}, phased));
  unchanged(core, state);
  retire(core);
  state.r[0] = 1;
  state.pc = boot + 1;
  require(core.registers() == state && bus.calls == 0,
          "An explicit R MOVE remains a later destination, distinct from automatic AGU update");
}

void old_alu_operands(const Backend backend) {
  Bus bus;
  bus.events = {{.address = x_address}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  constexpr Word add_with_load = 0x44d840;
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = add_with_load}, phased));
  writeback(core, {.previous = x_address, .value = x_address + 1}, x_address);
  state.r[0] = x_address + 1;
  unchanged(core, state);
  static_cast<void>(core.advance_instruction(pending(core).ticket));
  unchanged(core, state);
  retire(core);
  state.pc = boot + 1;
  state.x = staged_test::datum;
  state.a = std::uint64_t{1} << word_bits;
  state.sr = flag_u;
  require(core.registers() == state && bus.calls == 1 && bus.exact,
          "Parallel ALU uses the old X0 while the load and early R retain their separate results");
}
} // namespace

void paired_restore() {
  for (const auto from : backends) {
    for (const auto to : backends) {
      for (unsigned cut = 0; cut < 5; ++cut) { restore_pair({.from = from, .to = to}, cut); }
    }
  }
}

void controls() {
  for (const auto backend : backends) {
    fault_prefix(backend);
    update_only(backend);
    nonupdates(backend);
    old_alu_operands(backend);
  }
  Bus bus;
  Core core(bus);
  core.set_registers(seed());
  constexpr std::uint8_t invalid_mode = 0xff;
  rejected([&] {
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = 0},
                                             std::bit_cast<InstructionPublication>(invalid_mode)));
  });
  require(!core.pending_instruction() && core.registers() == seed(),
          "Unknown publication policies reject before owning or modifying CPU state");
}
} // namespace dsp56300::test::address_publication_test
