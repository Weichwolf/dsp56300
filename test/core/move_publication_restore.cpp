// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "move_publication_support.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::move_publication_test {
namespace {
constexpr Word saturated = 0x7fffff;
constexpr Word load_into_same_r = 0x60d800, y0_to_r0 = 0x20d000;
constexpr Word y_address = 0x200, x_value = 0x123456, y_value = 0x654321;
constexpr std::uint64_t large_accumulator = 0x01400000000000;
enum class Scenario : std::uint8_t { pair, flags, stack };

Registers initial(const Scenario scenario) {
  auto state = seed();
  if (scenario == Scenario::pair) { state.r[4] = y_address; }
  if (scenario == Scenario::flags) { state.a = large_accumulator; }
  return state;
}

void start(Core &core, const Scenario scenario) {
  core.set_registers(initial(scenario));
  constexpr std::array instructions{Word{0xf09800}, Word{0x21c400}, Word{0x0444bc}};
  static_cast<void>(core.begin_instruction(
      {.address = boot, .opcode = instructions.at(static_cast<unsigned>(scenario))}, phased));
}

void pair_move(Core &core, Registers &state, const bool x) {
  const auto old = x ? state.x : state.y;
  auto &value = x ? state.x : state.y;
  value = (old & main_mask) | (x ? x_value : y_value);
  publish(core,
          single(x ? 4U : 6U,
                 field(x ? RegisterStorage::x : RegisterStorage::y, word_mask, old, value)),
          state);
}

void pair_boundary(Core &core, const unsigned ordinal) {
  auto state = initial(Scenario::pair);
  if (ordinal > 0) { ++state.r[0]; }
  if (ordinal > 1) { ++state.r[4]; }
  if (ordinal > 4) { state.x = (state.x & main_mask) | x_value; }
  if (ordinal > 5) { state.y = (state.y & main_mask) | y_value; }
  held(core, state);
  if (ordinal < 2) {
    const auto index = ordinal == 0 ? 0U : 4U;
    const auto address = ordinal == 0 ? data_address : y_address;
    const auto action = pending(core);
    require(action.effect.kind == EffectKind::address_update &&
                action.effect.address_update ==
                    AddressWriteback{.index = static_cast<std::uint8_t>(index),
                                     .previous = address,
                                     .value = address + 1} &&
                action.effect.address == address,
            "MOVE mode retains authentic automatic R producers before paired data reads");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal < 4) {
    const auto action = pending(core);
    require(action.effect.kind == EffectKind::read &&
                action.effect.space == (ordinal == 2 ? Space::x : Space::y) &&
                action.effect.address == (ordinal == 2 ? data_address : y_address),
            "Paired publication keeps distinct actual X/Y effects");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal < 6) {
    pair_move(core, state, ordinal == 4);
  } else {
    retire(core);
    ++state.pc;
    require(
        core.registers() == state &&
            core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
        "Paired restored retirement retains every early result and genuine hidden continuation");
  }
}

void side_boundary(Core &core, const unsigned ordinal, const Scenario scenario) {
  auto state = initial(scenario);
  const bool stack = scenario == Scenario::stack;
  if (ordinal > 0) {
    if (stack) {
      state.sp = 1;
      state.sc = 1;
    } else {
      state.sr = flag_s | flag_l;
    }
  }
  const Word value = stack ? register_model::frame_high + 2 : saturated;
  if (ordinal > 1) { state.x = (state.x & main_mask) | value; }
  held(core, state);
  if (ordinal == 0) {
    MoveWriteback expected;
    if (stack) {
      state.sp = 1;
      state.sc = 1;
      expected = {.producer = MoveProducer::stack_read,
                  .selector = ssh,
                  .count = 2,
                  .updates = {{field(RegisterStorage::sp, stack_pointer_mask, 2, 1),
                               field(RegisterStorage::sc, stack_counter_mask, 2, 1)}}};
    } else {
      state.sr = flag_s | flag_l;
      expected = single(14,
                        field(RegisterStorage::sr, flag_s | flag_l, 0, state.sr),
                        MoveProducer::accumulator_read);
    }
    publish(core, expected, state);
  } else if (ordinal == 1) {
    const auto before = state.x;
    state.x = (state.x & main_mask) | value;
    publish(core, single(4, field(RegisterStorage::x, word_mask, before, state.x)), state);
  } else {
    retire(core);
    ++state.pc;
    require(core.registers() == state && core.progress().instructions == 1,
            "Restored stack/status effects preserve full state without early PC or progress");
  }
}

void boundary(Core &core, const unsigned ordinal, const Scenario scenario) {
  if (scenario == Scenario::pair) {
    pair_boundary(core, ordinal);
  } else {
    side_boundary(core, ordinal, scenario);
  }
}

void restore_case(const Backend selected, const unsigned cut, const Scenario scenario) {
  Bus bus;
  if (scenario == Scenario::pair) {
    bus.events = {{.space = Space::x, .address = data_address, .value = x_value},
                  {.space = Space::y, .address = y_address, .value = y_value}};
  }
  Core original(bus, {.backend = selected});
  start(original, scenario);
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { boundary(original, ordinal, scenario); }
  const auto steps = scenario == Scenario::pair ? 7U : 3U;
  for (unsigned ordinal = cut; ordinal < steps; ++ordinal) {
    boundary(original, ordinal, scenario);
  }
  const auto calls = scenario == Scenario::pair ? 2U : 0U;
  require(bus.calls == calls && bus.exact,
          "Every actual native suffix consumes each external response only once");
}

void explicit_r_alias(const Backend backend) {
  Bus bus;
  bus.events = {{.address = data_address, .value = data_address}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = load_into_same_r}, phased));
  const auto automatic = pending(core);
  require(automatic.effect.kind == EffectKind::address_update &&
              automatic.effect.address_update ==
                  AddressWriteback{.previous = data_address, .value = data_address + 1},
          "Automatic R and the later explicit same-R destination retain distinct native producers");
  static_cast<void>(core.advance_instruction(automatic.ticket));
  ++state.r[0];
  held(core, state);
  const auto read = pending(core);
  require(read.effect.kind == EffectKind::read, "The original R addresses its actual data read");
  static_cast<void>(core.advance_instruction(read.ticket));
  --state.r[0];
  publish(core,
          single(16, field(RegisterStorage::r, word_mask, data_address + 1, data_address)),
          state);
  retire(core);
  ++state.pc;
  require(
      core.registers() == state && bus.calls == 1 && bus.exact,
      "Ordered native publication preserves an explicit R write that cancels an automatic update");
}

void mode_and_reset(const Backend backend) {
  for (const auto mode :
       {InstructionPublication::retirement, InstructionPublication::address_updates}) {
    Bus bus;
    Core core(bus, {.backend = backend});
    auto state = seed();
    core.set_registers(state);
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = y0_to_r0}, mode));
    held(core, state);
    retire(core);
    state.r[0] = payload;
    ++state.pc;
    require(core.registers() == state && bus.calls == 0,
            "Existing publication modes keep explicit R writes at retirement");
  }
  Bus bus;
  Core core(bus, {.backend = backend});
  start(core, Scenario::flags);
  side_boundary(core, 0, Scenario::flags);
  const auto old = pending(core);
  core.reset();
  require(!core.pending_instruction() && core.registers().sr == 0 && core.progress() == Progress{},
          "Reset cancels published MOVE state and its native continuation");
  rejected([&] { static_cast<void>(core.advance_instruction(old.ticket)); });
}
}

void restores() {
  for (const auto scenario : {Scenario::pair, Scenario::flags, Scenario::stack}) {
    const auto steps = scenario == Scenario::pair ? 7U : 3U;
    for (const auto from : backends) {

      for (unsigned cut = 0; cut < steps; ++cut) { restore_case(from, cut, scenario); }
    }
  }
  for (const auto backend : backends) {
    mode_and_reset(backend);
    explicit_r_alias(backend);
  }
}
} // namespace dsp56300::test::move_publication_test
