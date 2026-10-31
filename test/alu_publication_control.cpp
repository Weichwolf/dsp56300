// SPDX-License-Identifier: GPL-3.0-only
#include "alu_publication_support.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <cstdint>

namespace dsp56300::test::alu_publication_test {
namespace {
constexpr Word add_load = 0x44e040, data_address = 0x180, data = 0x654321;
constexpr Word conditional_add = 0x202040, update_condition = 0x1000, add = 0x200040;
constexpr Word normalize = 0x01d815, loop_flag = Word{1} << 15;

Registers initial() {
  auto state = seed(0);
  state.a = std::uint64_t{2} << word_bits;
  state.x = 1;
  state.r[0] = data_address;
  state.sr = flag_u;
  return state;
}

AluWriteback add_result(const Word opcode, const bool flags) {
  AluWriteback result{.opcode = opcode, .count = static_cast<std::uint8_t>(flags ? 2 : 1)};
  result.updates[0] = field(RegisterStorage::a,
                            accumulator_mask,
                            std::uint64_t{2} << word_bits,
                            std::uint64_t{3} << word_bits);
  if (flags) {
    result.updates[1] = field(RegisterStorage::sr, arithmetic_flags | flag_c, flag_u, flag_u);
  }
  return result;
}

void conditional(const Backend backend, const bool taken, const bool update_flags) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = initial();
  if (!taken) { state.sr |= flag_c; }
  core.set_registers(state);
  const Word opcode = conditional_add | (update_flags ? update_condition : 0);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, phased));
  held(core, state);
  if (taken) {
    state.a = std::uint64_t{3} << word_bits;
    publish(core, add_result(opcode, update_flags), state);
  }
  retire(core);
  ++state.pc;
  require(core.registers() == state && bus.calls == 0,
          "False IFcc creates no writer; IFcc without U retains no fictitious SR write");
}

void normalize_result(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = initial();
  state.a = std::uint64_t{1} << word_bits;
  state.sr |= flag_c;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = normalize}, phased));
  held(core, state);
  const auto old = state;
  state.a *= 2;
  --state.r[0];
  AluWriteback expected{.opcode = normalize, .count = 3};
  expected.updates = {{field(RegisterStorage::a, accumulator_mask, old.a, state.a),
                       field(RegisterStorage::r, word_mask, old.r[0], state.r[0]),
                       field(RegisterStorage::sr, arithmetic_flags, old.sr, state.sr)}};
  publish(core, expected, state);
  retire(core);
  ++state.pc;
  require(core.registers() == state,
          "Native NORM publishes data/R/flags together while retaining carry");
}

void loop_fault(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = initial();
  state.sr |= loop_flag;
  state.la = boot;
  state.lc = 1;
  state.sc = 0;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = add}, phased));
  const auto saved = core.checkpoint();
  const auto pending_result = pending(core);
  state.a = std::uint64_t{3} << word_bits;
  auto result = add_result(add, true);
  result.updates[1] = field(RegisterStorage::sr, arithmetic_flags | flag_c, state.sr, state.sr);
  publish(core, result, state);
  require(core.fault() == Fault{.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = boot,
                                .value = state.sp,
                                .pc = boot} &&
              !core.pending_instruction(),
          "A later native loop-stack refusal preserves the admitted unit prefix");
  held(core, state);
  core.restore(saved);
  rejected([&] { static_cast<void>(core.advance_instruction(pending_result.ticket)); });
  state.a = std::uint64_t{2} << word_bits;
  held(core, state);
  core.reset();
  require(!core.pending_instruction() && core.progress() == Progress{},
          "Reset cancels pending unit state");
}

void pair_boundary(Core &core, const unsigned ordinal) {
  auto state = initial();
  if (ordinal > 1) { state.x = data; }
  if (ordinal > 2) { state.a = std::uint64_t{3} << word_bits; }
  held(core, state);
  auto action = pending(core);
  if (ordinal == 0) {
    require(action.effect.kind == EffectKind::read && action.effect.address == data_address,
            "A source-overwrite instruction retains its genuine data operation");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal == 1) {
    const MoveWriteback moved{
        .selector = 4, .count = 1, .updates = {{field(RegisterStorage::x, word_mask, 1, data)}}};
    require(action.effect.kind == EffectKind::move_update && action.effect.move_update == moved,
            "Actual MOVE producer precedes the native ALU result");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal == 2) {
    state.a = std::uint64_t{3} << word_bits;
    publish(core, add_result(add_load, true), state);
  } else {
    retire(core);
    ++state.pc;
    require(core.registers() == state &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
            "Restored native execution uses the old X0 despite its early published replacement");
  }
}

struct BackendPair {
  Backend from, to;
};

void restore(const BackendPair backend, const unsigned cut) {
  Bus bus;
  bus.events = {{.address = data_address, .value = data}};
  Core original(bus, {.backend = backend.from});
  original.set_registers(initial());
  static_cast<void>(original.begin_instruction({.address = boot, .opcode = add_load}, phased));
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { pair_boundary(original, ordinal); }
  const auto saved = original.checkpoint();
  const auto old = pending(original);
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = backend.to});
  resumed.restore(saved);
  rejected([&] { static_cast<void>(resumed.advance_instruction(old.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(old.ticket)); });
  for (unsigned ordinal = cut; ordinal < 4; ++ordinal) {
    pair_boundary(original, ordinal);
    pair_boundary(resumed, ordinal);
  }
  require(bus.calls == 1 && resumed_bus.calls == 1 && bus.exact && resumed_bus.exact,
          "Every restored unit suffix reuses its accepted source read without replay");
}
}

void controls() {
  for (const auto backend : backends) {
    for (const bool taken : {false, true}) {
      for (const bool update_flags : {false, true}) { conditional(backend, taken, update_flags); }
    }
    normalize_result(backend);
    loop_fault(backend);
  }
}

void restores() {
  for (const auto from : backends) {
    for (const auto to : backends) {
      for (unsigned cut = 0; cut < 4; ++cut) { restore({.from = from, .to = to}, cut); }
    }
  }
}
} // namespace dsp56300::test::alu_publication_test
