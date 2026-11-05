// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::unit_publication_test {
namespace {
constexpr Word memory_clear = 0x0a5800, bit_call_a = 0x0bce20, bit_call_ssh = 0x0bfc00;
constexpr Word target = 0x300, data_address = staged_test::data_address;
constexpr Word register_clear = 0x0ac440;
constexpr Word io_address = 0xffffc0;
constexpr std::uint64_t limited_accumulator = 0x01400000000000;

void memory_boundary(Core &core, const unsigned ordinal) {
  auto state = seed();
  state.r[0] = io_address + (ordinal > 0 ? 1U : 0U);
  if (ordinal > 2) { state.sr = flag_c; }
  held(core, state);
  const auto next = pending(core);
  if (ordinal == 0) {
    require(next.effect.kind == EffectKind::address_update && next.effect.address == io_address &&
                next.effect.address_update ==
                    AddressWriteback{.previous = io_address, .value = io_address + 1},
            "Native AGU publication precedes the real memory bit operation");
    static_cast<void>(core.advance_instruction(next.ticket));
  } else if (ordinal == 1) {
    require(next.effect.kind == EffectKind::bit && next.effect.address == io_address &&
                next.effect.action == BitAction::clear && next.effect.bit == 0,
            "The original owned atomic bit transaction remains an actual Io operation");
    static_cast<void>(core.advance_instruction(next.ticket));
  } else if (ordinal == 2) {
    const UnitWriteback result{.unit = RegisterUnit::bit,
                               .opcode = memory_clear,
                               .count = 1,
                               .updates = {{field(RegisterStorage::sr, flag_c, 0, flag_c)}}};
    state.sr = flag_c;
    publish(core, result, state);
  } else {
    retire(core);
    ++state.pc;
    require(core.registers() == state && core.progress().instructions == 1,
            "Native bit retirement retains the actual accepted address/Io/carry prefix");
  }
}

struct BackendPair {
  Backend from, to;
};

void restore(const BackendPair backend, const unsigned cut) {
  Bus bus;
  bus.events = {
      {.kind = EffectKind::bit, .address = io_address, .value = 1, .action = BitAction::clear}};
  Core original(bus, {.backend = backend.from});
  auto state = seed();
  state.r[0] = io_address;
  original.set_registers(state);
  static_cast<void>(original.begin_instruction({.address = boot, .opcode = memory_clear}, policy));
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { memory_boundary(original, ordinal); }
  const auto saved = original.checkpoint();
  const auto stale = pending(original);
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = backend.to});
  resumed.restore(saved);
  rejected([&] { static_cast<void>(resumed.advance_instruction(stale.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(stale.ticket)); });
  for (unsigned ordinal = cut; ordinal < 4; ++ordinal) {
    memory_boundary(original, ordinal);
    memory_boundary(resumed, ordinal);
  }
  require(bus.calls == 1 && resumed_bus.calls == 1 && bus.exact && resumed_bus.exact,
          "Every unit checkpoint/backend suffix retains its genuine accepted bit response");
}

void refused_bit(const Backend backend) {
  Bus bus;
  bus.events = {{.kind = EffectKind::bit,
                 .address = io_address,
                 .action = BitAction::clear,
                 .accept = false}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.r[0] = io_address;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = memory_clear}, policy));
  memory_boundary(core, 0);
  const auto next = pending(core);
  static_cast<void>(core.advance_instruction(next.ticket));
  require(core.fault() == Fault{.kind = FaultKind::io_bit,
                                .space = Space::x,
                                .address = io_address,
                                .pc = boot} &&
              !core.pending_instruction() && bus.calls == 1 && bus.accepted == 0 && bus.exact,
          "Real Io refusal preserves the accepted AGU update without inventing carry or success");
  ++state.r[0];
  held(core, state);
}

void late_call_fault(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.a = limited_accumulator;
  state.sc = 15;
  core.set_registers(state);
  static_cast<void>(
      core.begin_instruction({.address = boot, .opcode = bit_call_a, .extension = target}, policy));
  const auto saved = core.checkpoint();
  const auto stale = pending(core);
  held(core, state);
  const UnitWriteback result{
      .unit = RegisterUnit::bit,
      .opcode = bit_call_a,
      .count = 1,
      .updates = {{field(RegisterStorage::sr, flag_s | flag_l, 0, flag_s | flag_l)}}};
  state.sr = flag_s | flag_l;
  publish(core, result, state);
  require(core.fault() == Fault{.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = boot,
                                .value = state.sp,
                                .pc = boot} &&
              !core.pending_instruction() && bus.calls == 0,
          "Actual source-limiting flags precede and survive a later PCU stack refusal");
  core.restore(saved);
  rejected([&] { static_cast<void>(core.advance_instruction(stale.ticket)); });
  state.sr = 0;
  held(core, state);
  const auto restored = pending(core);
  core.reset();
  require(!core.pending_instruction() && core.progress() == Progress{},
          "Reset cancels the pending native scalar unit");
  rejected([&] { static_cast<void>(core.advance_instruction(restored.ticket)); });
}

void pop_then_call(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  const auto before = state;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction(
      {.address = boot, .opcode = bit_call_ssh, .extension = target}, policy));
  --state.sp;
  --state.sc;
  const UnitWriteback result{
      .unit = RegisterUnit::bit,
      .opcode = bit_call_ssh,
      .count = 2,
      .updates = {{field(RegisterStorage::sp, stack_pointer_mask, before.sp, state.sp),
                   field(RegisterStorage::sc, stack_counter_mask, before.sc, state.sc)}}};
  publish(core, result, state);
  retire(core);
  state.sp = before.sp;
  state.sc = before.sc;
  state.stack[2] = std::uint64_t{boot + 2} << word_bits;
  state.pc = target;
  require(core.registers() == state && bus.calls == 0,
          "A bit-call SSH read publishes its real pop before the distinct native PCU push");
}

void inherited_results(const Backend backend) {
  constexpr Word opcode = 0x44e040;
  constexpr Word payload = 0x654321;
  Bus bus;
  bus.events = {{.value = payload}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.r[0] = data_address;
  state.x = 1;
  state.a = std::uint64_t{2} << word_bits;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  const auto read = pending(core);
  require(read.effect.kind == EffectKind::read, "The extended policy preserves genuine data reads");
  static_cast<void>(core.advance_instruction(read.ticket));
  const MoveWriteback moved{
      .selector = 4, .count = 1, .updates = {{field(RegisterStorage::x, word_mask, 1, payload)}}};
  const auto move = pending(core);
  require(move.effect.kind == EffectKind::move_update && move.effect.move_update == moved,
          "Scalar-unit policy includes actual earlier MOVE producer boundaries");
  static_cast<void>(core.advance_instruction(move.ticket));
  state.x = payload;
  held(core, state);
  const AluWriteback summed{
      .opcode = opcode,
      .count = 2,
      .updates = {{field(RegisterStorage::a,
                         accumulator_mask,
                         std::uint64_t{2} << word_bits,
                         std::uint64_t{3} << word_bits),
                   field(RegisterStorage::sr, status_byte & ~(flag_s | flag_l), 0, flag_u)}}};
  const auto alu = pending(core);
  require(
      alu.effect.kind == EffectKind::alu_update && alu.effect.alu_update == summed,
      "Extended policy retains original ALU source latches despite the accepted X0 replacement");
  static_cast<void>(core.advance_instruction(alu.ticket));
  state.a = std::uint64_t{3} << word_bits;
  state.sr = flag_u;
  held(core, state);
  retire(core);
  ++state.pc;
  require(core.registers() == state && bus.calls == 1 && bus.exact,
          "Extended scalar policy preserves all actual native MOVE/ALU and external prefixes");
}

void late_loop_fault(const Backend backend) {
  constexpr Word loop_flag = Word{1} << 15;
  constexpr Word opcode = 0x0001f9;
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.sr = loop_flag;
  state.la = boot;
  state.lc = 1;
  state.sc = 0;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  const UnitWriteback result{.unit = RegisterUnit::control_byte,
                             .opcode = opcode,
                             .count = 1,
                             .updates = {{field(RegisterStorage::sr, status_byte, 0, flag_c)}}};
  state.sr |= flag_c;
  publish(core, result, state);
  require(core.fault() == Fault{.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = boot,
                                .value = state.sp,
                                .pc = boot} &&
              !core.pending_instruction(),
          "A later real loop-stack failure retains the accepted native control-byte prefix");
}

void prior_policies(const Backend backend) {
  constexpr std::array prior{InstructionPublication::retirement,
                             InstructionPublication::address_updates,
                             InstructionPublication::move_updates,
                             InstructionPublication::alu_updates};
  for (const auto selected : prior) {
    Bus bus;
    Core core(bus, {.backend = backend});
    auto state = seed();
    core.set_registers(state);
    static_cast<void>(
        core.begin_instruction({.address = boot, .opcode = register_clear}, selected));
    held(core, state);
    require(pending(core).effect.kind == EffectKind::retire,
            "Prior publication policies keep scalar-unit results held until native retirement");
    retire(core);
    state.x &= ~std::uint64_t{1};
    ++state.pc;
    require(core.registers() == state, "Existing scalar bit result remains unchanged");
  }
}
}

void boundaries() {
  for (const auto from : backends) {
    for (const auto to : backends) {
      for (unsigned cut = 0; cut < 4; ++cut) { restore({.from = from, .to = to}, cut); }
    }
    refused_bit(from);
    late_call_fault(from);
    pop_then_call(from);
    prior_policies(from);
    inherited_results(from);
    late_loop_fault(from);
  }
}
} // namespace dsp56300::test::unit_publication_test
