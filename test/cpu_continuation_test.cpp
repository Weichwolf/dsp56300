// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using unit_publication_test::field;
constexpr auto policy = InstructionPublication::pcu_updates;
constexpr Word target = 0x300, call = 0x0d0300, return_opcode = 0xc;
constexpr Word sp_mask = 63, sc_mask = 31;
constexpr Word bit_call = 0x0bce20, short_load = 0x44d800;
constexpr std::uint64_t limited = 0x01400000000000, old_x = 0x345678000001;

struct Writer {
  InstructionEffect effect;
  Registers computed;
};

struct Fixture {
  InstructionWords input;
  Registers before;
  std::array<Writer, 2> writers{};
};

struct BackendPair {
  Backend from, to;
};

InstructionEffect pcu(const InstructionWords input, const PcuWriteback writer) {
  return {.kind = EffectKind::pcu_update, .pc = input.address, .pcu_update = writer};
}

Fixture control(const bool returning) {
  Fixture result{.input = {.address = boot, .opcode = returning ? return_opcode : call},
                 .before = staged_test::seed()};
  auto &before = result.before;
  before.sr = flag_c;
  const auto frame = (std::uint64_t{returning ? target : boot + 1} << word_bits) | before.sr;
  if (returning) {
    before.sp = before.sc = 1;
    before.stack[1] = frame;
  }
  auto prefix = before;
  prefix.pc = boot + 1;
  prefix.sp = prefix.sc = returning ? 0U : 1U;
  if (!returning) { prefix.stack[1] = frame; }
  PcuWriteback first{.opcode = result.input.opcode,
                     .count = static_cast<std::uint8_t>(returning ? 3U : 4U)};
  first.updates[0] = field(RegisterStorage::sp, sp_mask, before.sp, prefix.sp);
  first.updates[1] = field(RegisterStorage::sc, sc_mask, before.sc, prefix.sc);
  const auto pc_index = returning ? 2U : 3U;
  if (!returning) { first.updates[2] = field(RegisterStorage::stack, long_mask, 0, frame, 1); }
  first.updates[pc_index] = field(RegisterStorage::pc, word_mask, boot, boot + 1);
  result.writers[0] = {.effect = pcu(result.input, first), .computed = prefix};
  prefix.pc = target;
  result.writers[1] = {
      .effect = pcu(result.input,
                    {.opcode = result.input.opcode,
                     .count = 1,
                     .updates = {{field(RegisterStorage::pc, word_mask, boot + 1, target)}}}),
      .computed = prefix};
  return result;
}

void finish(Core &core, const Fixture &fixture, const unsigned cut) {
  for (unsigned phase = cut; phase < fixture.writers.size(); ++phase) {
    const auto &expected = fixture.writers.at(phase);
    auto action = pending(core);
    require(
        action.effect == expected.effect && action.computed == expected.computed &&
            core.registers() ==
                (phase == 0 ? fixture.before : fixture.writers.at(phase - 1).computed) &&
            core.progress() == Progress{} && core.fault().kind == FaultKind::none,
        "Each JSR/RTS writer retains its literal intermediate PC, stack and complete observation");
    rejected([&] { static_cast<void>(core.candidate_checkpoint(action.ticket)); });
    action.effect.pcu_update = {};
    action.computed = {};
    require(!core.advance_instruction(action.ticket).retired &&
                core.registers() == expected.computed && core.progress() == Progress{},
            "Only the protected next writer becomes visible; retirement stays held");
    rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
  }
  const auto action = pending(core);
  require(action.effect.kind == EffectKind::retire &&
              action.computed == fixture.writers.back().computed,
          "The terminal observation follows both actual writers");
  const auto completed = core.advance_instruction(action.ticket);
  require(completed.retired &&
              completed.progress == Progress{.instructions = 1, .estimated_cycles = 3} &&
              core.registers() == fixture.writers.back().computed &&
              core.instruction_outcome() == InstructionOutcome{.instructions = 1,
                                                               .address = boot,
                                                               .opcode = fixture.input.opcode,
                                                               .control_transfer = true},
          "Original control-transfer identity and FM base cost retire once");
}

void checkpoints(const BackendPair backend) {
  for (const bool returning : {false, true}) {
    const auto fixture = control(returning);
    for (unsigned cut = 0; cut < 3; ++cut) {
      Bus bus;
      Core original(bus, {.backend = backend.from});
      Core restored(bus, {.backend = backend.to});
      original.set_registers(fixture.before);
      static_cast<void>(original.begin_instruction(fixture.input, policy));
      for (unsigned phase = 0; phase < cut; ++phase) {
        static_cast<void>(original.advance_instruction(pending(original).ticket));
      }
      const auto stale = pending(original).ticket;
      const auto saved = original.checkpoint();
      restored.restore(saved);
      original.restore(saved);
      rejected([&] { static_cast<void>(original.advance_instruction(stale)); });
      rejected([&] { static_cast<void>(restored.advance_instruction(stale)); });
      finish(original, fixture, cut);
      finish(restored, fixture, cut);
      require(bus.calls == 0,
              "CPU result planning and restored continuations invent no Io operation");
    }
  }
}

void read_boundary(const Backend backend, const bool accept) {
  Bus bus;
  bus.events = {{.value = staged_test::raw_datum, .accept = accept}};
  Core core(bus, {.backend = backend});
  auto before = staged_test::seed();
  before.x = old_x;
  core.set_registers(before);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = short_load}, policy));
  auto computed = before;
  ++computed.pc;
  ++computed.r[0];
  auto action = pending(core);
  require(action.effect.kind == EffectKind::address_update && action.computed == computed &&
              bus.calls == 0,
          "A computed post-increment is private and does not sample its future operand");
  static_cast<void>(core.advance_instruction(action.ticket));
  auto visible = before;
  ++visible.r[0];
  action = pending(core);
  require(action.effect.kind == EffectKind::read && action.effect.address == before.r[0] &&
              action.computed == computed && core.registers() == visible && bus.calls == 0,
          "The next real read retains the old EA and exact prefix after its CPU writer");
  constexpr Word changed = 0x123456;
  bus.events[0].value = changed;
  const auto read = core.advance_instruction(action.ticket);
  require(bus.calls == 1 && bus.exact && !read.retired,
          "Exactly one actual host operation samples the changed input");
  if (!accept) {
    require(read.fault == Fault{.kind = FaultKind::memory_read,
                                .space = Space::x,
                                .address = before.r[0],
                                .pc = boot} &&
                core.registers() == visible && core.progress() == Progress{} && !read.pending,
            "A refused real input preserves only the accepted AGU prefix and exact fault");
    return;
  }
  computed.x = (old_x & (long_mask ^ word_mask)) | changed;
  action = pending(core);
  require(action.effect.kind == EffectKind::move_update && action.computed == computed,
          "The native MOVE result uses the actual changed operand without publishing the PC");
  static_cast<void>(core.advance_instruction(action.ticket));
  visible.x = computed.x;
  require(core.registers() == visible && bus.calls == 1,
          "The MOVE writer publishes alone without repeating the read");
  core.clear_code_cache();
  const auto saved = core.checkpoint();
  Core restored(bus, {.backend = backend});
  restored.restore(saved);
  for (Core *machine : {&core, &restored}) {
    action = pending(*machine);
    require(action.effect.kind == EffectKind::pcu_update && action.computed == computed,
            "A checkpoint retains the queued PC writer after host code-cache mutation");
    static_cast<void>(machine->advance_instruction(action.ticket));
    staged_test::retire(*machine);
    require(machine->registers() == computed &&
                machine->progress() == Progress{.instructions = 1, .estimated_cycles = 1} &&
                bus.calls == 1,
            "Original/restored retirement never resamples an accepted input");
  }
}

void store_boundary(const Backend backend, const bool accept) {
  // FM MOVEM SSH,P:(R0)+: AGU update, source stack pop, real P store, PC.
  constexpr Word opcode = 0x0758bc;
  constexpr Word value = 0x123456;
  Bus bus;
  bus.events = {{.kind = EffectKind::write,
                 .space = Space::program,
                 .address = staged_test::data_address,
                 .value = value,
                 .accept = accept}};
  Core core(bus, {.backend = backend});
  auto visible = staged_test::seed();
  visible.sp = visible.sc = 1;
  visible.stack[1] = std::uint64_t{value} << word_bits;
  core.set_registers(visible);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  auto computed = visible;
  ++computed.pc;
  ++computed.r[0];
  auto action = pending(core);
  require(action.effect.kind == EffectKind::address_update && action.computed == computed &&
              bus.calls == 0,
          "The first queued CPU writer precedes source evaluation and actual program storage");
  static_cast<void>(core.advance_instruction(action.ticket));
  ++visible.r[0];
  computed.sp = computed.sc = 0;
  action = pending(core);
  require(action.effect.kind == EffectKind::move_update && action.computed == computed &&
              core.registers() == visible && bus.calls == 0,
          "The second writer retains the distinct native source-pop observation");
  static_cast<void>(core.advance_instruction(action.ticket));
  visible.sp = visible.sc = 0;
  action = pending(core);
  require(action.effect.kind == EffectKind::write && action.effect.space == Space::program &&
              action.effect.address == staged_test::data_address && action.effect.value == value &&
              action.computed == computed && core.registers() == visible && bus.calls == 0,
          "Both accepted CPU writers precede the one unchanged real program write");
  const auto stale = action.ticket;
  core.restore(core.checkpoint());
  rejected([&] { static_cast<void>(core.advance_instruction(stale)); });
  const auto result = core.advance_instruction(pending(core).ticket);
  require(bus.calls == 1 && bus.exact,
          "Restored CPU-prefix ownership performs the program write exactly once");
  if (!accept) {
    require(
        result.fault == Fault{.kind = FaultKind::memory_write,
                              .space = Space::program,
                              .address = staged_test::data_address,
                              .value = value,
                              .pc = boot} &&
            core.registers() == visible && core.progress() == Progress{} && !result.pending,
        "Write refusal retains both actual CPU writers and the original complete fault context");
    return;
  }
  require(pending(core).effect.kind == EffectKind::pcu_update && core.registers() == visible,
          "Accepted storage cannot publish its later PC or retirement early");
  static_cast<void>(core.advance_instruction(pending(core).ticket));
  staged_test::retire(core);
  ++visible.pc;
  require(core.registers() == visible && core.progress().instructions == 1 && bus.calls == 1,
          "The source-pop, AGU and actual memory prefix survive once-only native retirement");
}

void late_fault(const BackendPair backend) {
  Bus bus;
  Core original(bus, {.backend = backend.from});
  auto before = staged_test::seed();
  before.a = limited;
  before.sc = stack_entries - 1;
  before.sp = 7;
  original.set_registers(before);
  static_cast<void>(original.begin_instruction(
      {.address = boot, .opcode = bit_call, .extension = target}, policy));
  auto computed = before;
  computed.pc += 2;
  computed.sr = flag_s | flag_l;
  const auto action = pending(original);
  require(action.effect.kind == EffectKind::unit_update && action.computed == computed &&
              original.fault().kind == FaultKind::none && original.registers() == before,
          "A later stack failure stays behind the real earlier source-limiting publication");
  const auto saved = original.checkpoint();
  Core restored(bus, {.backend = backend.to});
  restored.restore(saved);
  auto visible = before;
  visible.sr = flag_s | flag_l;
  for (Core *machine : {&original, &restored}) {
    const auto failed = machine->advance_instruction(pending(*machine).ticket);
    require(
        failed.fault == Fault{.kind = FaultKind::stack,
                              .space = Space::program,
                              .address = boot,
                              .value = before.sp,
                              .pc = boot} &&
            !failed.pending && !failed.retired && machine->registers() == visible &&
            machine->progress() == Progress{} && bus.calls == 0,
        "Fault fallback and restore preserve the accepted CPU prefix without fake success or Io");
  }
}

} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto from : dsp56300::test::backends) {
      for (const bool accept : {false, true}) {
        dsp56300::test::read_boundary(from, accept);
        dsp56300::test::store_boundary(from, accept);
      }
      for (const auto to : dsp56300::test::backends) {
        dsp56300::test::checkpoints({.from = from, .to = to});
        dsp56300::test::late_fault({.from = from, .to = to});
      }
    }
    std::cout << dsp56300::test::checks << " bounded CPU continuation checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
