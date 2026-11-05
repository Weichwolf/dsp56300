// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::pcu_publication_test {
namespace {
using staged_test::backends;
using staged_test::boot;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
using unit_publication_test::field;
constexpr auto policy = InstructionPublication::pcu_updates;
constexpr Word target = 0x300, call = 0x0d0300, repeat_two = 0x0602a0;
constexpr Word stack_pointer_mask = 63, stack_counter_mask = 31;
constexpr Word bit_call_a = 0x0bce20;
constexpr Word reset_opcode = 0x84;
constexpr unsigned reset_cycles = 7;
constexpr std::uint64_t limited_accumulator = 0x01400000000000;

void publish(Core &core, const PcuWriteback &expected, const Registers &after) {
  auto action = pending(core);
  const auto progress = core.progress();
  require(
      action.effect.kind == EffectKind::pcu_update && action.effect.pcu_update == expected,
      "Actual native PCU masks, first-written prior bits and values match literal expectations");
  action.effect.pcu_update = PcuWriteback{};
  action.computed = {};
  require(!core.advance_instruction(action.ticket).retired && core.registers() == after &&
              core.progress() == progress,
          "Only the stored native PCU result changes registers; hidden retirement stays held");
  rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
}

PcuWriteback pc(const Word opcode, const Word before, const Word after) {
  return {.opcode = opcode,
          .count = 1,
          .updates = {{field(RegisterStorage::pc, word_mask, before, after)}}};
}

void nop(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = staged_test::seed();
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = 0}, policy));
  require(core.registers() == state, "Native sequential PC is held before its owned publication");
  ++state.pc;
  publish(core, pc(0, boot, boot + 1), state);
  retire(core);
  require(core.registers() == state &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 1} &&
              bus.calls == 0 &&
              core.instruction_outcome() ==
                  InstructionOutcome{.instructions = 1, .address = boot, .opcode = 0},
          "NOP commits native accounting once without copying an obsolete register file");
}

void call_phase(Core &core, const unsigned phase) {
  auto state = staged_test::seed();
  if (phase > 0) {
    state.sp = state.sc = 1;
    state.stack[1] = std::uint64_t{boot + 1} << word_bits;
    state.pc = boot + 1;
  }
  if (phase > 1) { state.pc = target; }
  require(core.registers() == state && core.progress() == Progress{},
          "Each call prefix retains every unrelated native field and all retirement counters");
  if (phase == 0) {
    const auto before = state;
    state.sp = state.sc = 1;
    state.stack[1] = std::uint64_t{boot + 1} << word_bits;
    state.pc = boot + 1;
    const PcuWriteback push{
        .opcode = call,
        .count = 4,
        .updates = {{field(RegisterStorage::sp, stack_pointer_mask, 0, 1),
                     field(RegisterStorage::sc, stack_counter_mask, 0, 1),
                     field(RegisterStorage::stack, long_mask, 0, state.stack[1], 1),
                     field(RegisterStorage::pc, word_mask, before.pc, state.pc)}}};
    publish(core, push, state);
  } else if (phase == 1) {
    state.pc = target;
    publish(core, pc(call, boot + 1, target), state);
  } else {
    retire(core);
    require(core.registers() == state && core.progress().instructions == 1 &&
                core.instruction_outcome() == InstructionOutcome{.instructions = 1,
                                                                 .address = boot,
                                                                 .opcode = call,
                                                                 .control_transfer = true},
            "A native call retires after its separate genuine stack and target publications");
  }
}

void call_restore(const Backend backend, const unsigned cut) {
  Bus bus;
  Core original(bus, {.backend = backend});
  original.set_registers(staged_test::seed());
  static_cast<void>(original.begin_instruction({.address = boot, .opcode = call}, policy));
  for (unsigned phase = 0; phase < cut; ++phase) { call_phase(original, phase); }
  for (unsigned phase = cut; phase < 3; ++phase) { call_phase(original, phase); }
  require(bus.calls == 0,
          "Actual PCU continuation has no invented memory or peripheral transaction");
}

void rti(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = staged_test::seed();
  state.sp = state.sc = 1;
  state.stack[1] = (std::uint64_t{target} << word_bits) | flag_c;
  core.set_registers(state);
  constexpr Word opcode = 4;
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  state.sp = state.sc = 0;
  ++state.pc;
  const PcuWriteback popped{.opcode = opcode,
                            .count = 3,
                            .updates = {{field(RegisterStorage::sp, stack_pointer_mask, 1, 0),
                                         field(RegisterStorage::sc, stack_counter_mask, 1, 0),
                                         field(RegisterStorage::pc, word_mask, boot, boot + 1)}}};
  publish(core, popped, state);
  state.pc = target;
  state.sr = flag_c;
  const PcuWriteback returned{
      .opcode = opcode,
      .count = 2,
      .updates = {{field(RegisterStorage::sr, word_mask, 0, flag_c),
                   field(RegisterStorage::pc, word_mask, boot + 1, target)}}};
  publish(core, returned, state);
  retire(core);
  require(!core.interruptible() && core.registers() == state &&
              core.instruction_outcome() == InstructionOutcome{.instructions = 1,
                                                               .address = boot,
                                                               .opcode = opcode,
                                                               .control_transfer = true},
          "RTI commits its real hidden inhibition after pop, PC and SR results");
}

void repeat(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = staged_test::seed();
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = repeat_two}, policy));
  const Word old_lc = state.lc;
  state.lc = 2;
  ++state.pc;
  const PcuWriteback setup{.opcode = repeat_two,
                           .count = 2,
                           .updates = {{field(RegisterStorage::lc, word_mask, old_lc, 2),
                                        field(RegisterStorage::pc, word_mask, boot, boot + 1)}}};
  publish(core, setup, state);
  retire(core);
  for (const bool last : {false, true}) {
    static_cast<void>(core.begin_instruction({.address = boot + 1, .opcode = 0}, policy));
    state.pc = boot + 2;
    publish(core, pc(0, boot + 1, boot + 2), state);
    const PcuWriteback tail =
        last ? PcuWriteback{.opcode = 0,
                            .count = 1,
                            .updates = {{field(RegisterStorage::lc, word_mask, 1, old_lc)}}}
             : PcuWriteback{
                   .opcode = 0,
                   .count = 2,
                   .updates = {{field(RegisterStorage::lc, word_mask, 2, 1),
                                field(RegisterStorage::pc, word_mask, boot + 2, boot + 1)}}};
    state.lc = last ? old_lc : 1;
    if (!last) { state.pc = boot + 1; }
    publish(core, tail, state);
    retire(core);
    const auto fetch = core.instruction_fetch();
    const auto outcome = core.instruction_outcome();
    require(fetch.address == state.pc && fetch.latched_opcode.has_value() == !last &&
                (last || *fetch.latched_opcode == 0) && outcome && !outcome->control_transfer &&
                outcome->address == boot + 1 && outcome->opcode == 0 &&
                outcome->instructions == (last ? 3U : 2U),
            "Sparse hidden retirement retains the genuine REP opcode latch and completion");
  }
  require(core.progress().instructions == 3 && state.lc == old_lc && bus.calls == 0,
          "REP uses original saved LC and native repeated words without invented reads");
}

void reset(const Backend backend, const bool accepted) {
  Bus bus;
  bus.events = {
      {.kind = EffectKind::reset, .space = Space::program, .address = boot, .accept = accepted}};
  Core core(bus, {.backend = backend});
  auto state = staged_test::seed();
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = reset_opcode}, policy));
  const auto request = pending(core);
  require(request.effect.kind == EffectKind::reset && request.effect.value == reset_opcode,
          "Owned RESET retains its real decoded opcode before any peripheral reply");
  static_cast<void>(core.advance_instruction(request.ticket));
  require(bus.calls == 1 && bus.exact && bus.accepted == (accepted ? 1U : 0U),
          "RESET performs one explicitly configured callback with its actual acceptance");
  if (accepted) {
    ++state.pc;
    publish(core, pc(reset_opcode, boot, boot + 1), state);
    retire(core);
    require(core.registers() == state &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = reset_cycles},
            "Actual RESET acceptance permits its native PC and accounting result");
  } else {
    require(core.fault() == Fault{.kind = FaultKind::host_event,
                                  .space = Space::program,
                                  .address = boot,
                                  .value = reset_opcode,
                                  .pc = boot} &&
                core.registers() == state && core.progress() == Progress{} &&
                !core.pending_instruction() && !core.instruction_outcome(),
            "Real RESET refusal retains original native opcode/fault context without success");
  }
}

void reset_absent(const Backend backend) {
  std::array<Word, 1> program{reset_opcode};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
  auto state = staged_test::seed();
  state.pc = 0;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = state.pc, .opcode = reset_opcode}, policy));
  const auto request = pending(core);
  require(request.effect.kind == EffectKind::reset && request.effect.value == reset_opcode,
          "Absent RESET participant still has a genuine pending native request");
  static_cast<void>(core.advance_instruction(request.ticket));
  require(core.fault() == Fault{.kind = FaultKind::host_event,
                                .space = Space::program,
                                .address = state.pc,
                                .value = reset_opcode,
                                .pc = state.pc} &&
              core.registers() == state && core.progress() == Progress{} &&
              !core.pending_instruction(),
          "No participant is a real refusal, not a fabricated callback or acknowledgement");
}

void late_fault(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = staged_test::seed();
  state.a = limited_accumulator;
  state.sc = stack_entries - 1;
  core.set_registers(state);
  static_cast<void>(
      core.begin_instruction({.address = boot, .opcode = bit_call_a, .extension = target}, policy));
  const UnitWriteback source{
      .unit = RegisterUnit::bit,
      .opcode = bit_call_a,
      .count = 1,
      .updates = {{field(RegisterStorage::sr, flag_s | flag_l, 0, flag_s | flag_l)}}};
  state.sr = flag_s | flag_l;
  unit_publication_test::publish(core, source, state);
  require(core.fault() == Fault{.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = boot,
                                .pc = boot} &&
              !core.pending_instruction() && core.registers() == state && bus.calls == 0,
          "A later genuine stack refusal preserves native source flags without a fake PCU result");
}
}
} // namespace dsp56300::test::pcu_publication_test

int main() {
  try {
    using namespace dsp56300::test::pcu_publication_test;
    for (const auto from : backends) {
      nop(from);
      rti(from);
      repeat(from);
      late_fault(from);
      reset(from, true);
      reset(from, false);
      reset_absent(from);

      for (unsigned cut = 0; cut < 3; ++cut) { call_restore(from, cut); }
    }
    std::cout << dsp56300::test::checks << " native PCU publication assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
