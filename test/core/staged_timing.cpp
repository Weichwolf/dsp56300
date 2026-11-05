// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::staged_test {
namespace {
constexpr Word repeat_twenty_three = 0x0617a0, move_immediate = 0x44f400;
constexpr Word absolute_jump = 0x0af080, literal = 0x123456;
constexpr unsigned maximum_effects = 32;
constexpr unsigned repeat_count = 23;
constexpr auto publication = InstructionPublication::pcu_updates;

struct Case {
  InstructionWords words;
  std::uint64_t cycles;
};

// FM table A-1/A-4/A-7: NOP, REP immediate, long MOVE and long absolute JMP.
constexpr std::array cases{
    Case{.words = {.address = boot, .opcode = 0}, .cycles = 1},
    Case{.words = {.address = boot, .opcode = repeat_twenty_three}, .cycles = 5},
    Case{.words = {.address = boot, .opcode = move_immediate, .extension = literal}, .cycles = 2},
    Case{.words = {.address = boot, .opcode = absolute_jump, .extension = data_address},
         .cycles = 4}};

PendingInstruction reach_retirement(Core &core, const EffectTicket &earlier) {
  for (unsigned index = 0; index < maximum_effects; ++index) {
    const auto next = pending(core);
    if (next.effect.kind == EffectKind::retire) { return next; }
    rejected([&] { static_cast<void>(core.retirement_delta(next.ticket)); });
    require(core.advance_instruction(next.ticket).fault.kind == FaultKind::none,
            "Actual authored timing observation remains healthy");
    rejected([&] { static_cast<void>(core.retirement_delta(earlier)); });
  }
  throw std::runtime_error("Authored timing fixture exceeded its effect bound");
}

void observe(Core &core, Bus &bus, const PendingInstruction &next, const std::uint64_t cycles) {
  const auto before = core.registers();
  const auto progress = core.progress();
  const auto calls = bus.calls;
  const auto expected = Progress{.instructions = 1, .estimated_cycles = cycles};
  require(core.retirement_delta(next.ticket) == expected &&
              core.retirement_delta(next.ticket) == expected && core.registers() == before &&
              core.progress() == progress && bus.calls == calls &&
              pending(core).effect == next.effect && !core.instruction_outcome(),
          "Repeated timing observation has independent native units without publication or I/O");
}

void timing_case(const Backend backend, const Case sample) {
  Bus bus;
  Core core(bus, {.backend = backend});
  core.set_registers(seed());
  static_cast<void>(core.begin_instruction(sample.words, publication));
  const auto first = pending(core);
  const auto next = reach_retirement(core, first.ticket);
  observe(core, bus, next, sample.cycles);
  Bus other_bus;
  Core other(other_bus, {.backend = backend});
  other.set_registers(seed());
  static_cast<void>(other.begin_instruction(sample.words, publication));
  static_cast<void>(reach_retirement(other, pending(other).ticket));
  rejected([&] { static_cast<void>(other.retirement_delta(next.ticket)); });
  const auto before = core.progress();
  retire(core);
  require(core.progress() == Progress{.instructions = before.instructions + 1,
                                      .estimated_cycles = before.estimated_cycles + sample.cycles},
          "Genuine later retirement commits the independently specified observed increment");
  rejected([&] { static_cast<void>(core.retirement_delta(next.ticket)); });
}

void input_and_fault(const Backend backend, const bool accept) {
  Bus bus;
  bus.events = {{.accept = accept}};
  Core core(bus, {.backend = backend});
  core.set_registers(seed());
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = read_x0}, publication));
  const auto first = pending(core);
  rejected([&] { static_cast<void>(core.retirement_delta(first.ticket)); });
  auto input = first;
  for (unsigned index = 0; input.effect.kind != EffectKind::read && index < maximum_effects;
       ++index) {
    require(core.advance_instruction(input.ticket).fault.kind == FaultKind::none && bus.calls == 0,
            "Actual AGU publication can precede the operand without inventing I/O");
    input = pending(core);
  }
  require(input.effect.kind == EffectKind::read && bus.calls == 0,
          "The genuine operand read remains pending before its response");
  rejected([&] { static_cast<void>(core.retirement_delta(input.ticket)); });
  if (!accept) {
    const auto result = core.advance_instruction(input.ticket);
    require(result.fault.kind == FaultKind::memory_read && bus.calls == 1 && bus.accepted == 0,
            "An actual rejected read has no retirement timing receipt");
    rejected([&] { static_cast<void>(core.retirement_delta(first.ticket)); });
    return;
  }
  const auto next = reach_retirement(core, first.ticket);
  require(bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "Timing observation follows exactly one actual accepted operand read");
  observe(core, bus, next, 1);
  retire(core);
}

void wrong_publication(const Backend backend) {
  constexpr std::array others{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates};
  for (const auto other : others) {
    Bus bus;
    Core core(bus, {.backend = backend});
    core.set_registers(seed());
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = 0}, other));
    const auto next = reach_retirement(core, pending(core).ticket);
    rejected([&] { static_cast<void>(core.retirement_delta(next.ticket)); });
    retire(core);
  }
}

void repetition_timing(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  core.set_registers(seed());
  static_cast<void>(
      core.begin_instruction({.address = boot, .opcode = repeat_twenty_three}, publication));
  const auto setup = reach_retirement(core, pending(core).ticket);
  observe(core, bus, setup, 5);
  retire(core);
  for (unsigned index = 0; index < repeat_count; ++index) {
    require(core.instruction_fetch().address == boot + 1,
            "All twenty-three actual REP bodies retain the same independent address");
    static_cast<void>(core.begin_instruction({.address = boot + 1, .opcode = 0}, publication));
    const auto body = reach_retirement(core, pending(core).ticket);
    observe(core, bus, body, 1);
    retire(core);
    require(core.progress() == Progress{.instructions = index + 2U, .estimated_cycles = index + 6U},
            "REP setup costs five native units and each real NOP body adds exactly one");
  }
  require(core.instruction_fetch().address == boot + 2 && bus.calls == 0,
          "The actual repeated body falls through after exactly twenty-three executions");
}
}

void timing() {
  for (const auto backend : backends) {
    for (const auto sample : cases) { timing_case(backend, sample); }
    input_and_fault(backend, true);
    input_and_fault(backend, false);
    wrong_publication(backend);
    repetition_timing(backend);
  }
}
}
