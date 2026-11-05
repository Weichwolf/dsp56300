// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

namespace dsp56300::test::staged_test {
namespace {
constexpr Word y_address = 0x240, y_initial = 2;
constexpr Word pair_stores = 0xb01800, load_x = 0x8000;
constexpr Word reset_opcode = 0x84, reset_cycles = 7;

void rejected_second(const Backend backend, const bool read_first) {
  Bus bus;
  bus.events = {{.kind = read_first ? EffectKind::read : EffectKind::write,
                 .value = read_first ? raw_datum : 1U},
                {.kind = EffectKind::write,
                 .space = Space::y,
                 .address = y_address,
                 .value = y_initial,
                 .accept = false}};
  Core core(bus, {.backend = backend});
  auto initial = seed();
  initial.y = y_initial;
  initial.r[4] = y_address;
  core.set_registers(initial);
  const auto opcode = pair_stores | (read_first ? load_x : 0);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}));
  const auto first = pending(core);
  static_cast<void>(core.advance_instruction(first.ticket));
  const auto second = pending(core);
  require(bus.calls == 1 && bus.accepted == 1 && bus.exact &&
              bus.x_written == (read_first ? 0U : 1U) && bus.y_written == 0,
          "The accepted first effect has a real persistent external prefix");
  const auto failed = core.advance_instruction(second.ticket);
  require(failed.fault.kind == FaultKind::memory_write && failed.fault.space == Space::y &&
              failed.fault.address == y_address && failed.fault.pc == boot &&
              failed.fault.value == y_initial && !failed.pending && core.registers() == initial &&
              core.progress() == Progress{} && bus.calls == 2 && bus.accepted == 1 && bus.exact &&
              bus.y_written == 0,
          "A rejected second effect reports its exact context without retiring or undoing I/O");
  rejected([&] { static_cast<void>(core.advance_instruction(first.ticket)); });
  rejected([&] { static_cast<void>(core.advance_instruction(second.ticket)); });
  rejected([&] { static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode})); });
  rejected([&] { static_cast<void>(core.advance_instruction(second.ticket)); });
  require(core.fault() == failed.fault && core.registers() == initial &&
              core.progress() == Progress{} && !core.pending_instruction() && bus.calls == 2 &&
              bus.accepted == 1 && bus.x_written == (read_first ? 0U : 1U) && bus.y_written == 0,
          "Fault latching and stale owners never replay or roll back accepted external effects");
}

void reset_case(const Backend backend, const bool accepted) {
  Bus bus;
  bus.events = {
      {.kind = EffectKind::reset, .space = Space::program, .address = boot, .accept = accepted}};
  Core core(bus, {.backend = backend});
  const auto initial = seed();
  core.set_registers(initial);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = reset_opcode}));
  const auto action = pending(core);
  require(action.effect.kind == EffectKind::reset && action.effect.pc == boot &&
              action.effect.address == boot && bus.calls == 0 && core.registers() == initial &&
              core.progress() == Progress{},
          "RESET calculation never fabricates or performs a peripheral acknowledgement");
  const auto result = core.advance_instruction(action.ticket);
  require(bus.calls == 1 && bus.accepted == (accepted ? 1U : 0U) && bus.exact &&
              core.registers() == initial && core.progress() == Progress{},
          "Only advancing the stored RESET owner invokes one actual callback");
  if (!accepted) {
    require(result.fault.kind == FaultKind::host_event && result.fault.pc == boot &&
                result.fault.address == boot && !result.pending,
            "Rejected RESET stays visibly faulted without successful retirement");
    rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
    core.reset();
    rejected([&] { static_cast<void>(core.advance_instruction(action.ticket)); });
    require(bus.calls == 1, "CPU reset cancels continuation without repeating a device action");
    return;
  }
  auto expected = initial;
  ++expected.pc;
  retire(core);
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = reset_cycles},
          "Actual RESET retirement preserves the CPU and its independently specified base cost");
  require(bus.calls == 1 && bus.accepted == 1,
          "Actual RESET retirement does not repeat the accepted peripheral callback");
}
} // namespace

void events() {
  for (const auto backend : backends) {
    for (const bool choice : {false, true}) {
      rejected_second(backend, choice);
      reset_case(backend, choice);
    }
  }
}
} // namespace dsp56300::test::staged_test
