// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <initializer_list>

namespace dsp56300::test::external_run {
namespace {
void peripheral_word_case(const Backend backend,
                          const Space space,
                          const bool write,
                          const bool refuse) {
  constexpr Word y_space = Word{1} << 19;
  constexpr Word raw_value = 0xab123456;
  Bus bus;
  const auto opcode = (write ? store_x0 : load_x0) | (space == Space::y ? y_space : 0U);
  bus.program(0, {opcode, port});
  bus.memory[{space, port}] = raw_value;
  if (refuse && write) { bus.refuse_write = Bus::Key{space, port}; }
  if (refuse && !write) { bus.refuse_read = Bus::Key{space, port}; }
  Core core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.pc = 0;
  expected.x = other;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc = 2;
  if (!write && !refuse) { expected.x = datum; }
  require(core.registers() == expected && result.dispatches == (refuse ? 0U : 1U) &&
              result.progress.instructions == result.dispatches,
          "External X/Y port word transfers preserve literal value/mask/PC/retirement state");
  const auto operation = write ? Operation::write : Operation::read;
  const auto value = write ? other : 0U;
  require(bus.events.size() == 3 &&
              bus.events.back() ==
                  Event{.operation = operation, .space = space, .address = port, .value = value} &&
              bus.memory.at({space, port}) == (write && !refuse ? other : raw_value),
          "Each direct high-port transfer invokes one actual masked host operation");
  if (refuse) {
    require(result.fault == Fault{.kind = write ? FaultKind::io_write : FaultKind::io_read,
                                  .space = space,
                                  .address = port,
                                  .value = value},
            "Direct X/Y port refusals retain the actual read/write operation and payload");
  } else {
    require(result.fault.kind == FaultKind::none, "An accepted port operation remains healthy");
  }
}

void peripheral_word_cases(const Backend backend) {
  for (const auto space : {Space::x, Space::y}) {
    for (const bool write : {false, true}) {
      for (const bool refuse : {false, true}) {
        peripheral_word_case(backend, space, write, refuse);
      }
    }
  }
}

void buffered_read_refusal(const Backend backend) {
  std::array<Word, 2> program{load_x0, address};
  std::array<Word, 1> data{};
  Core core({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
  auto expected = initial_state();
  expected.pc = 0;
  expected.x = datum;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc = 2;
  require(
      result.dispatches == 0 && result.progress == Progress{} && core.registers() == expected &&
          result.fault ==
              Fault{.kind = FaultKind::memory_read, .space = Space::x, .address = address},
      "An unmapped buffer-backed MOVE preserves its independent destination and native PC prefix");
  const auto blocked = core.run(1);
  require(blocked.dispatches == 0 && blocked.fault == result.fault && core.registers() == expected,
          "The corresponding buffer-backed fault also blocks replay and retirement");
}

void mixed_ownership(const Backend backend) {
  Bus bus;
  bus.program(origin, {0, load_post, 0, store_post, 0});
  bus.memory[{Space::x, address}] = datum;
  Core core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.r[0] = address;
  core.set_registers(expected);
  require(core.run(1).dispatches == 1,
          "External ordinary execution reaches a genuine staged boundary");
  ++expected.pc;
  auto progress = core.begin_fetch();
  while (progress.pending && progress.pending->effect.kind != EffectKind::read) {
    progress = core.advance_instruction(progress.pending->ticket);
  }
  require(progress.pending.has_value() && core.registers() == expected,
          "A synchronous successor can retain an actual pending staged RAM read");
  staged_test::rejected([&] { static_cast<void>(core.run(1)); });
  auto step = core.advance_instruction(staged_test::pending(core).ticket);
  require(step.pending.has_value() && !step.retired,
          "Accepted staged read awaits native retirement");
  staged_test::retire(core);
  auto after = expected;
  ++after.pc;
  ++after.r[0];
  after.x = datum;
  require(core.registers() == after && core.progress().instructions == 2,
          "Mixed owners retain every literal native register and retirement");
  require(core.run(3).dispatches == 3,
          "An ordinary suffix resumes only after actual staged retirement");
  after.pc += 3;
  ++after.r[0];
  require(core.registers() == after && core.progress().instructions == 5 && bus.writes == 1 &&
              bus.memory.at({Space::x, address + 1}) == datum,
          "A mixed synchronous/staged suffix performs its one actual independent RAM "
          "store");
}

void repeated_opcode(const Backend backend) {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_x0 = 0x200040;
  Bus bus;
  bus.program(origin, {repeat_three, add_x0, 0});
  Core core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.x = 1;
  core.set_registers(expected);
  const auto prefix = core.run(2);
  require(prefix.dispatches == 2 && prefix.fault.kind == FaultKind::none && bus.reads == 2,
          "External REP fetches the actual body once and retires the first arithmetic operation");
  bus.memory[{Space::program, origin + 1}] = 0; // Mutated RAM cannot replace a genuine REP latch.
  const auto suffix = core.run(2);
  expected.pc += 2;
  expected.a = std::uint64_t{3} << word_bits;
  expected.sr = flag_u;
  require(
      suffix.dispatches == 2 && suffix.fault.kind == FaultKind::none &&
          core.registers() == expected && core.progress().instructions == 4 && bus.reads == 2,
      "Remaining external REP iterations preserve the literal original opcode and complete state");
}

void vector_pair(const Backend backend) {
  constexpr Word vector = 0x40;
  constexpr Word second = 0x320700;
  constexpr Word changed = 0x320900;
  constexpr Word program_r0 = 0x076084;
  Bus bus;
  bus.program(origin, {0});
  bus.program(vector, {program_r0, second});
  Core core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.x = changed;
  expected.r[0] = vector + 1;
  core.set_registers(expected);
  core.request_interrupt(vector, 1, {.address = origin, .opcode = 0});
  const auto prefix = core.run(1);
  require(
      prefix.dispatches == 1 && prefix.fault.kind == FaultKind::none && bus.reads == 2 &&
          bus.writes == 1 && bus.memory.at({Space::program, vector + 1}) == changed,
      "External fast interrupt captures both real vector words before its first slot modifies RAM");
  const auto suffix = core.run(1);
  expected.r[2] = 7;
  require(suffix.dispatches == 1 && suffix.fault.kind == FaultKind::none &&
              core.registers() == expected && bus.reads == 2,
          "The original second vector slot executes from its genuine pair latch and resumes "
          "foreground");
}
}

void continuation_checks() {
  for (const auto backend : backends) {
    peripheral_word_cases(backend);
    buffered_read_refusal(backend);
    mixed_ownership(backend);
    repeated_opcode(backend);
    vector_pair(backend);
  }
}
}
