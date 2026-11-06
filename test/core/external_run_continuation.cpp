// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
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
  TestCore core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.pc = 0;
  expected.x = other;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc = 2;
  if (!write && !refuse) { expected.x = datum; }
  require(core.registers() == expected && result.retired == (refuse ? 0U : 1U) &&
              result.progress.instructions == result.retired,
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
  TestCore core({.program = program, .x = data, .y = data}, nullptr, {.backend = backend});
  auto expected = initial_state();
  expected.pc = 0;
  expected.x = datum;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc = 2;
  require(
      result.retired == 0 && result.progress == Progress{} && core.registers() == expected &&
          result.fault ==
              Fault{.kind = FaultKind::memory_read, .space = Space::x, .address = address},
      "An unmapped buffer-backed MOVE preserves its independent destination and native PC prefix");
  const auto blocked = core.run(1);
  require(blocked.retired == 0 && blocked.fault == result.fault && core.registers() == expected,
          "The corresponding buffer-backed fault also blocks replay and retirement");
}

void repeated_opcode(const Backend backend) {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_x0 = 0x200040;
  Bus bus;
  bus.program(origin, {repeat_three, add_x0, 0});
  TestCore core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.x = 1;
  core.set_registers(expected);
  const auto prefix = core.run(2);
  require(prefix.retired == 2 && prefix.fault.kind == FaultKind::none && bus.reads == 2,
          "External REP fetches the actual body once and retires the first arithmetic operation");
  bus.memory[{Space::program, origin + 1}] = 0; // Mutated RAM cannot replace a genuine REP latch.
  const auto suffix = core.run(2);
  expected.pc += 2;
  expected.a = std::uint64_t{3} << word_bits;
  expected.sr = flag_u;
  require(
      suffix.retired == 2 && suffix.fault.kind == FaultKind::none && core.registers() == expected &&
          core.progress().instructions == 4 && bus.reads == 2,
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
  TestCore core(bus, {.backend = backend});
  auto expected = initial_state();
  expected.x = changed;
  expected.r[0] = vector + 1;
  core.set_registers(expected);
  core.request_interrupt(vector, 1, {.address = origin, .opcode = 0});
  const auto prefix = core.run(1);
  require(
      prefix.retired == 1 && prefix.fault.kind == FaultKind::none && bus.reads == 2 &&
          bus.writes == 1 && bus.memory.at({Space::program, vector + 1}) == changed,
      "External fast interrupt captures both real vector words before its first slot modifies RAM");
  const auto suffix = core.run(1);
  expected.r[2] = 7;
  require(suffix.retired == 1 && suffix.fault.kind == FaultKind::none &&
              core.registers() == expected && bus.reads == 2,
          "The original second vector slot executes from its genuine pair latch and resumes "
          "foreground");
}
}

void continuation_checks() {
  for (const auto backend : backends) {
    peripheral_word_cases(backend);
    buffered_read_refusal(backend);

    repeated_opcode(backend);
    vector_pair(backend);
  }
}
}
