// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using dsp56300::Word;
constexpr std::size_t memory_words = 1024;
constexpr Word start = 0x100;
constexpr unsigned word_bits = 24;
constexpr Word parallel_none = 0x200000;
constexpr Word mac_operation = 0xd2;
constexpr Word mac_x0_y0_a = parallel_none | mac_operation;
constexpr Word test_word = 0x123456;
constexpr std::uint64_t accumulator_initial = 0x00100000000000;
constexpr std::uint64_t accumulator_mac = 0x001296cd9619c8;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

struct Fixture {
  explicit Fixture(const dsp56300::Backend backend)
      : core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend}) {
    auto state = core.registers();
    state.pc = start;
    core.set_registers(state);
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  dsp56300::Core core;
};

void check_mac(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  fixture.core.write(dsp56300::Space::program, start, mac_x0_y0_a);
  auto state = fixture.core.registers();
  state.x = test_word;
  state.y = test_word;
  state.a = accumulator_initial;
  fixture.core.set_registers(state);
  const auto result = fixture.core.run(1);
  require(result.fault.kind == dsp56300::FaultKind::none, "MAC executes");
  require(fixture.core.registers().a == accumulator_mac, "Manual fractional MAC example");
}

void check_parallel_latch(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  // Appendix A: register move eeeee=A=14, ddddd=X1=5; MAC low byte 0xd2.
  constexpr Word move_a_x1 = parallel_none | (14U << 13) | (5U << 8);
  fixture.core.write(dsp56300::Space::program, start, move_a_x1 | mac_operation);
  auto state = fixture.core.registers();
  state.x = test_word;
  state.y = test_word;
  state.a = accumulator_initial;
  fixture.core.set_registers(state);
  const auto result = fixture.core.run(1);
  require(result.fault.kind == dsp56300::FaultKind::none, "Parallel MAC executes");
  const auto actual = fixture.core.registers();
  require(actual.a == accumulator_mac && actual.x >> word_bits == 0x100000,
          "Parallel move reads old A and ALU reads old X/Y");
}

void check_loop(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  constexpr Word repeat_four = 0x0604a0;
  constexpr std::array program{repeat_four, mac_x0_y0_a};
  fixture.core.load(dsp56300::Space::program, start, program);
  auto state = fixture.core.registers();
  state.x = 1;
  state.y = 1;
  state.lc = 7;
  fixture.core.set_registers(state);
  require(fixture.core.run(5).fault.kind == dsp56300::FaultKind::none, "REP executes");
  const auto actual = fixture.core.registers();
  require(actual.a == 8 && actual.lc == 7 && actual.pc == start + 2, "REP count and LC restore");
  require(fixture.core.progress().estimated_cycles == 9,
          "REP base cost five plus four single-cycle MAC instructions");
}

void check_fail_stop(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  // WAIT is a real architectural instruction, currently an explicit ISA gap.
  constexpr Word unimplemented_wait = 0x000086;
  fixture.core.write(dsp56300::Space::program, start, unimplemented_wait);
  const auto result = fixture.core.run(1);
  require(result.dispatches == 0 &&
              result.fault.kind == dsp56300::FaultKind::unsupported_instruction &&
              result.fault.pc == start && result.fault.value == unimplemented_wait,
          "An unimplemented architectural instruction reports explicit context");
  require(fixture.core.run(1).dispatches == 0, "Failed core does not resume");
}

void check_repeat_latch(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  constexpr Word repeat_four = 0x0604a0;
  constexpr Word repeat_two = 0x0602a0;
  constexpr std::array program{repeat_four, mac_x0_y0_a, Word{0}, Word{0}};
  fixture.core.load(dsp56300::Space::program, start, program);
  auto state = fixture.core.registers();
  state.x = 1;
  state.y = 1;
  fixture.core.set_registers(state);
  require(fixture.core.run(2).dispatches == 2, "REP and first iteration retire");
  require(!fixture.core.interruptible(), "Interrupts wait for REP completion");
  fixture.core.write(dsp56300::Space::program, start + 1, 0);
  require(fixture.core.run(3).dispatches == 3 && fixture.core.registers().a == 8,
          "REP executes its latched opcode after a host program-memory change");
  require(fixture.core.interruptible(), "Interrupts are admitted before a non-REP successor");
  fixture.core.write(dsp56300::Space::program, start + 2, repeat_two);
  require(!fixture.core.interruptible(), "Adjacent REP instructions form one interrupt exclusion");
  require(fixture.core.run(3).dispatches == 3 && fixture.core.interruptible(),
          "Interrupt exclusion ends after the final adjacent REP body");
}

void check_repeat_restriction(const dsp56300::Backend backend) {
  Fixture fixture(backend);
  constexpr Word repeat_two = 0x0602a0;
  constexpr Word jump = 0x0c0100;
  constexpr std::array program{repeat_two, jump};
  fixture.core.load(dsp56300::Space::program, start, program);
  const auto result = fixture.core.run(2);
  require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::invalid_instruction &&
              result.fault.pc == start + 1,
          "REP rejects a control-flow instruction instead of repeating it");
}
}

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      check_mac(backend);
      check_parallel_latch(backend);
      check_loop(backend);
      check_fail_stop(backend);
      check_repeat_latch(backend);
      check_repeat_restriction(backend);
    }
    std::cout << "Independent DSP instruction checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
