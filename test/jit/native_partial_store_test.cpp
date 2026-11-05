// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_partial_store_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::native_partial {
namespace {
constexpr std::array backends{
    Backend::reference, Backend::cached, Backend::jit, Backend::automatic};

void check_store(const Store input) {
  std::array<Word, 2> program{opcode(input), input.mode == 6 ? address : 0U};
  for (const auto backend : backends) {
    std::array<Word, memory_words> x{};
    std::array<Word, memory_words> y{};
    x.fill(sentinel);
    y.fill(sentinel);
    auto expected_x = x;
    auto expected_y = y;
    const auto before = seed(input);
    (input.space == Space::x ? expected_x : expected_y).at(effective_address(input)) =
        partial_word(before, input.selector);
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    cpu.set_registers(before);
    const auto run = cpu.run(1);
    const auto cycles = input.mode == 5 || input.mode == 6 || input.mode == 7 ? 2U : 1U;
    require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                cpu.registers() == after(input) && x == expected_x && y == expected_y &&
                cpu.progress() == Progress{.instructions = 1, .estimated_cycles = cycles},
            "Independent partial-field value, unscaled flags, full RAM, AGU and progress");
    if (backend == Backend::jit || backend == Backend::automatic) {
      const auto path = cpu.jit_statistics();
      require(path.native_move_instructions == 1 && path.native_alu_instructions == 1 &&
                  path.helper_instructions == 0,
              "Ordinary partial stores execute actual native code, not a helper wrapper");
    }
  }
}

void operand_matrix() {
  for (const auto selector : selectors) {
    for (const auto space : {Space::x, Space::y}) {
      for (unsigned mode = 0; mode < 8; ++mode) {
        for (unsigned index = 0; index < 8; ++index) {
          if (mode == 6 && index != 0) { continue; }
          check_store({.selector = selector, .mode = mode, .index = index, .space = space});
        }
      }
    }
  }
}

void numerical_matrix() {
  for (const auto selector : selectors) {
    for (const auto space : {Space::x, Space::y}) {
      for (const auto value : values) {
        for (const auto mode : modes) {
          for (const Word flags : {masked_status, masked_status | status_byte}) {
            check_store({.selector = selector, .space = space, .status = mode | flags, .a = value});
          }
        }
      }
    }
  }
}

class Port final : public Io {
public:
  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool
  write(const Space space, const Word target, const Word pc, const Word value) noexcept override {
    ++writes;
    actual = {.space = space, .address = target, .pc = pc, .value = value};
    return accept;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  unsigned writes = 0;

  struct Access {
    Space space = Space::program;
    Word address = 0, pc = 1, value = 0;
  };

  Access actual;
  bool accept = true;
};

void io_boundary(const unsigned selector, const bool accepted) {
  constexpr Word port_address = 0xffff80;
  const Store input{.selector = selector, .mode = 3};
  std::array<Word, 1> program{opcode(input)};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Port port;
  port.accept = accepted;
  Core cpu({.program = program, .x = x, .y = y}, &port, {.backend = Backend::jit});
  auto before = seed(input);
  before.r[0] = port_address;
  cpu.set_registers(before);
  auto expected = before;
  expected.pc = 1;
  ++expected.r[0];
  const auto run = cpu.run(1);
  const auto instructions = accepted ? 1U : 0U;
  require(
      run.dispatches == instructions && cpu.registers() == expected &&
          cpu.progress() ==
              Progress{.instructions = instructions, .estimated_cycles = instructions},
      "A peripheral side exit preserves exactly one AGU update and the real partial fault state");
  require(port.writes == 1 && port.actual.space == Space::x &&
              port.actual.address == port_address && port.actual.pc == 0 &&
              port.actual.value == partial_word(before, selector) && x[0] == 0 && y[0] == 0,
          "The native RAM guard never bypasses or duplicates the genuine I/O callback");
  require(run.fault.kind == (accepted ? FaultKind::none : FaultKind::io_write),
          "I/O refusal remains visible");
}

void stream_cases() {
  for (const auto domain : domains) {
    for (const bool partial : {false, true}) {
      const auto result = workload(domain, partial, 1);
      require(result.instructions != 0 && result.native == result.instructions &&
                  result.moves == result.instructions / 2 && result.helpers == 0,
              "Shared independent three-domain streams prove actual native ALU and stores");
    }
  }
}

void repeated_store(const unsigned selector) {
  constexpr Word repeat_four = 0x0604a0;
  constexpr Word saved_lc = 0x357;
  const Store input{.selector = selector, .mode = 3};
  std::array<Word, 2> program{repeat_four, opcode(input)};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  x.fill(sentinel);
  auto expected_x = x;
  auto before = seed(input);
  before.lc = saved_lc;
  for (unsigned index = 0; index < 4; ++index) {
    expected_x.at(address + index) = partial_word(before, selector);
  }
  Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  cpu.set_registers(before);
  require(cpu.run(1).dispatches == 1 && cpu.progress().estimated_cycles == 5,
          "Partial-store REP has its real five-cycle setup (FM Table A-1)");
  const auto counters = cpu.jit_statistics();
  auto expected = before;
  expected.pc = 2;
  expected.r[0] += 4;
  const auto run = cpu.run(4);
  require(run.dispatches == 4 && run.fault.kind == FaultKind::none && cpu.registers() == expected &&
              x == expected_x && y == std::array<Word, memory_words>{} &&
              cpu.progress() == Progress{.instructions = 5, .estimated_cycles = 9},
          "Retained partial-store bodies preserve LC, all registers, exact RAM and progress");
  const auto current = cpu.jit_statistics();
  require(current.native_move_instructions == counters.native_move_instructions + 4 &&
              current.helper_instructions == counters.helper_instructions,
          "Every eligible repeated partial store executes emitted code after setup");
}

void program_alias() {
  constexpr Word clear_a = 0x200013;
  constexpr Word clear_status = 0x314;
  constexpr std::uint64_t source = 0x00123456200013;
  const Store input{.selector = 8, .a = source};
  std::array<Word, 3> program{opcode(input)};
  std::array<Word, 1> y{};
  Core cpu({.program = program, .x = program, .y = y}, nullptr, {.backend = Backend::jit});
  auto before = seed(input);
  before.r[0] = 1;
  cpu.set_registers(before);
  auto expected = before;
  expected.pc = 2;
  expected.a = 0;
  expected.sr = clear_status;
  const auto run = cpu.run(2);
  require(run.dispatches == 2 && run.fault.kind == FaultKind::none && cpu.registers() == expected &&
              program == std::array<Word, 3>{opcode(input), clear_a} && y[0] == 0 &&
              cpu.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
          "A partial store through program-aliasing RAM invalidates the next decoded instruction");
  require(cpu.jit_statistics().invalidations != 0,
          "Self-modification retains actual code invalidation");
}

void parallel_old_value(const unsigned selector) {
  constexpr Word add = 0x200040;
  constexpr Word mac = 0x2000d2;
  constexpr Word mac_alu = 0xd2;
  constexpr std::uint64_t before_add = 0x7fffffffffffff - (std::uint64_t{0x010203} << 24);
  const Store input{.selector = selector, .a = before_add};
  const std::array<Word, 2> words{add, opcode(input) | mac_alu};
  for (const auto backend : backends) {
    auto program = words;
    std::array<Word, memory_words> x{};
    std::array<Word, memory_words> y{};
    auto expected_x = x;
    const auto before = seed(input);
    auto expected = before;
    sequence_model::alu(expected, before, add);
    expected_x.at(address) = partial_word(expected, selector);
    const auto old_inputs = expected;
    sequence_model::alu(expected, old_inputs, mac);
    expected.pc = 2;
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    cpu.set_registers(before);
    const auto run = cpu.run(2);
    require(run.dispatches == 2 && run.fault.kind == FaultKind::none &&
                cpu.registers() == expected && x == expected_x &&
                y == std::array<Word, memory_words>{} &&
                cpu.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
            "A parallel store samples dirty cached old fields before the overflowing MAC");
    if (backend == Backend::automatic || backend == Backend::jit) {
      const auto path = cpu.jit_statistics();
      require(path.native_move_instructions == 1 && path.native_alu_instructions == 2 &&
                  path.helper_instructions == 0,
              "Parallel partial fields and their arithmetic run without general helpers");
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test::native_partial;
    operand_matrix();
    numerical_matrix();
    for (const auto selector : selectors) {
      io_boundary(selector, false);
      io_boundary(selector, true);
      repeated_store(selector);
      parallel_old_value(selector);
    }
    program_alias();
    stream_cases();
    std::cout << "PASS native partial stores: " << dsp56300::test::checks << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
