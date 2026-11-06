// SPDX-License-Identifier: GPL-3.0-only
#include "backend_support.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
static_assert(CoreImplementation<BackendCore<Backend::automatic>>);
constexpr Word immediate_x0 = 0x44f400, payload = 0x923456, replacement = 0x123456;
constexpr Word move_port_x0 = 0x044480, port_address = 0xffff80;
constexpr Word repeat_three = 0x0603a0, clear_a = 0x200013;
constexpr std::uint64_t high_x = 0x654321000000;
constexpr std::array backends{
    Backend::reference, Backend::cached, Backend::jit, Backend::automatic};

struct LocalRam {
  std::array<Word, 1> x{}, y{};
};

class Port final : public Io {
public:
  bool accept = true;
  unsigned effects = 0;
  Space last_space = Space::program;
  Word last_address = 0, last_pc = 0, last_value = 0;
  bool last_write = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    observe({.space = space, .address = address, .pc = pc, .value = payload});
    value = payload;
    return accept && space == Space::x && address == port_address;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    observe({.space = space, .address = address, .pc = pc, .value = value, .write = true});
    return accept && space == Space::x && address == port_address;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

private:
  struct Access {
    Space space;
    Word address, pc, value;
    bool write = false;
  };

  void observe(const Access event) noexcept {
    ++effects;
    last_space = event.space;
    last_address = event.address;
    last_pc = event.pc;
    last_write = event.write;
    last_value = event.value;
  }
};

void immutable_capture() {
  for (const auto backend : backends) {
    std::array<Word, 4> program{};
    LocalRam ram;
    Core cpu({.program = program, .x = ram.x, .y = ram.y}, nullptr, {.backend = backend});
    Registers initial;
    initial.x = high_x | 1;
    cpu.set_registers(initial);
    for (const Word value : {payload, replacement}) {
      cpu.set_registers(initial);
      const auto actual =
          cpu.run_instruction({.address = 0, .opcode = immediate_x0, .extension = value});
      auto expected = initial;
      expected.x = high_x | value;
      expected.pc = 2;
      const std::uint64_t instructions = value == payload ? 1U : 2U;
      require(actual.dispatches == 1 && actual.fault.kind == FaultKind::none &&
                  cpu.registers() == expected &&
                  actual.progress ==
                      Progress{.instructions = instructions, .estimated_cycles = instructions * 2},
              "Captured two-word load preserves high X and executes its own literal despite "
              "backing NOPs");
    }
    if (backend == Backend::jit || backend == Backend::automatic) {
      require(cpu.jit_statistics().native_move_instructions == 2 &&
                  cpu.jit_statistics().helper_instructions == 0,
              "Changed captured literals use real native transfer lowering");
    }
  }
}

void native_ram_capture() {
  constexpr Word load_long_x = 0x42d800;
  constexpr Word address = 8;
  constexpr std::uint64_t raw = 0x923456abcdef;
  for (const auto backend : backends) {
    std::array<Word, 2> program{};
    std::array<Word, 16> x{};
    std::array<Word, 16> y{};
    x[address] = payload;
    y[address] = static_cast<Word>(raw & word_mask);
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    Registers initial;
    initial.r[0] = address;
    initial.m.fill(word_mask);
    initial.sr = status_byte;
    cpu.set_registers(initial);
    const auto actual = cpu.run_instruction({.address = 0, .opcode = load_long_x});
    auto expected = initial;
    expected.x = raw;
    expected.pc = 1;
    ++expected.r[0];
    require(actual.dispatches == 1 && actual.fault.kind == FaultKind::none &&
                cpu.registers() == expected &&
                actual.progress == Progress{.instructions = 1, .estimated_cycles = 1},
            "Captured native long RAM read retains full flags and exactly one AGU update");
    if (backend == Backend::jit || backend == Backend::automatic) {
      require(cpu.jit_statistics().native_move_instructions == 1 &&
                  cpu.jit_statistics().helper_instructions == 0,
              "Captured RAM work uses emitted moves without staged replay or interpretation");
    }
  }
}

struct IoInput {
  Backend backend;
  bool write, accept, external;
};

Core make_io_cpu(Port &port, std::array<Word, 2> &program, LocalRam &ram, const IoInput input) {
  if (input.external) { return Core(port, {.backend = input.backend}); }
  return Core({.program = program, .x = ram.x, .y = ram.y}, &port, {.backend = input.backend});
}

void real_io_case(const IoInput input) {
  constexpr Word direction = 0x8000;
  std::array<Word, 2> program{};
  LocalRam ram;
  Port port;
  port.accept = input.accept;
  auto cpu = make_io_cpu(port, program, ram, input);
  Registers initial;
  initial.x = high_x | (input.write ? payload : 1U);
  cpu.set_registers(initial);
  const auto actual =
      cpu.run_instruction({.address = 0, .opcode = move_port_x0 | (input.write ? direction : 0U)});
  require(port.effects == 1 && port.last_space == Space::x && port.last_address == port_address &&
              port.last_pc == 0 && port.last_write == input.write && port.last_value == payload,
          "Captured register MOVEP performs the actual addressed callback exactly once");
  const auto failure_kind = input.write ? FaultKind::io_write : FaultKind::io_read;
  if (input.accept) {
    auto expected = initial;
    expected.x = high_x | payload;
    expected.pc = 1;
    require(actual.dispatches == 1 && actual.fault.kind == FaultKind::none &&
                cpu.registers() == expected &&
                actual.progress == Progress{.instructions = 1, .estimated_cycles = 1},
            "Actual successful Io preserves all independently expected CPU state");
  } else {
    require(actual.dispatches == 0 && actual.progress == Progress{} &&
                actual.fault == Fault{.kind = failure_kind,
                                      .space = Space::x,
                                      .address = port_address,
                                      .value = input.write ? payload : 0U,
                                      .pc = 0},
            "Refused Io exposes precise real fault and cannot retire a successful instruction");
  }
}

void real_io() {
  for (const auto backend : backends) {
    for (const bool write : {false, true}) {
      for (const bool accept : {false, true}) {
        real_io_case({.backend = backend, .write = write, .accept = accept, .external = false});
        real_io_case({.backend = backend, .write = write, .accept = accept, .external = true});
      }
    }
  }
}

void ownership_guards() {
  std::array<Word, 4> program{};
  LocalRam ram;
  Core cpu({.program = program, .x = ram.x, .y = ram.y});
  const auto before = cpu.registers();
  const std::array invalid{InstructionWords{.address = 1, .opcode = 0},
                           InstructionWords{.address = 0, .opcode = immediate_x0},
                           InstructionWords{.address = 0, .opcode = 0, .extension = 1},
                           InstructionWords{.address = 0, .opcode = word_mask + 1}};
  for (const auto words : invalid) {
    bool refused = false;
    try {
      static_cast<void>(cpu.run_instruction(words));
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused && cpu.registers() == before && cpu.progress() == Progress{},
            "Wrong PC, width or extension refuses before any CPU effect");
  }
}

void fetch_ownership() {
  constexpr Word vector = 0x20;
  constexpr Word cache_enable = 0x80000;
  std::array<Word, 4> program{};
  LocalRam ram;
  Core vector_cpu({.program = program, .x = ram.x, .y = ram.y});
  vector_cpu.set_registers({});
  vector_cpu.request_interrupt(vector, 0);
  const auto before = vector_cpu.registers();
  bool refused = false;
  try {
    static_cast<void>(vector_cpu.run_instruction({.address = 0, .opcode = 0}));
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused && vector_cpu.registers() == before && vector_cpu.progress() == Progress{} &&
              vector_cpu.instruction_fetch().exception_vector,
          "A captured ordinary instruction cannot bypass actual pending vector ownership");
  Core cache_cpu({.program = program, .x = ram.x, .y = ram.y},
                 nullptr,
                 {.cacheable_begin = 0, .cacheable_end = program.size()});
  Registers enabled;
  enabled.sr = cache_enable;
  cache_cpu.set_registers(enabled);
  refused = false;
  try {
    static_cast<void>(cache_cpu.run_instruction({.address = 0, .opcode = 0}));
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused && cache_cpu.registers() == enabled && cache_cpu.progress() == Progress{},
          "Architectural cache fetches retain their actual owner without a fabricated hit");
}

void loop_capture() {
  constexpr Word dor_three = 0x060390;
  constexpr Word end_displacement = 3;
  for (const auto backend : backends) {
    std::array<Word, 5> program{dor_three, end_displacement, 0, 0, 0};
    LocalRam ram;
    Core cpu({.program = program, .x = ram.x, .y = ram.y}, nullptr, {.backend = backend});
    require(cpu.run(1).dispatches == 1, "Actual DOR establishes a three-iteration two-NOP loop");
    for (unsigned iteration = 0; iteration < 3; ++iteration) {
      require(cpu.run_instruction({.address = 2, .opcode = 0}).dispatches == 1,
              "Captured loop entry retires only its single owned NOP");
      const auto end = cpu.run_instruction({.address = 3, .opcode = 0});
      require(end.dispatches == 1 && end.fault.kind == FaultKind::none &&
                  cpu.registers().pc == (iteration == 2 ? 4U : 2U),
              "The actual loop endpoint selects saved start or normal completion");
    }
    constexpr Word loop_flag = 1U << 15;
    require(
        cpu.progress() == Progress{.instructions = 7, .estimated_cycles = 11} &&
            cpu.registers().sp == 0 && (cpu.registers().sr & loop_flag) == 0,
        "Captured DO endpoints preserve independently expected counters and real stack cleanup");
  }
}

void repeat_capture() {
  for (const auto backend : backends) {
    std::array<Word, 4> program{repeat_three, 0, 0, 0};
    LocalRam ram;
    Core cpu({.program = program, .x = ram.x, .y = ram.y}, nullptr, {.backend = backend});
    require(cpu.run(1).dispatches == 1, "Actual REP establishes three body iterations");
    for (unsigned iteration = 0; iteration < 3; ++iteration) {
      const auto actual = cpu.run_instruction({.address = 1, .opcode = clear_a});
      require(actual.dispatches == 1 && actual.fault.kind == FaultKind::none &&
                  cpu.registers().pc == (iteration == 2 ? 2U : 1U),
              "Captured REP body retires exactly one real iteration and restores its PC");
    }
    require(cpu.registers().a == 0 && cpu.registers().lc == 0 &&
                cpu.progress() == Progress{.instructions = 4, .estimated_cycles = 8},
            "Three captured CLR iterations keep original REP cost and hidden completion");
  }
}
}
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    immutable_capture();
    native_ram_capture();
    real_io();
    ownership_guards();
    fetch_ownership();
    loop_capture();
    repeat_capture();
    std::cout << "PASS captured instructions: " << checks << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
