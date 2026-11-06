// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "return_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
using return_model::foreground;
using return_model::rti;
using return_model::rts;
constexpr Word vector_a = 32, vector_b = 34, failure_vector = 512;
constexpr Word service_a = 0x120, service_b = 0x160;
constexpr Word call_short = 0x0d0000, call_long = 0x0bf080, call_post = 0x0bd880;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, round_a = 0x200011;
constexpr Word invalid_modifier = 0x8000;
constexpr std::uint64_t half_seed = 0x1234567ffffe;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core;

  explicit Machine(const Backend backend)
      : core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend}) {}
};

void empty_returns(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  for (const Word operation : {rti, rts}) {
    for (Word pointer = 0; pointer < return_model::pointer_range; ++pointer) {
      for (const Word status : {0U, word_mask, mode_rm, mode_up | flag_c}) {
        core.reset();
        auto state = return_model::seed({.pointer = pointer, .count = 0});
        state.sr = status;
        core.write(Space::program, start, operation);
        core.set_registers(state);
        auto expected = state;
        ++expected.pc;
        const auto result = core.run(1);
        require(result.dispatches == 0 && result.progress == Progress{} &&
                    result.fault == Fault{.kind = FaultKind::stack,
                                          .address = start,
                                          .value = pointer,
                                          .pc = start} &&
                    core.registers() == expected && !core.interruptible(),
                "A failed RTI/RTS cannot fabricate a zero PC/SR from an empty stack");
        require(core.run(2).dispatches == 0 && core.registers() == expected &&
                    core.progress() == Progress{},
                "A failed return remains stopped without repeated pops or retirement");
      }
    }
  }
}

struct Resume {
  Word pointer = 0, status = 0;
};

void native_resume(const Backend backend, const Resume input) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array service{mac_a, mac_b, rti};
  const std::array continuation{adc_x_a, round_a};
  core.load(Space::program, start, service);
  core.load(Space::program, foreground, continuation);
  auto state = return_model::seed({.status = input.status, .pointer = input.pointer});
  state.sr = 0;
  state.x = 1;
  state.y = 1;
  state.a = half_seed;
  auto warm = state;
  warm.pc = foreground;
  core.set_registers(warm);
  require(core.run(2).dispatches == 2,
          "Warm foreground arithmetic before returning under a new mode");
  const auto progress = core.progress();
  const auto native = core.jit_statistics().native_alu_instructions;
  core.set_registers(state);
  auto expected = state;
  for (const auto operation : {mac_a, mac_b}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
  }
  return_model::apply(expected);
  const auto returned = core.run(3);
  require(returned.dispatches == 3 && returned.fault.kind == FaultKind::none &&
              returned.progress == Progress{.instructions = progress.instructions + 3,
                                            .estimated_cycles = progress.estimated_cycles + 5} &&
              core.registers() == expected && !core.interruptible(),
          "RTI materializes dirty native A/B and restores the complete saved PC/SR/frame");
  require(core.run(0).dispatches == 0 && core.registers() == expected && !core.interruptible(),
          "A zero budget cannot consume the post-return interrupt exclusion");
  bool rejected = false;
  try {
    core.interrupt(vector_a, 0);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && core.registers() == expected && core.progress() == returned.progress,
          "Interrupt admission rejects the immediate post-RTI boundary without side effects");
  for (const auto operation : continuation) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  const auto result = core.run(2);
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && core.interruptible() &&
              result.progress == Progress{.instructions = progress.instructions + 5,
                                          .estimated_cycles = progress.estimated_cycles + 7},
          "Fresh Carry/scaling/RM select correct consumers despite warm old-mode native code");
  if (backend == Backend::jit) {
    require(core.jit_statistics().native_alu_instructions == native + 4,
            "The return boundary contains actual native arithmetic on both sides of RTI");
  }
}

void fast_roundtrip(const Backend backend, const Resume input) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array program{mac_a, mac_b};
  core.load(Space::program, vector_a, program);
  auto expected = return_model::seed({.status = input.status, .pointer = input.pointer});
  expected.pc = foreground;
  expected.sr = input.status;
  expected.x = 1;
  expected.y = 1;
  core.set_registers(expected);
  for (const auto operation : program) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
  }
  core.interrupt(vector_a, 0);
  require(
      core.registers() == expected && !core.interruptible() &&
          core.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
      "Fast vector instructions retain foreground PC/stack and count both documented base costs");
  ++expected.pc;
  require(core.run(1).dispatches == 1 && core.registers() == expected && core.interruptible() &&
              core.progress() == Progress{.instructions = 3, .estimated_cycles = 3},
          "Foreground resumes after fast service without an RTI or fabricated timing costs");
}

void nested_interrupts(const Backend backend, const Resume input) {
  Machine machine(backend);
  auto &core = machine.core;
  core.write(Space::program, vector_a, call_short | service_a);
  core.write(Space::program, vector_b, call_short | service_b);
  const std::array outer{Word{0}, rti};
  core.load(Space::program, service_a, outer);
  core.write(Space::program, service_b, rti);
  auto expected = return_model::seed({.status = input.status, .pointer = input.pointer});
  expected.pc = foreground;
  expected.sr = input.status;
  core.set_registers(expected);
  return_model::interrupt_entry(expected, {.target = service_a});
  core.interrupt(vector_a, 0);
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 3} &&
              core.interruptible(),
          "Long interrupt entry saves foreground PC/SR and sets its admitted priority mask");
  return_model::interrupt_entry(expected, {.target = service_b, .level = 1});
  core.interrupt(vector_b, 1);
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 6},
          "A higher-priority long interrupt saves the outer service frame independently");
  return_model::apply(expected);
  require(core.run(1).dispatches == 1 && core.registers() == expected && !core.interruptible(),
          "Inner RTI restores the exact outer service PC/SR and remaining stack");
  ++expected.pc;
  require(core.run(1).dispatches == 1 && core.registers() == expected && core.interruptible(),
          "One resumed outer instruction releases the library interrupt exclusion");
  return_model::apply(expected);
  require(core.run(1).dispatches == 1 && core.registers() == expected && !core.interruptible(),
          "Outer RTI restores the exact original foreground PC/SR and stack");
  ++expected.pc;
  require(core.run(1).dispatches == 1 && core.registers() == expected && core.interruptible() &&
              core.progress() == Progress{.instructions = 6, .estimated_cycles = 14},
          "Nested entry/return counts guest retirement separately from uncalibrated IRQ timing");
}

enum class Failure : std::uint8_t {
  full_short,
  full_ea,
  invalid_ea,
  invalid_m,
  missing_opcode,
  missing_extension
};

struct FailedEntry {
  Failure failure = Failure::full_short;
  Word pointer = 0;
  unsigned slot = 0, index = 0;
};

void entry_failure(const Backend backend, const FailedEntry input) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  const auto pc = failure_vector + input.slot;
  auto size = memory_words;
  if (input.failure == Failure::missing_opcode) { size = pc; }
  if (input.failure == Failure::missing_extension) { size = pc + 1; }
  Core core({.program = std::span(p).first(size), .x = x, .y = y}, nullptr, {.backend = backend});
  auto expected = return_model::seed({.pointer = input.pointer});
  expected.pc = foreground;
  expected.sr = mode_rm | flag_c;
  expected.r[0] = service_a;
  const bool full = input.failure == Failure::full_short || input.failure == Failure::full_ea;
  if (full) { expected.sc = static_cast<Word>(stack_entries - 1); }
  if (input.failure == Failure::invalid_m) { expected.m[0] = invalid_modifier; }
  core.set_registers(expected);
  Word opcode = call_short | service_a;
  if (input.failure == Failure::full_ea || input.failure == Failure::invalid_m) {
    opcode = call_post;
  }
  if (input.failure == Failure::invalid_ea || input.failure == Failure::missing_extension) {
    opcode = call_long | (input.index << 8);
  }
  if (input.failure != Failure::missing_opcode) { core.write(Space::program, pc, opcode); }
  if (input.failure == Failure::invalid_ea) { core.write(Space::program, pc + 1, service_a); }
  Fault fault{.kind = FaultKind::stack, .address = pc, .value = input.pointer, .pc = pc};
  if (input.failure == Failure::invalid_ea) {
    fault.kind = FaultKind::invalid_instruction;
    fault.value = opcode;
  }
  if (input.failure == Failure::invalid_m) {
    fault.kind = FaultKind::unsupported_mode;
    fault.value = invalid_modifier;
  }
  if (input.failure == Failure::missing_opcode || input.failure == Failure::missing_extension) {
    fault.kind = FaultKind::memory_read;
    fault.address = static_cast<Word>(size);
    fault.value = 0;
  }
  if (input.failure == Failure::full_ea) { ++expected.r[0]; }
  core.interrupt(failure_vector, 0);
  const Progress progress{.instructions = input.slot, .estimated_cycles = input.slot};
  require(core.fault() == fault && core.progress() == progress && core.registers() == expected &&
              !core.interruptible(),
          "Failed long interrupt decode/EA/push cannot fabricate a service PC/SR or retirement");
  require(
      core.run(2).dispatches == 0 && core.registers() == expected && core.progress() == progress,
      "Failed long entry remains stopped, preserving any prior vector instruction and AGU effect");
}

void failures(const Backend backend) {
  constexpr std::array kinds{Failure::full_short,
                             Failure::full_ea,
                             Failure::invalid_ea,
                             Failure::invalid_m,
                             Failure::missing_opcode,
                             Failure::missing_extension};
  for (const auto failure : kinds) {
    for (const Word pointer : {1U, 7U, 14U, 15U}) {
      if ((failure == Failure::full_short || failure == Failure::full_ea) && pointer != 15) {
        continue;
      }
      for (unsigned slot = 0; slot < 2; ++slot) {
        if (failure == Failure::invalid_ea) {
          for (unsigned index = 1; index < agu_registers; ++index) {
            entry_failure(backend,
                          {.failure = failure, .pointer = pointer, .slot = slot, .index = index});
          }
        } else {
          entry_failure(backend, {.failure = failure, .pointer = pointer, .slot = slot});
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::empty_returns(backend);
      dsp56300::test::failures(backend);
      for (const auto mode : dsp56300::test::modes) {
        for (const dsp56300::Word pointer : {1U, 7U, 12U, 13U}) {
          for (const dsp56300::Word flags : {0U, 1U, 128U, 255U}) {
            dsp56300::test::native_resume(backend, {.pointer = pointer, .status = mode | flags});
            dsp56300::test::fast_roundtrip(backend, {.pointer = pointer, .status = mode | flags});
            dsp56300::test::nested_interrupts(backend,
                                              {.pointer = pointer, .status = mode | flags});
          }
        }
      }
    }
    std::cout << dsp56300::test::checks
              << " return/interrupt architectural and native checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << dsp56300::test::checks << " checks before failure: " << error.what() << '\n';
    return 1;
  }
}
