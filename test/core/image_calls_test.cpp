// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word entry = 0x100, outer = 0x140, inner = 0x160;
constexpr unsigned word_bits = 24;
constexpr Word pointer_mask = 63, status_mask = 255;
constexpr std::size_t operations = 9;
constexpr Word move_r0 = op<isa::Opcode::move_immediate_r0> | (3U << 8);
constexpr Word move_r1 = op<isa::Opcode::move_immediate_r0> | (1U << 16) | (5U << 8);
constexpr Word change_r1 = op<isa::Opcode::move_immediate_r0> | (1U << 16) | (7U << 8);
constexpr Word move_r2 = op<isa::Opcode::move_immediate_r0> | (2U << 16) | (9U << 8);
constexpr std::array root{
    move_r0, op<isa::Opcode::jsr_absolute_short> | outer, op<isa::Opcode::stop>};
constexpr std::array loop{root[0], root[1], op<isa::Opcode::jmp_absolute_short> | entry};
constexpr std::array body{
    move_r1, op<isa::Opcode::jsr_absolute_short> | inner, change_r1, op<isa::Opcode::rts>};
constexpr std::array leaf{move_r2, op<isa::Opcode::rts>};

auto streams(const std::span<const Word> caller) {
  // One logical caller deliberately crosses transport fragments.
  return std::array{CodeSpan{.address = entry, .words = caller.first(1)},
                    CodeSpan{.address = entry + 1, .words = caller.subspan(1)},
                    CodeSpan{.address = outer, .words = body},
                    CodeSpan{.address = inner, .words = leaf}};
}

NativeImage compile() {
  const auto stopped = streams(root);
  const auto running = streams(loop);
  constexpr std::array entries{entry};
  const std::array versions{CodeVariant{.id = 0, .code = stopped, .entries = entries},
                            CodeVariant{.id = 1, .code = running, .entries = entries}};
  return Core::compile({.variants = versions}, {});
}

void push(Registers &state, const Word resume) {
  std::uint64_t frame = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    frame |= std::uint64_t{(resume >> bit) & 1U} << (bit + word_bits);
    frame |= std::uint64_t{(state.sr >> bit) & 1U} << bit;
  }
  state.sp = (state.sp + 1) % 64;
  ++state.sc;
  state.stack[state.sp % stack_entries] = frame;
}

std::array<State, operations + 1> prefixes(const Registers &initial) {
  struct Boundary {
    Word pc;
    unsigned cycles;
  };

  std::array<State, operations + 1> result;
  State current{
      .registers = initial, .failure = {}, .execution = ExecutionState::running, .progress = {}};
  result[0] = current;
  const auto save = [&](const Boundary boundary) {
    current.registers.pc = boundary.pc;
    ++current.progress.instructions;
    current.progress.estimated_cycles += boundary.cycles;
    result[static_cast<std::size_t>(current.progress.instructions)] = current;
  };
  current.registers.r[0] = 3;
  save({.pc = entry + 1, .cycles = 1});
  push(current.registers, entry + 2);
  save({.pc = outer, .cycles = 3});
  current.registers.r[1] = 5;
  save({.pc = outer + 1, .cycles = 1});
  push(current.registers, outer + 2);
  save({.pc = inner, .cycles = 3});
  current.registers.r[2] = 9;
  save({.pc = inner + 1, .cycles = 1});
  current.registers.sp = (current.registers.sp + pointer_mask) % (pointer_mask + 1);
  --current.registers.sc;
  save({.pc = outer + 2, .cycles = 3});
  current.registers.r[1] = 7;
  save({.pc = outer + 3, .cycles = 1});
  current.registers.sp = (current.registers.sp + pointer_mask) % (pointer_mask + 1);
  --current.registers.sc;
  save({.pc = entry + 2, .cycles = 3});
  current.execution = ExecutionState::stopped;
  save({.pc = entry + 3, .cycles = 10});
  return result;
}

void compare(const Core &core, const RunResult &result, const State &expected) {
  const auto actual = core.state();
  require(actual.registers == expected.registers && actual.progress == expected.progress &&
              actual.execution == expected.execution && actual.failure == Fault{} &&
              result.retired == expected.progress.instructions &&
              result.progress == expected.progress && result.state == expected.execution &&
              result.fault == Fault{} && !result.yielded,
          "Native function transfers preserve the independent complete architectural prefix");
}

struct Machine {
  explicit Machine(const NativeImage &image)
      : core(Memory::separate({.program = program, .x = x, .y = y})) {
    for (const auto &span : streams(root)) {
      std::ranges::copy(span.words, program.begin() + span.address);
    }
    core.install(image, 0);
  }

  void reset(const Registers &initial) {
    core.reset();
    core.set_registers(initial);
  }

  std::array<Word, 512> program{};
  std::array<Word, 1> x{}, y{};
  Core core;
};

void boundaries(Machine &machine) {
  for (Word flags = 0; flags < 256; ++flags) {
    for (const Word pointer : {0U, 15U, pointer_mask}) {
      auto initial = test::register_model::seed(0);
      initial.pc = entry;
      initial.sr = flags;
      initial.sp = pointer;
      initial.sc = 0;
      const auto expected = prefixes(initial);
      for (std::uint32_t count = 0; count <= operations + 2; ++count) {
        machine.reset(initial);
        compare(machine.core,
                machine.core.run(count),
                expected[std::min<std::size_t>(count, operations)]);
      }
      for (std::uint64_t deadline = 0; deadline <= expected.back().progress.estimated_cycles + 1;
           ++deadline) {
        machine.reset(initial);
        std::size_t index = 0;
        while (index < operations && expected[index].progress.estimated_cycles < deadline) {
          ++index;
        }
        compare(machine.core, machine.core.run_until(deadline), expected[index]);
      }
    }
  }
}

void continuous(Machine &machine, const NativeImage &image) {
  constexpr std::uint32_t iterations = 100000;
  constexpr std::uint32_t count = iterations * operations;
  constexpr std::uint64_t loop_cycles = 19;
  machine.core.load(Space::program, entry + 2, std::span(loop).last(1));
  machine.core.install(image, 1);
  auto initial = test::register_model::seed(0);
  initial.pc = entry;
  initial.sr = status_mask;
  initial.sp = pointer_mask;
  initial.sc = 0;
  machine.reset(initial);
  auto expected = prefixes(initial).back();
  expected.registers.pc = entry;
  expected.execution = ExecutionState::running;
  expected.progress = {.instructions = count, .estimated_cycles = iterations * loop_cycles};
  compare(machine.core, machine.core.run(count), expected);
}

struct ResetIo final : Io {
  bool reset_peripherals(const Word pc) noexcept override {
    if (calls >= addresses.size()) { return false; }
    addresses[calls] = pc;
    visible[calls] = owner->registers().pc;
    ++calls;
    if (calls == 1) { owner->request_yield(); }
    return true;
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Core *owner = nullptr;
  std::array<Word, 2> addresses{}, visible{};
  unsigned calls = 0;
};

void shared_io() {
  constexpr std::uint64_t total_cycles = (2 * 7) + 10;
  constexpr std::array code{op<isa::Opcode::reset>, op<isa::Opcode::reset>, op<isa::Opcode::stop>};
  const std::array spans{CodeSpan{.address = entry, .words = code}};
  constexpr std::array entries{entry};
  const CodeVariant version{.id = 0, .code = spans, .entries = entries};
  const auto image = Core::compile({.variants = {&version, 1}}, {});
  std::array<Word, 512> program{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  std::ranges::copy(code, program.begin() + entry);
  ResetIo io;
  Core core(Memory::separate({.program = program, .x = x, .y = y}), &io);
  io.owner = &core;
  auto initial = test::register_model::seed(0);
  initial.pc = entry;
  initial.sr = status_mask;
  core.set_registers(initial);
  core.install(image, 0);
  require(io.calls == 0, "Cold specialization and installation perform no real I/O");
  const auto first = core.run();
  require(first.retired == 1 && first.yielded && first.fault == Fault{} && io.calls == 1 &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 7},
          "A real first action yields at its exact completed native prefix");
  const auto rest = core.run();
  initial.pc += code.size();
  require(rest.retired == 2 && !rest.yielded && rest.fault == Fault{} && io.calls == 2 &&
              rest.state == ExecutionState::stopped && core.registers() == initial &&
              core.progress() == Progress{.instructions = 3, .estimated_cycles = total_cycles} &&
              io.addresses == std::array<Word, 2>{entry, entry + 1} &&
              io.visible == std::array<Word, 2>{entry + 1, entry + 2},
          "Repeated identical encodings keep distinct real I/O PCs and never replay an action");
}
}

int main() {
  try {
    const auto image = compile();
    Machine machine(image);
    boundaries(machine);
    continuous(machine, image);
    shared_io();
    std::cout << test::checks << " public native function-transfer checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
