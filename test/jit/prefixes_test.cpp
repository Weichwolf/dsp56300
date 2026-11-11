// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit/image_plan.hpp"
#include "jit/prefixes.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word entry = 0x100, pure = 0x180, output = 0x1a0, repeated = 0x1c0, vector = 0x10;
constexpr Word port = 0xffffc0, datum = 0x654321;
constexpr unsigned word_bits = 24, byte_shift = 8;
constexpr Word load_direction = 0x8000;
constexpr Word write = op<isa::Opcode::movep_register_high> | (4U << byte_shift) | load_direction;
constexpr Word call = op<isa::Opcode::jsr_absolute_short> | entry;
constexpr Word repeat_count = 4;
constexpr auto infinite = std::numeric_limits<std::uint64_t>::max();
constexpr std::array body{op<isa::Opcode::nop>, op<isa::Opcode::nop>, write, op<isa::Opcode::stop>};
constexpr std::array cycle{op<isa::Opcode::bra_relative_short>};
constexpr std::array immediate{write, op<isa::Opcode::stop>};
constexpr std::array repeat{op<isa::Opcode::rep_immediate> | (repeat_count << byte_shift),
                            op<isa::Opcode::nop>,
                            write,
                            op<isa::Opcode::stop>};
constexpr std::array vectors{write,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::nop>,
                             write,
                             call,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::nop>,
                             call,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::nop>};

auto streams() {
  return std::array{CodeSpan{.address = entry, .words = body},
                    CodeSpan{.address = pure, .words = cycle},
                    CodeSpan{.address = output, .words = immediate},
                    CodeSpan{.address = repeated, .words = repeat},
                    CodeSpan{.address = vector, .words = vectors}};
}

const NativeImage &image() {
  static const auto compiled = [] {
    const auto spans = streams();
    constexpr std::array entries{entry, pure, output, repeated};
    const std::array versions{
        CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
    constexpr std::array requests{InterruptRequest{.vector = vector, .level = 0},
                                  InterruptRequest{.vector = vector + 2, .level = 0},
                                  InterruptRequest{.vector = vector + 4, .level = 0},
                                  InterruptRequest{.vector = vector + 6, .level = 0},
                                  InterruptRequest{.vector = vector + 8, .level = 0}};
    return Core::compile(
        {.variants = versions},
        {.memory = {}, .interrupts = requests, .transitions = {}, .callbacks = {}, .bindings = {}});
  }();
  return compiled;
}

bool unchanged(const State &actual, const State &before) {
  return actual.registers == before.registers && actual.failure == before.failure &&
         actual.execution == before.execution && actual.progress == before.progress;
}

struct Write {
  Space space = Space::program;
  Word address = 0, pc = 0, value = 0;
};

struct Bus final : Io {
  Core *core = nullptr;
  Progress observed{};
  Write access;
  bool called = false;

  bool observe(const Write value) noexcept {
    called = true;
    observed = core->progress();
    access = value;
    return false;
  }

  bool write(Space space, Word address, Word pc, Word value) noexcept override {
    return observe({.space = space, .address = address, .pc = pc, .value = value});
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
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
};

struct Machine {
  Machine() : core(Memory::separate({.program = program, .x = x, .y = y}), &bus) {
    for (const auto &span : streams()) {
      std::ranges::copy(span.words, program.begin() + span.address);
    }
    bus.core = &core;
    Registers registers;
    registers.pc = entry;
    registers.x = datum;
    registers.m.fill(word_mask);
    core.set_registers(registers);
    core.install(image(), 0);
  }

  std::array<Word, 512> program{};
  std::array<Word, 2> x{}, y{};
  Bus bus;
  Core core;
};

detail::ImagePlan plan() {
  const auto spans = streams();
  constexpr std::array entries{entry, pure, output, repeated};
  return detail::plan_image(spans, entries);
}

void foreground() {
  const auto source = plan();
  const auto prepared = detail::foreground_prefixes(source, {});
  require(prepared.at(entry) == 2 && prepared.at(entry + 1) == 1 && prepared.at(entry + 2) == 0 &&
              prepared.at(pure) == infinite && prepared.at(repeated) == 0,
          "Cold ordinary prefixes retain the actual owner boundary and pure closed cycle");
  Machine machine;
  const auto before = machine.core.state();
  detail::Cpu cpu;
  cpu.reg = machine.core.registers();
  require(detail::native_prefix(source.instructions, {}, prepared, cpu) == 2 &&
              unchanged(machine.core.state(), before) && !machine.bus.called,
          "Ordinary prefix inspection executes no guest, owner call or clock change");
  const auto result = machine.core.run(16);
  require(result.fault.kind == FaultKind::io_write && machine.bus.observed.estimated_cycles == 2 &&
              machine.bus.observed.instructions == 2 && machine.bus.access.space == Space::x &&
              machine.bus.access.address == port && machine.bus.access.pc == entry + 2 &&
              machine.bus.access.value == datum,
          "The real public native image first calls its owner after exactly two NOPs");
}

void inserted_vectors() {
  const auto ordinary = plan();
  constexpr std::array<std::uint64_t, 5> expected{0, 1, 3, 4, 2};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    Machine machine;
    const auto address = vector + static_cast<Word>(2 * index);
    const auto captured =
        detail::plan_vector(address, {vectors[2 * index], vectors[(2 * index) + 1]}, ordinary);
    const auto before = machine.core.state();
    require(detail::vector_entry_prefix(captured, before.registers, {}) == expected[index] &&
                unchanged(machine.core.state(), before) && !machine.bus.called,
            "Inserted vector analysis uses captured slots without adding a fixed IRQ latency");
    machine.core.interrupt(address, 0);
    if (index < 2) {
      require(
          machine.core.state().failure.kind == FaultKind::io_write && machine.bus.called &&
              machine.bus.observed.estimated_cycles == expected[index] &&
              machine.bus.access.pc == address + index && machine.bus.access.value == datum,
          "A real fast vector can call the owner before any cycle or after just its first slot");
    } else {
      require(machine.core.state().failure == Fault{} && !machine.bus.called &&
                  machine.core.progress().estimated_cycles == expected[index],
              "Pure fast completion and long service calls end their precise vector context");
      require(machine.core.run(16).fault.kind == FaultKind::io_write &&
                  machine.bus.observed.estimated_cycles == expected[index] + 2,
              "The next original foreground or long-service prefix resumes without replay");
    }
  }
}

void stack_fault() {
  Machine machine;
  auto registers = machine.core.registers();
  registers.sc = stack_entries - 1;
  machine.core.set_registers(registers);
  const auto ordinary = plan();
  const auto captured = detail::plan_vector(vector + 6, {vectors[6], vectors[7]}, ordinary);
  require(detail::vector_entry_prefix(captured, registers, {}) == 1,
          "A full hardware stack cannot promise the following long-call retirement");
  machine.core.interrupt(vector + 6, 0);
  require(machine.core.state().failure.kind == FaultKind::stack && !machine.bus.called &&
              machine.core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "The actual overflowing vector call faults after only its preceding NOP");
}

void retained_control() {
  const auto source = plan();
  const auto prepared = detail::foreground_prefixes(source, {});
  detail::Cpu cpu;
  cpu.reg.pc = entry;
  cpu.reg.sr = detail::loop_flag;
  require(detail::native_prefix(source.instructions, {}, prepared, cpu) == 0,
          "Ordinary edges cannot authorize an inherited DO return to a different prepared block");
  Machine loop;
  auto inherited = loop.core.registers();
  inherited.sr = detail::loop_flag;
  inherited.la = entry;
  inherited.lc = 2;
  inherited.sp = inherited.sc = 2;
  inherited.stack[2] = std::uint64_t{output} << word_bits;
  loop.core.set_registers(inherited);
  require(loop.core.run(16).fault.kind == FaultKind::io_write && loop.bus.access.pc == output &&
              loop.bus.observed.estimated_cycles == 1,
          "Real inherited DO state reaches an earlier owner call than the ordinary two-NOP path");
  Machine rep;
  auto initial = rep.core.registers();
  initial.pc = repeated;
  rep.core.set_registers(initial);
  require(rep.core.run(1).retired == 1 && !rep.core.interrupt_ready(),
          "A real prepared REP suppresses IRQ admission throughout its repeated body");
  cpu.reg.sr = 0;
  cpu.reg.pc = repeated + 1;
  cpu.repeat_pc = repeated + 1;
  cpu.repeat_remaining = repeat_count;
  require(detail::native_prefix(source.instructions, {}, prepared, cpu) == repeat_count,
          "A qualified retained REP-NOP body is pure until its actual count completes");
  cpu.repeat_loaded = true;
  cpu.repeat_opcode = write;
  require(detail::native_prefix(source.instructions, {}, prepared, cpu) == 0,
          "A different captured repeat opcode cannot borrow the prepared pure body's guarantee");
  const auto before = rep.core.progress();
  constexpr Word half = repeat_count / 2;
  require(rep.core.run_until(before.estimated_cycles + half).retired == half &&
              !rep.core.interrupt_ready() && !rep.bus.called,
          "An actual partial repeat remains noninterruptible at its precise native deadline");
  cpu.reg = rep.core.registers();
  cpu.repeat_remaining = half;
  cpu.repeat_opcode = op<isa::Opcode::nop>;
  require(detail::native_prefix(source.instructions, {}, prepared, cpu) == half,
          "A retained captured body uses its remaining count rather than replaying completed work");
  require(rep.core.run_until(before.estimated_cycles + repeat_count).retired == half &&
              !rep.bus.called &&
              rep.core.progress().estimated_cycles - before.estimated_cycles == repeat_count &&
              rep.core.interrupt_ready(),
          "The exact original repeat completes before ordinary IRQ eligibility resumes");
}

void memory_grants() {
  constexpr Word store = op<isa::Opcode::move_x_absolute_short> | (1U << byte_shift);
  constexpr Word load = store | load_direction;
  std::array memory{RegionLayout{.space = Space::x, .first = 1, .size = 1}};
  require(!detail::prefix_boundary(detail::decode(store), memory) &&
              !detail::prefix_boundary(detail::decode(load), memory),
          "A fixed ordinary RAM transfer has no host callback boundary");
  memory[0].tracked = true;
  require(!detail::prefix_boundary(detail::decode(store), memory) &&
              detail::prefix_boundary(detail::decode(load), memory),
          "Tracked writes initialize RAM directly; cold analysis does not assume a known read");
  memory[0].tracked = false;
  memory[0].writable = false;
  require(detail::prefix_boundary(detail::decode(store), memory),
          "A denied RAM write retains its actual owner path");
  memory[0].writable = true;
  memory[0].aliases_program = true;
  require(detail::prefix_boundary(detail::decode(store), memory),
          "A program-alias write ends the old executable binding proof");
}
}

int main() try {
  foreground();
  inserted_vectors();
  stack_fault();
  retained_control();
  memory_grants();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
