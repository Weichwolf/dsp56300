// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <utility>

namespace dsp56300::test::checkpoint_test {
namespace {
constexpr Word cache_enable = Word{1} << 19, external = 1024;
constexpr Word service_mask = 0x300, wait_opcode = 0x86, stop_opcode = 0x87;
constexpr Word lock_opcode = 0x0bf081;

void sleep(const Backend backend, const Word opcode, const ProcessingState kind) {
  Machine machine(backend);
  machine.core.load(Space::program, start, std::array{opcode, Word{0}});
  machine.core.set_registers(seed());
  const auto healthy = machine.core.checkpoint();
  step(machine.core);
  auto expected = seed();
  ++expected.pc;
  require(machine.core.registers() == expected && machine.core.processing_state() == kind,
          "Own processing instruction establishes literal sleep/debug state");
  const auto saved = machine.core.checkpoint();
  machine.core.reset();
  machine.core.restore(saved);
  require(machine.core.run(8).dispatches == 0 && machine.core.registers() == expected &&
              machine.core.processing_state() == kind,
          "Restored sleep/debug cannot consume a successor budget");
  machine.core.restore(healthy);
  require(machine.core.processing_state() == ProcessingState::running &&
              machine.core.registers() == seed() && machine.core.progress() == Progress{},
          "An earlier complete checkpoint restores execution and counters");
}

void fault(const Backend backend) {
  Machine machine(backend);
  machine.core.load(Space::program, start, std::array{Word{7}});
  machine.core.set_registers(seed());
  const auto healthy = machine.core.checkpoint();
  require(machine.core.run(1).fault.kind == FaultKind::unsupported_instruction,
          "Own unsupported word creates a visible stopped fault");
  const auto saved = machine.core.checkpoint();
  const auto diagnostic = machine.core.fault();
  const auto registers = machine.core.registers();
  machine.core.restore(healthy);
  machine.core.write(Space::program, start, 0);
  step(machine.core);
  machine.core.restore(saved);
  require(machine.core.fault() == diagnostic && machine.core.registers() == registers &&
              machine.core.run(3).dispatches == 0,
          "Fault checkpoints retain diagnostics and do not replay accepted effects");
  machine.core.restore(healthy);
  step(machine.core);
  require(machine.core.registers().pc == start + 1,
          "Healthy CPU restoration pairs with caller-restored or deliberately changed RAM");
}

void pending_debug(const Backend backend) {
  Machine machine(backend);
  machine.core.set_registers(seed());
  machine.core.set_debug_interrupt_mode(true);
  machine.core.write(Space::program, 6, short_jsr | service);
  machine.core.request_debug();
  const auto saved = machine.core.checkpoint();
  machine.core.reset();
  machine.core.restore(saved);
  require(machine.core.debug_status().interrupt_mode,
          "Debug mode survives separately from architectural register initialization");
  step(machine.core);
  auto expected = seed();
  expected.pc = service;
  expected.sp = expected.sc = 1;
  expected.sr = service_mask;
  expected.stack[1] = std::uint64_t{start} << word_bits;
  require(machine.core.registers() == expected,
          "Restored pending debug request selects its saved vector and genuine frame");
}
}

void processing() {
  for (const auto backend : backends) {
    sleep(backend, wait_opcode, ProcessingState::waiting);
    sleep(backend, stop_opcode, ProcessingState::stopped);
    sleep(backend, 0x200, ProcessingState::debugging);
    fault(backend);
    pending_debug(backend);
  }
}

void cache() {
  for (const auto origin : backends) {
    for (const auto destination : backends) {
      for (const unsigned width : {128U, 256U}) {
        Machine source(origin, nullptr, width);
        auto expected = seed();
        expected.sr = cache_enable;
        source.core.set_registers(expected);
        source.core.load(Space::program, start, std::array{lock_opcode, external, Word{1}});
        source.core.write(Space::program, external, add_x0);
        step(source.core); // Own PLOCK locks the external sector.
        expected.pc = external;
        source.core.set_registers(expected);
        step(source.core);
        add(expected);
        expected.pc = start + 2;
        source.core.set_registers(expected);
        source.core.write(Space::program, external, 0);
        const auto saved = source.core.checkpoint();
        Machine target(destination, nullptr, width);
        target.memory = source.memory;
        target.core.restore(saved);
        step(target.core); // PFLUSHUN must retain the locked old ADD.
        expected.pc = external;
        target.core.set_registers(expected);
        step(target.core);
        add(expected);
        ++expected.pc;
        require(target.core.registers() == expected && target.memory.p[external] == 0,
                "Cache contents/locks survive fresh owners and differ from restored backing RAM");
      }
    }
  }
}

void geometry() {
  Machine source(Backend::cached);
  auto original = source.core.checkpoint();
  const auto copy = original;
  auto moved = std::move(original);
  invalid([&] { source.core.restore(Checkpoint{}); });
  source.core.restore(moved);
  source.core.restore(copy);
  Machine width(Backend::cached, nullptr, 256);
  const auto before = width.core.registers();
  invalid([&] { width.core.restore(copy); });
  require(width.core.registers() == before && width.core.progress() == Progress{},
          "Geometry rejection leaves destination state unchanged");
  const Buffers ram{.program = source.memory.p, .x = source.memory.x, .y = source.memory.y};
  const std::array incompatible{
      Buffers{.program = ram.program.first(words - 1), .x = ram.x, .y = ram.y},
      Buffers{.program = ram.program, .x = ram.program, .y = ram.y},
      Buffers{.program = ram.program, .x = ram.x, .y = ram.x}};
  for (const auto memory : incompatible) {
    Core target(memory, nullptr, {.cacheable_begin = 1024, .cacheable_end = words});
    invalid([&] { target.restore(copy); });
    require(target.registers() == before, "Memory shape/alias rejection does not modify CPU state");
  }
}
} // namespace dsp56300::test::checkpoint_test
