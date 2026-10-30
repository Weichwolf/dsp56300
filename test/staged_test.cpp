// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::staged_test {
void arithmetic() {
  for (const auto backend : backends) {
    Bus bus;
    Core core(bus, {.backend = backend});
    const auto before = seed();
    core.set_registers(before);
    const auto saved = core.checkpoint();
    const auto result = core.begin_instruction({.address = boot, .opcode = add_x0});
    const auto value = pending(core);
    auto expected = before;
    expected.a = std::uint64_t{1} << word_bits;
    expected.sr = flag_u;
    ++expected.pc;
    require(result.pending.has_value() && value.effect.kind == EffectKind::retire &&
                value.computed == expected && core.registers() == before &&
                core.progress() == Progress{} && bus.calls == 0,
            "Captured ADD computes literal full state without fetching or publishing");
    rejected([&] { static_cast<void>(core.run(1)); });
    rejected([&] { core.set_registers(expected); });
    rejected([&] { core.request_debug(); });
    retire(core);
    require(core.registers() == expected &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
            "Only retirement publishes the real arithmetic and counter result");
    rejected([&] { static_cast<void>(core.advance_instruction(value.ticket)); });
    core.restore(saved);
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = add_x0}));
    rejected([&] { static_cast<void>(core.advance_instruction(value.ticket)); });
    retire(core);
  }
}

namespace {
void memory_case(const Backend backend, const Word opcode) {
  Bus bus;
  const bool write = opcode == write_x0;
  const auto space = opcode == movem_x0 ? Space::program : Space::x;
  bus.events = {{.kind = write ? EffectKind::write : EffectKind::read,
                 .space = space,
                 .value = write ? 1U : raw_datum}};
  Core core(bus, {.backend = backend});
  const auto initial = seed();
  core.set_registers(initial);
  InstructionWords captured{.address = boot, .opcode = opcode};
  if (opcode == movem_x0) { captured.extension = data_address; }
  static_cast<void>(core.begin_instruction(captured));
  const auto effect = pending(core);
  require(effect.effect.space == space && effect.effect.address == data_address &&
              effect.effect.pc == boot && bus.calls == 0 && core.registers() == initial,
          "P/X effects use computed mapped addresses without reading program spans");
  if (opcode != movem_x0) {
    require(effect.computed.r[0] == data_address + 1,
            "The native AGU prefix is observed without an early host register patch");
  }
  auto tampered = effect;
  tampered.effect.address = 0;
  tampered.effect.value = 0;
  const auto completed = core.advance_instruction(tampered.ticket);
  require(completed.pending.has_value() && bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "The owner performs its stored bus effect; a copied view cannot replace it");
  require(core.registers() == initial && core.progress() == Progress{},
          "Accepted I/O does not publish architectural retirement");
  auto expected = initial;
  expected.pc += opcode == movem_x0 ? 2U : 1U;
  if (opcode != movem_x0) { ++expected.r[0]; }
  if (!write) { expected.x = datum; }
  retire(core);
  require(core.registers() == expected && bus.calls == 1,
          "Recomputation consumes the accepted word without a repeated physical operation");
}
}

void memory() {
  for (const auto backend : backends) {
    for (const auto opcode : {read_x0, write_x0, movem_x0}) { memory_case(backend, opcode); }
  }
}

void restore() {
  for (const auto origin : backends) {
    for (const auto destination : backends) {
      Bus bus;
      bus.events = {{}};
      Core core(bus, {.backend = origin});
      core.set_registers(seed());
      static_cast<void>(core.begin_instruction({.address = boot, .opcode = read_x0}));
      const auto first = pending(core);
      const auto waiting = core.checkpoint();
      Bus clone_bus = bus;
      Core clone(clone_bus, {.backend = destination});
      clone.restore(waiting);
      rejected([&] { static_cast<void>(clone.advance_instruction(first.ticket)); });
      const auto restored = pending(clone);
      static_cast<void>(clone.advance_instruction(restored.ticket));
      const auto after_read = clone.checkpoint();
      const auto old = pending(clone);
      retire(clone);
      clone.restore(after_read);
      rejected([&] { static_cast<void>(clone.advance_instruction(old.ticket)); });
      retire(clone);
      auto expected = seed();
      expected.x = datum;
      ++expected.pc;
      ++expected.r[0];
      require(clone.registers() == expected && clone_bus.calls == 1 && bus.calls == 0,
              "Restored pending/read-complete owners preserve accepted I/O and rebind tickets");
    }
  }
}
} // namespace dsp56300::test::staged_test

int main() {
  try {
    using namespace dsp56300::test::staged_test;
    arithmetic();
    memory();
    multiple_effects();
    restore();
    interrupt();
    repetition();
    faults();
    events();
    std::cout << dsp56300::test::checks << " staged instruction assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
