// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::staged_test {
void interrupt() {
  constexpr Word level = 1;
  constexpr Word mask = 0x200;
  for (const auto backend : backends) {
    for (const bool long_entry : {false, true}) {
      Bus bus;
      Core core(bus, {.backend = backend});
      auto expected = seed();
      core.set_registers(expected);
      core.request_interrupt(vector, level, {.address = boot, .opcode = 0});
      require(core.instruction_fetch().address == vector &&
                  core.instruction_fetch().exception_vector && bus.calls == 0 &&
                  core.registers() == expected,
              "Admitted IRQ retains genuine normal PC and does not read or acknowledge a bus");
      static_cast<void>(core.begin_instruction({.address = vector, .opcode = add_x0}));
      retire(core);
      expected.a = std::uint64_t{1} << word_bits;
      expected.sr = flag_u;
      require(core.registers() == expected && core.instruction_fetch().address == vector + 1 &&
                  core.instruction_fetch().exception_vector,
              "First actual vector instruction has a distinct result and unchanged normal PC");
      const auto opcode = long_entry ? short_call | handler : add_x0;
      static_cast<void>(core.begin_instruction({.address = vector + 1, .opcode = opcode}));
      retire(core);
      if (long_entry) {
        expected.sp = expected.sc = 1;
        expected.stack[1] = (std::uint64_t{boot} << word_bits) | flag_u;
        expected.sr = mask | flag_u;
        expected.pc = handler;
        require(core.registers() == expected,
                "Genuine vector JSR saves the foreground frame and raises the selected IPL");
        static_cast<void>(core.begin_instruction({.address = handler, .opcode = rti}));
        retire(core);
        expected.sp = expected.sc = 0;
        expected.sr = flag_u;
        expected.pc = boot;
      } else {
        expected.a = std::uint64_t{2} << word_bits;
      }
      require(core.registers() == expected && !core.instruction_fetch().exception_vector &&
                  bus.calls == 0,
              "Fast completion or real RTI restores the independently specified foreground");
    }
  }
}

void vector_provenance() {
  constexpr Word level = 1;
  constexpr Word trap = 6;
  constexpr Word trap_vector = 8;
  for (const auto backend : backends) {
    Bus bus;
    Core core(bus, {.backend = backend});
    core.set_registers(seed());
    core.request_interrupt(vector, level, {.address = boot, .opcode = 0});
    const auto fetch = core.instruction_fetch();
    require(fetch.address == vector && fetch.exception_vector, "IRQ must expose vector provenance");
    static_cast<void>(
        core.begin_instruction({.address = vector, .opcode = short_call | (vector + 1)}));
    retire(core);
    const auto handler_fetch = core.instruction_fetch();
    require(handler_fetch.address == vector + 1 && !handler_fetch.exception_vector &&
                core.registers().sp == 1,
            "A one-word call to the second vector address is a handler, not a vector fetch");
    Core software(bus, {.backend = backend});
    software.set_registers(seed());
    static_cast<void>(software.begin_instruction({.address = boot, .opcode = trap}));
    retire(software);
    require(software.instruction_fetch().address == trap_vector &&
                software.instruction_fetch().exception_vector,
            "A native software exception exposes its actual first vector fetch");
    static_cast<void>(software.begin_instruction({.address = trap_vector, .opcode = 0}));
    retire(software);
    require(software.instruction_fetch().address == trap_vector + 1 &&
                software.instruction_fetch().exception_vector,
            "The second software-vector fetch retains exception ownership");
    static_cast<void>(software.begin_instruction({.address = trap_vector + 1, .opcode = 0}));
    retire(software);
    require(software.instruction_fetch().address == boot + 1 &&
                !software.instruction_fetch().exception_vector,
            "Software-vector completion restores ordinary foreground ownership");
  }
}

void repetition() {
  for (const auto backend : backends) {
    Bus bus;
    Core core(bus, {.backend = backend});
    auto expected = seed();
    core.set_registers(expected);
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = repeat_four}));
    retire(core);
    ++expected.pc;
    expected.lc = 4;
    require(core.registers() == expected, "Native REP establishes its real count and resume PC");
    for (unsigned iteration = 0; iteration < 4; ++iteration) {
      const auto fetch = core.instruction_fetch();
      require(fetch.address == boot + 1 && fetch.latched_opcode.has_value() == (iteration != 0),
              "Subsequent REP iterations expose their actual native latch without a P read");
      if (iteration != 0) {
        rejected(
            [&] { static_cast<void>(core.begin_instruction({.address = boot + 1, .opcode = 0})); });
      }
      static_cast<void>(core.begin_instruction({.address = boot + 1, .opcode = add_x0}));
      retire(core);
      expected.a += std::uint64_t{1} << word_bits;
      expected.sr = flag_u;
      expected.lc = iteration == 3 ? 7U : 3 - iteration;
      if (iteration == 3) { ++expected.pc; }
      require(core.registers() == expected,
              "REP snapshots retain each literal body result and saved LC");
    }
    rejected([&] { static_cast<void>(core.interruptible()); });
    require(core.interruptible({.address = boot + 2, .opcode = 0}) &&
                !core.interruptible({.address = boot + 2, .opcode = repeat_four}) && bus.calls == 0,
            "Post-REP exclusion uses provided metadata and cannot introduce a physical fetch");
  }
}

void faults() {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word bit_set = 0x0a8020;
  constexpr Word port = 0xffffc0;
  for (const auto backend : backends) {
    Bus bus;
    bus.events = {{.accept = false}};
    Core core(bus, {.backend = backend});
    const auto initial = seed();
    core.set_registers(initial);
    rejected([&] { static_cast<void>(core.begin_instruction({.address = 0, .opcode = 0})); });
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = read_x0}));
    const auto value = pending(core);
    const auto failed = core.advance_instruction(value.ticket);
    require(failed.fault.kind == FaultKind::memory_read && failed.fault.address == data_address &&
                failed.fault.pc == boot && core.registers() == initial &&
                core.progress() == Progress{} && bus.calls == 1 && !failed.pending,
            "Unknown/rejected memory stops visibly without an invented word or retirement");
    rejected([&] { static_cast<void>(core.advance_instruction(value.ticket)); });
    core.reset();
    core.set_registers(initial);
    auto cached = initial;
    cached.sr = cache_enable;
    core.set_registers(cached);
    const auto cached_nop = core.begin_instruction({.address = boot, .opcode = 0});
    const auto cached_pending = cached_nop.pending;
    if (!cached_pending) { throw std::runtime_error("Captured CE-enabled NOP lost its result"); }
    require(cached_nop.fault.kind == FaultKind::none &&
                cached_pending->effect.kind == EffectKind::retire && bus.calls == 1,
            "Captured CE-enabled NOP must not manufacture a program-bus access");
    retire(core);
    ++cached.pc;
    require(core.registers() == cached &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 1} &&
                bus.calls == 1,
            "Captured CE-enabled NOP must retire with literal state and progress");
    core.reset();
    core.set_registers(initial);
    bus.events = {{.kind = EffectKind::bit, .address = port, .value = 1, .action = BitAction::set}};
    bus.calls = bus.accepted = 0;
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = bit_set}));
    const auto bit = pending(core);
    require(bit.effect.kind == EffectKind::bit && bit.effect.address == port && bus.calls == 0,
            "Atomic bit action remains an owned pending bus request");
    static_cast<void>(core.advance_instruction(bit.ticket));
    retire(core);
    auto expected = initial;
    ++expected.pc;
    expected.sr = flag_c;
    require(core.registers() == expected && bus.calls == 1 && bus.exact,
            "The actual atomic old-bit response drives native carry exactly once");
  }
}
} // namespace dsp56300::test::staged_test
