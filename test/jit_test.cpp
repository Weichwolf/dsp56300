// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word mac = 0x2000d2, clear = 0x200013, negate = 0x200036;
constexpr Word io_load_clear = 0x44e013, io_load = 0x44e000;
constexpr Word io_address = 0xffff80;

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {
    core.set_registers(initial_state());
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

void exact_budget(const bool optimize) {
  Machine machine({.backend = Backend::jit, .instructions_per_block = 8, .optimize = optimize});
  std::array<Word, 16> program{};
  program.fill(mac);
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  machine.core.set_registers(state);
  require(machine.core.run(0).dispatches == 0 && machine.core.jit_statistics().compiled_blocks == 0,
          "Zero budget does not compile or execute");
  require(machine.core.run(8).dispatches == 8 && machine.core.registers().a == 16,
          "A generated block retires exactly its guest budget");
  machine.core.set_registers(state);
  require(machine.core.run(3).dispatches == 3 && machine.core.registers().a == 6 &&
              machine.core.registers().pc == start + 3,
          "A short tail never enters an oversized cached block");
  const auto statistics = machine.core.jit_statistics();
  require(statistics.native_alu_instructions + statistics.helper_instructions == 11,
          "Native and helper retirement counts account for partitioned runs");
  require(optimize ? statistics.native_alu_instructions != 0
                   : statistics.native_alu_instructions == 0,
          "The native lowering option selects actual native ALU execution");
}

void numeric_modes() {
  Machine machine({.backend = Backend::jit});
  machine.core.write(Space::program, start, mac);
  auto state = initial_state();
  constexpr unsigned extension_bit = 46;
  state.a = std::uint64_t{1} << extension_bit;
  machine.core.set_registers(state);
  require(machine.core.run(1).dispatches == 1 && machine.core.registers().sr == 0,
          "Native flags use the unscaled extension boundary");
  state.sr = mode_up;
  machine.core.set_registers(state);
  require(machine.core.run(1).dispatches == 1 && machine.core.registers().sr == (mode_up | flag_e),
          "A changed scaling mode selects a separately guarded translation");
  require(machine.core.jit_statistics().compiled_blocks == 2,
          "Numeric modes have distinct translation keys");
}

void host_program_write() {
  Machine machine({.backend = Backend::jit});
  machine.core.write(Space::program, start, mac);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  machine.core.set_registers(state);
  require(machine.core.run(1).dispatches == 1 && machine.core.registers().a == 2,
          "Native code is populated");
  machine.core.write(Space::program, start, clear);
  machine.core.set_registers(state);
  require(machine.core.run(1).dispatches == 1 && machine.core.registers().a == 0,
          "Host program writes discard the previous native operation");
  require(machine.core.jit_statistics().invalidations != 0, "Program invalidation is observable");
  machine.core.reset();
  require(machine.core.jit_statistics().executable_bytes == 0 &&
              machine.core.jit_statistics().compiled_blocks == 0,
          "Reset releases executable code and diagnostics");
}

class ProgramWriter final : public Io {
public:
  Core *core = nullptr;
  unsigned reads = 0;
  bool reentry_rejected = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space != Space::x || address != io_address || pc != start || core == nullptr) {
      return false;
    }
    ++reads;
    try {
      static_cast<void>(core->run(0));
    } catch (const std::logic_error &) { reentry_rejected = true; }
    core->write(Space::program, start, negate);
    core->write(Space::program, start + 1, 0);
    value = 1;
    return true;
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
};

void callback_program_write(const Word opcode) {
  ProgramWriter io;
  Machine machine({.backend = Backend::jit}, &io);
  io.core = &machine.core;
  const std::array program{opcode, mac};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.r[0] = io_address;
  state.a = 3;
  state.y = 1;
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none && io.reads == 1,
          "An I/O callback may invalidate code currently executing");
  require(io.reentry_rejected, "Callbacks cannot reenter the execution engine");
  const auto actual = machine.core.registers();
  require(actual.a == (opcode == io_load_clear ? 0U : 3U) && actual.pc == start + 2 &&
              actual.x == 1,
          "The fetched operation completes and the modified successor is fetched afresh");
  require(machine.core.jit_statistics().invalidations == 1,
          "Executing code and embedded metadata remain alive until the block returns");
}

void memory_fault() {
  Machine machine({.backend = Backend::jit});
  machine.core.write(Space::program, start, io_load_clear);
  auto state = initial_state();
  state.r[0] = memory_words;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  require(result.dispatches == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.address == memory_words && result.fault.pc == start,
          "The instrumented move helper retains span checks before native retirement");
  require(machine.core.jit_statistics().native_alu_instructions == 0,
          "A failed move never executes or retires its ALU");
}

void interrupt_callback_boundary() {
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    ProgramWriter io;
    Machine machine({.backend = backend}, &io);
    io.core = &machine.core;
    const std::array program{io_load_clear, Word{0}};
    machine.core.load(Space::program, start, program);
    auto state = initial_state();
    state.r[0] = io_address;
    state.pc = start + 4;
    machine.core.set_registers(state);
    machine.core.interrupt(start, 0);
    require(io.reads == 1 && io.reentry_rejected && machine.core.registers().pc == state.pc,
            "Interrupt callbacks obey the same stopped-boundary contract");
    require(machine.core.run(0).dispatches == 0,
            "Interrupt entry releases its execution guard on return");
  }
}

void loop_boundary() {
  Machine machine({.backend = Backend::jit});
  constexpr Word do_three = 0x060380;
  const std::array program{do_three, start + 3, mac, mac};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  constexpr Word old_loop_address = 0x70;
  state.la = old_loop_address;
  state.lc = 7;
  machine.core.set_registers(state);
  require(machine.core.run(7).dispatches == 7, "DO and six body operations retire");
  const auto actual = machine.core.registers();
  require(actual.a == 12 && actual.la == state.la && actual.lc == state.lc &&
              actual.pc == start + 4 && actual.sp == 0 && actual.sc == 0,
          "Native execution stops before loop cleanup and preserves loop restoration");
  require(machine.core.jit_statistics().native_alu_instructions != 0,
          "The loop test actually exercises emitted ALU code");
}

void loop_cleanup_fault() {
  constexpr Word loop_flag = Word{1} << 15;
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    Machine machine({.backend = backend});
    machine.core.write(Space::program, start, mac);
    auto state = initial_state();
    state.la = start;
    state.lc = 1;
    state.sr = loop_flag;
    machine.core.set_registers(state);
    const auto result = machine.core.run(1);
    require(result.dispatches == 0 && result.progress == Progress{} &&
                result.fault.kind == FaultKind::stack,
            "Failed loop cleanup does not retire or consume instruction base cycles");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::exact_budget(false);
    dsp56300::test::exact_budget(true);
    dsp56300::test::numeric_modes();
    dsp56300::test::host_program_write();
    dsp56300::test::callback_program_write(dsp56300::test::io_load_clear);
    dsp56300::test::callback_program_write(dsp56300::test::io_load);
    dsp56300::test::memory_fault();
    dsp56300::test::interrupt_callback_boundary();
    dsp56300::test::loop_boundary();
    dsp56300::test::loop_cleanup_fault();
    std::cout << dsp56300::test::checks << " native boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
