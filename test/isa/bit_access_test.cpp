// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word low_begin = 0xffff80, high_begin = 0xffffc0;
constexpr Word target = 0x234, relative_offset = 0x34;
constexpr Word port_half = 32, port_last = 63;

struct Kind {
  Word high = 0, low = 0;
  BitAction action = BitAction::test;
  bool flow = false, relative = false, call = false, set = false;
};

constexpr std::array<Kind, 12> kinds{
    {{.high = 0x0a8000, .low = 0x010000, .action = BitAction::clear},
     {.high = 0x0a8020, .low = 0x010020, .action = BitAction::set},
     {.high = 0x0b8000, .low = 0x014000, .action = BitAction::change},
     {.high = 0x0b8020, .low = 0x014020},
     {.high = 0x0a8080, .low = 0x018080, .flow = true},
     {.high = 0x0a80a0, .low = 0x0180a0, .flow = true, .set = true},
     {.high = 0x0b8080, .low = 0x01c080, .flow = true, .call = true},
     {.high = 0x0b80a0, .low = 0x01c0a0, .flow = true, .call = true, .set = true},
     {.high = 0x0cc000, .low = 0x048000, .flow = true, .relative = true},
     {.high = 0x0cc020, .low = 0x048020, .flow = true, .relative = true, .set = true},
     {.high = 0x0dc000, .low = 0x048080, .flow = true, .relative = true, .call = true},
     {.high = 0x0dc020,
      .low = 0x0480a0,
      .flow = true,
      .relative = true,
      .call = true,
      .set = true}}};

struct Event {
  Space space = Space::x;
  Word address = high_begin, pc = start;
  unsigned bit = 0, calls = 1;
  BitAction action = BitAction::test;
  bool previous = false, accept = true;
};

class Port final : public Io {
public:
  explicit Port(const Event expected) : expected_(expected) {}

  TestCore *core = nullptr;
  unsigned calls = 0;
  bool valid = true, mutate = false, reentry_rejected = false;
  Event expected_;

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    valid = false;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    valid = false;
    return false;
  }

  bool bit(const Space space,
           const Word address,
           const Word pc,
           const Word bit,
           const BitAction action,
           bool &previous) noexcept override {
    ++calls;
    valid = valid && calls <= expected_.calls && space == expected_.space &&
            address == expected_.address && pc == expected_.pc && bit == expected_.bit &&
            action == expected_.action;
    previous = expected_.previous;
    if (mutate && core != nullptr) {
      try {
        static_cast<void>(core->run(0));
      } catch (const std::logic_error &) { reentry_rejected = true; }
      core->write(Space::program, pc + 1, 0);
    }
    return valid && expected_.accept;
  }
};

struct Machine {
  Machine(const Backend backend, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, {.backend = backend}) {
    core.set_registers(initial_state());
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
};

struct PortCase {
  Kind kind{};
  Word index = 0;
  unsigned bit = 0;
  bool high = false, y = false, previous = false;
};

void port_case(const Backend backend, const PortCase &input) {
  const auto kind = input.kind;
  Port io({.space = input.y ? Space::y : Space::x,
           .address = (input.high ? high_begin : low_begin) + input.index,
           .bit = input.bit,
           .action = kind.action,
           .previous = input.previous});
  Machine machine(backend, &io);
  const Word opcode =
      (input.high ? kind.high : kind.low) | (input.index << 8) | (input.y ? 0x40U : 0U) | input.bit;
  std::vector<Word> program{opcode};
  if (kind.flow) { program.push_back(kind.relative ? relative_offset : target); }
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.sr = (status_byte & ~flag_c) | (input.previous ? 0U : flag_c);
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  state.pc += static_cast<Word>(program.size());
  Word cycles = 2;
  if (kind.flow) {
    cycles = kind.relative ? 5 : 4;
    if (kind.set == input.previous) {
      if (kind.call) {
        state.sp = 1;
        state.sc = 1;
        state.stack[1] = (std::uint64_t{start + 2} << word_bits) | state.sr;
      }
      state.pc = kind.relative ? start + relative_offset : target;
    }
  } else {
    state.sr = (state.sr & ~flag_c) | (input.previous ? flag_c : 0U);
  }
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = cycles} &&
              machine.core.registers() == state && io.valid && io.calls == 1,
          "Bit I/O uses one correctly addressed action callback and complete architectural state");
}

void port_variants(const Backend backend, const PortCase &input) {
  for (const bool previous : {false, true}) {
    for (const unsigned bit : {0U, 7U, word_bits - 1}) {
      auto variant = input;
      variant.previous = previous;
      variant.bit = bit;
      port_case(backend, variant);
    }
  }
}

void addressed_ports(const Backend backend, const Word index) {
  for (const auto kind : kinds) {
    if (kind.flow && index != 0 && index != port_half - 1 && index != port_half &&
        index != port_last) {
      continue;
    }
    for (const bool high : {false, true}) {
      for (const bool y : {false, true}) {
        port_variants(backend, {.kind = kind, .index = index, .high = high, .y = y});
      }
    }
  }
}

void effective_port(const Backend backend) {
  Port io({.space = Space::y,
           .address = low_begin,
           .bit = word_bits - 1,
           .action = BitAction::set,
           .previous = true});
  Machine machine(backend, &io);
  constexpr Word set_indirect = 0x0a5877;
  machine.core.write(Space::program, start, set_indirect);
  auto state = initial_state();
  state.r[0] = low_begin;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  state.pc = start + 1;
  state.r[0] = low_begin + 1;
  state.sr = flag_c;
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == state && io.valid && io.calls == 1,
          "A direct EA reaching I/O uses the atomic callback and performs its update once");
}

void rejected_port(const Backend backend) {
  Port io({.bit = word_bits - 1, .action = BitAction::change, .accept = false});
  Machine machine(backend, &io);
  constexpr Word change_port = 0x0b8017;
  machine.core.write(Space::program, start, change_port);
  const auto result = machine.core.run(1);
  require(result.retired == 0 && result.progress == Progress{} &&
              result.fault == Fault{.kind = FaultKind::io_bit,
                                    .space = Space::x,
                                    .address = high_begin,
                                    .value = word_bits - 1,
                                    .pc = start} &&
              io.valid && io.calls == 1,
          "Rejected bit I/O reports exact context without retiring");
  require(machine.core.run(1).retired == 0 && io.calls == 1,
          "Faulted bit I/O does not invoke the host again");
}

void invalid_bit(const Backend backend) {
  Port io({});
  Machine machine(backend, &io);
  constexpr Word invalid = 0x0a5818;
  machine.core.write(Space::program, start, invalid);
  auto state = initial_state();
  state.r[0] = low_begin;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  require(result.retired == 0 && result.fault.kind == FaultKind::invalid_instruction &&
              result.fault.value == invalid && io.calls == 0 &&
              machine.core.registers().r[0] == low_begin,
          "An invalid bit number faults before an address update or host callback");
}

void invalidation(const Backend backend) {
  Port io({.action = BitAction::change});
  Machine machine(backend, &io);
  io.core = &machine.core;
  io.mutate = true;
  constexpr Word change_port = 0x0b8000;
  constexpr Word negate = 0x200036;
  const std::array program{change_port, negate};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.a = 3;
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  state.pc = start + 2;
  require(
      result.retired == 2 && result.fault.kind == FaultKind::none &&
          machine.core.registers() == state && io.valid && io.calls == 1 && io.reentry_rejected,
      "Bit callbacks may change P but cannot reenter execution; stale successors are discarded");
}

void mode_change(const Backend backend) {
  Machine machine(backend);
  constexpr Word set_rounding = 0x0af975;
  constexpr Word round = 0x200011;
  const std::array program{set_rounding, round};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.a = std::uint64_t{1} << (word_bits - 1);
  state.sr = flag_c;
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  state.pc = start + 2;
  state.a = std::uint64_t{1} << word_bits;
  state.sr = mode_rm | flag_u | flag_c;
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == state,
          "BSET SR preserves C and exits a block before arithmetic in the changed mode");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend : {dsp56300::Backend::jit}) {
      for (dsp56300::Word index = 0; index < 64; ++index) {
        dsp56300::test::addressed_ports(backend, index);
      }
      dsp56300::test::effective_port(backend);
      dsp56300::test::rejected_port(backend);
      dsp56300::test::invalid_bit(backend);
      dsp56300::test::invalidation(backend);
      dsp56300::test::mode_change(backend);
    }
    std::cout << dsp56300::test::checks << " bit host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
