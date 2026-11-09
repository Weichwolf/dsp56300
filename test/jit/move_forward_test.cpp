// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word io_address = 0xffff80, datum = 0xfedcba, selector_mask = 0x370000;
constexpr Word add_a_b = static_cast<Word>(isa::Opcode::add_a_b);
constexpr Word add_b_a = static_cast<Word>(isa::Opcode::add_b_a);

struct Transfer {
  unsigned source, destination;
  Word alu = 0;
};

Word encode(const Transfer transfer) {
  return static_cast<Word>(isa::Opcode::move_none) | (transfer.source << 13) |
         (transfer.destination << 8) | (transfer.alu & status_byte);
}

void model(Registers &state, const Transfer transfer) {
  const auto old = state;
  const auto value = register_model::read(state, transfer.source);
  register_model::write(state, transfer.destination, value);
  sequence_model::alu(state, old, transfer.alu);
  ++state.pc;
}

struct Machine {
  explicit Machine(Io *io = nullptr, const bool external = false)
      : core(external ? Memory::external() : Memory::separate({.program = p, .x = x, .y = y}), io) {
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct RegisterInput {
  unsigned target;
  Word mode;
};

void register_prefixes(const RegisterInput input) {
  const auto [target, mode] = input;
  Machine machine;
  const std::array transfers{
      Transfer{.source = 4, .destination = target},
      Transfer{.source = target, .destination = (target & 1U) == 0 ? 15U : 14U},
      Transfer{.source = 4, .destination = 14, .alu = add_a_b},
      Transfer{.source = 15, .destination = 12},
      Transfer{.source = 6, .destination = 13, .alu = add_b_a}};
  for (std::size_t index = 0; index < transfers.size(); ++index) {
    machine.p[start + index] = encode(transfers[index]);
  }
  machine.p[start + transfers.size()] = static_cast<Word>(isa::Opcode::jmp_absolute_short) |
                                        (start + static_cast<Word>(transfers.size()));
  const auto initial = register_model::seed(mode);
  for (unsigned pass = 0; pass < 2; ++pass) {
    for (Word count = 1; count <= transfers.size(); ++count) {
      const auto length = pass == 0 ? count : static_cast<Word>(transfers.size()) + 1 - count;
      auto expected = initial;
      for (Word index = 0; index < length; ++index) { model(expected, transfers[index]); }
      machine.core.set_registers(initial);
      const auto before = machine.core.progress();
      const auto run = machine.core.run(length);
      require(run.retired == length && run.fault.kind == FaultKind::none && !run.yielded &&
                  machine.core.registers() == expected &&
                  run.progress == Progress{.instructions = before.instructions + length,
                                           .estimated_cycles = before.estimated_cycles + length},
              "Cold/warm MOVE prefixes preserve full/partial values, old ALU inputs and flags");
    }
  }
  require(machine.x == std::array<Word, memory_words>{} && machine.y == machine.x,
          "Register forwarding leaves both borrowed data spaces unchanged");
}

class Port final : public Io {
public:
  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional{program[address]} : std::nullopt;
  }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space == Space::program) {
      const auto word = inspect_program(address);
      if (!word) { return false; }
      value = *word;
      return true;
    }
    ++reads;
    observed = core != nullptr && space == Space::x && address == io_address && pc == start + 2 &&
               core->registers() == expected && core->progress() == progress;
    value = datum;
    if (yield && core != nullptr) { core->request_yield(); }
    return accept && observed;
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

  Core *core = nullptr;
  std::span<const Word> program;
  Registers expected;
  Progress progress;
  unsigned reads = 0;
  bool accept = true, yield = false, observed = false;
};

struct Input {
  bool only_b = false, accept = true, yield = false, external = false;
};

void ram_to_io(const Input input) {
  Port port;
  Machine machine(&port, input.external);
  port.program = machine.p;
  port.core = &machine.core;
  port.accept = input.accept;
  port.yield = input.yield;
  const Transfer first{.source = 4, .destination = input.only_b ? 15U : 14U};
  constexpr Transfer second{.source = 6, .destination = 15};
  constexpr Transfer last{.source = 14, .destination = 5};
  constexpr Word load_a = (static_cast<Word>(isa::Opcode::move_x_effective) & ~selector_mask) |
                          (6U << 16) | (8U << 17) | (1U << 15) | (3U << 11);
  constexpr Word transfer = load_a | (add_a_b & status_byte);
  const std::array program{encode(first), encode(second), transfer, encode(last)};
  std::ranges::copy(program, machine.p.begin() + start);
  machine.p[start + program.size()] = static_cast<Word>(isa::Opcode::jmp_absolute_short) |
                                      (start + static_cast<Word>(program.size()));
  auto initial = register_model::seed(mode_up | flag_s);
  initial.r[0] = 7;
  machine.x[7] = datum;
  machine.core.set_registers(initial);
  auto wanted_warm = initial;
  model(wanted_warm, first);
  model(wanted_warm, second);
  if (!input.external) {
    const auto old = wanted_warm;
    ++wanted_warm.pc;
    ++wanted_warm.r[0];
    register_model::write(wanted_warm, 14, datum);
    sequence_model::alu(wanted_warm, old, transfer);
    model(wanted_warm, last);
  }
  const Word warm = input.external ? 2U : static_cast<Word>(program.size());
  require(machine.core.run(warm).retired == warm && port.reads == 0 &&
              machine.core.registers() == wanted_warm,
          "Warm native RAM forwarding matches complete state before its real observed transfer");

  initial.r[0] = io_address;
  auto expected = initial;
  model(expected, first);
  model(expected, second);
  const auto old = expected;
  // The callback sees the fetched opcode and updated post-increment cursor.
  ++expected.pc;
  ++expected.r[0];
  port.expected = expected;
  machine.core.set_registers(initial);
  const auto before = machine.core.progress();
  port.progress = {.instructions = before.instructions + 2,
                   .estimated_cycles = before.estimated_cycles + 2};
  const auto run = machine.core.run(static_cast<Word>(program.size()));
  require(port.reads == 1 && port.observed,
          "A warm guard failure publishes only completed incoming MOVE state before real I/O");
  if (!input.accept) {
    require(run.retired == 2 && run.fault.kind == FaultKind::io_read && run.fault.pc == start + 2 &&
                run.fault.address == io_address && run.progress == port.progress &&
                machine.core.registers() == expected,
            "Rejected I/O preserves fetched-PC/AGU effects without uninitialized cache writes");
    return;
  }
  register_model::write(expected, 14, datum);
  sequence_model::alu(expected, old, transfer);
  if (!input.yield) { model(expected, last); }
  const Word retired = input.yield ? 3U : 4U;
  require(run.retired == retired && run.yielded == input.yield &&
              run.fault.kind == FaultKind::none && machine.core.registers() == expected &&
              run.progress == Progress{.instructions = before.instructions + retired,
                                       .estimated_cycles = before.estimated_cycles + retired},
          "Accepted I/O preserves old parallel operands, fresh MOVE destinations and exact yield");
  if (input.yield) {
    model(expected, last);
    require(machine.core.run(1).retired == 1 && machine.core.registers() == expected &&
                port.reads == 1,
            "Resuming a yielded transfer consumes its fresh accumulator without repeated I/O");
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (unsigned target = 8; target <= 15; ++target) {
      for (const auto mode : {0U, mode_down, mode_up}) {
        register_prefixes({.target = target, .mode = mode});
      }
    }
    for (const bool external : {false, true}) {
      for (const bool only_b : {false, true}) {
        ram_to_io({.only_b = only_b, .external = external});
        ram_to_io({.only_b = only_b, .yield = true, .external = external});
        ram_to_io({.only_b = only_b, .accept = false, .external = external});
      }
    }
    std::cout << checks << " MOVE forwarding boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
