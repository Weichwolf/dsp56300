// SPDX-License-Identifier: GPL-3.0-only
// Public boundaries retain transient scaling events across later RAM transfers.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word length = 16, words = 64, sentinel = 0xab5a36c9;
constexpr Word x_input = 32, y_input = 48, y_output = 16;
constexpr std::array<Word, 8> palette{
    0, 1, 1U << 20, 1U << 21, 1U << 22, sign24 - 1, sign24, word_mask};

Word alu(const Word index, const bool multiply) {
  if (!multiply || (index & 1U) != 0) { return 0; }
  return static_cast<Word>((index & 2U) == 0 ? isa::Opcode::mac_register_a
                                             : isa::Opcode::mac_register_b) &
         status_byte;
}

Word encoding(const Word index, const bool multiply) {
  const bool y = (index & 2U) != 0;
  const auto form = y ? isa::Opcode::move_y_effective : isa::Opcode::move_x_effective;
  const Word reg = y ? 15U : 14U;
  const Word address = (y ? 4U : 0U) + ((index & 1U) != 0 ? 2U : 0U);
  constexpr Word selector_mask = 7U << 16;
  constexpr Word high_selector_mask = 24;
  constexpr unsigned high_selector_shift = 17;
  return (static_cast<Word>(form) & ~selector_mask) | ((reg & 7U) << 16) |
         ((reg & high_selector_mask) << high_selector_shift) | ((index & 1U) != 0 ? 1U << 15 : 0U) |
         (3U << 11) | (address << 8) | alu(index, multiply);
}

using Bank = std::array<Word, words>;

void model(Registers &state, Bank &x, Bank &y, const bool multiply) {
  const Word index = state.pc;
  const bool y_port = (index & 2U) != 0;
  const bool load = (index & 1U) != 0;
  const Word reg = y_port ? 15U : 14U;
  const Word address = (y_port ? 4U : 0U) + (load ? 2U : 0U);
  const auto old = state;
  auto &bank = y_port ? y : x;
  auto &cursor = state.r.at(address);
  if (load) {
    register_model::write(state, reg, bank.at(cursor) & word_mask);
  } else {
    bank.at(cursor) = register_model::read(state, reg);
  }
  ++cursor;
  const auto operation = alu(index, multiply);
  if (operation != 0) {
    if ((old.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, old, operation);
    } else {
      sequence_model::alu(state, old, operation);
    }
  }
  ++state.pc;
}

struct Machine {
  explicit Machine(const bool multiply) : core(Memory::separate({.program = p, .x = x, .y = y})) {
    for (Word index = 0; index < length; ++index) { p.at(index) = encoding(index, multiply); }
  }

  std::array<Word, length> p{};
  Bank x{}, y{};
  Core core;
};

void check(Machine &machine, const Registers &initial, const Word cut, const bool multiply) {
  machine.x.fill(sentinel);
  machine.y.fill(sentinel);
  for (std::size_t index = 0; index < 4; ++index) {
    machine.x.at(x_input + index) = sentinel & ~word_mask;
    machine.y.at(y_input + index) = (sentinel & ~word_mask) | palette.at(index + 4);
  }
  auto x = machine.x;
  auto y = machine.y;
  auto expected = initial;
  machine.core.set_registers(initial);
  const auto before = machine.core.state().progress;
  for (const Word end : {cut, length}) {
    const Word budget = end - expected.pc;
    if (budget == 0) { continue; }
    while (expected.pc < end) { model(expected, x, y, multiply); }
    const auto result = machine.core.run(budget);
    require(result.retired == budget && result.fault.kind == FaultKind::none &&
                result.state == ExecutionState::running && !result.yielded &&
                machine.core.registers() == expected && machine.x == x && machine.y == y &&
                result.progress == Progress{.instructions = before.instructions + end,
                                            .estimated_cycles = before.estimated_cycles + end},
            "Whole and split RAM runs expose every scaling event, all registers and memory");
    for (Word index = 0; index < length; ++index) {
      require(machine.p.at(index) == encoding(index, multiply), "RAM stores never alter P");
    }
  }
}

void matrix(const bool multiply) {
  Machine machine(multiply);
  constexpr std::array<std::uint64_t, 8> values{0,
                                                1,
                                                std::uint64_t{1} << 44,
                                                std::uint64_t{1} << 45,
                                                std::uint64_t{1} << 46,
                                                sign56 - 1,
                                                sign56,
                                                accumulator_mask};
  for (const Word mode :
       {0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up}) {
    for (const Word saturation : {0U, saturation_model::mode}) {
      for (const Word flags : {0U, flag_s, status_byte}) {
        for (const auto value : values) {
          auto initial = register_model::seed(mode | saturation | flags);
          initial.pc = 0;
          initial.sr = mode | saturation | flags;
          initial.a = initial.b = value;
          initial.m.fill(word_mask);
          initial.r[0] = 0;
          initial.r[2] = x_input;
          initial.r[4] = y_output;
          initial.r[6] = y_input;
          for (Word cut = 1; cut <= length; ++cut) { check(machine, initial, cut, multiply); }
        }
      }
    }
  }
}
}
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::matrix(false);
    dsp56300::test::matrix(true);
    std::cout << dsp56300::test::checks << " independent public growth-status checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
