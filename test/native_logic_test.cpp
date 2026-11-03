// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using bfu_model::Function;
using bfu_model::Input;
constexpr Word add_a = 0x200040;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array functions{
    Function::bit_and, Function::bit_or, Function::bit_xor, Function::complement};
constexpr std::array registers{4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U};
constexpr std::array<std::uint64_t, 6> seeds{
    0, accumulator_mask, sign56, sign56 - 1, 0x123456abcdef12, 0x81fedcba654321};
constexpr std::uint64_t middle_mask = std::uint64_t{word_mask} << word_bits;

struct Machine {
  explicit Machine(const bool native)
      : core(
            {.program = p, .x = x, .y = y}, nullptr, {.backend = Backend::jit, .optimize = native}),
        optimize(native) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
  bool optimize;
};

void word_logic(Registers &state, const Registers &old, const Input input) {
  // The independent truth table uses old operands. Only A1/B1 is replaced:
  // parallel MOVE owns the extension and low word in the current destination.
  auto truth = state;
  bfu_model::apply(truth, old, input);
  auto &destination = input.destination_b ? state.b : state.a;
  const auto result = input.destination_b ? truth.b : truth.a;
  destination = (destination & ~middle_mask) | (result & middle_mask);
  state.sr = truth.sr;
  ++state.pc;
}

void arithmetic(Registers &state, const Word opcode) {
  if ((state.sr & bfu_model::sm) != 0) {
    saturation_model::alu(state, state, opcode);
  } else {
    sequence_model::alu(state, state, opcode);
  }
  ++state.pc;
}

void check(Machine &machine,
           const Registers &initial,
           const Word budget,
           const Registers &expected) {
  machine.core.set_registers(initial);
  const auto before = machine.core.jit_statistics();
  const auto progress = machine.core.progress();
  const auto result = machine.core.run(budget);
  const auto after = machine.core.jit_statistics();
  require(result.dispatches == budget && result.fault.kind == FaultKind::none &&
              machine.core.registers() == expected &&
              result.progress == Progress{.instructions = progress.instructions + budget,
                                          .estimated_cycles = progress.estimated_cycles + budget},
          "Word logic preserves independent per-bit results, full state, flags and prefix costs");
  require(after.native_alu_instructions - before.native_alu_instructions ==
                  (machine.optimize ? budget : 0) &&
              after.helper_instructions - before.helper_instructions ==
                  (machine.optimize ? 0 : budget),
          "Every admitted logic instruction actually uses native code or the selected fallback");
}

struct SingleSeed {
  std::uint64_t raw = 0;
  Word status = 0;
};

void single_case(Machine &machine, const Input input, const SingleSeed seed) {
  auto initial = register_model::seed(seed.status);
  initial.sr = seed.status;
  (input.destination_b ? initial.b : initial.a) = seed.raw;
  auto expected = initial;
  word_logic(expected, initial, input);
  check(machine, initial, 1, expected);
}

void single_patterns(Machine &machine, const Input input, const Word mode) {
  for (Word flags = 0; flags <= status_byte; ++flags) {
    single_case(machine, input, {.raw = seeds.back(), .status = mode | flags});
  }
  for (const Word flags : {0U, 0x55U, 0xaaU, status_byte}) {
    for (const auto raw : seeds) {
      single_case(machine, input, {.raw = raw, .status = mode | flags});
    }
    for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
      const auto raw = std::uint64_t{1} << bit;
      single_case(machine, input, {.raw = raw, .status = mode | flags});
      single_case(machine, input, {.raw = accumulator_mask ^ raw, .status = mode | flags});
    }
  }
}

void single_modes(Machine &machine, const Input input) {
  machine.core.write(Space::program, start, bfu_model::opcode(input));
  for (const auto numeric : modes) {
    for (const bool sm : {false, true}) {
      single_patterns(machine, input, numeric | (sm ? bfu_model::sm : 0));
    }
  }
}

void single_forms(const bool optimize) {
  Machine machine(optimize);
  for (const auto function : functions) {
    const unsigned sources = function == Function::complement ? 1U : 4U;
    for (unsigned source = 0; source < sources; ++source) {
      for (const bool destination_b : {false, true}) {
        const Input input{.function = function, .source = source, .destination_b = destination_b};
        single_modes(machine, input);
      }
    }
  }
  require(machine.x == std::array<Word, memory_words>{} && machine.x == machine.y,
          "Register-only word logic leaves all supplied data RAM intact");
}

struct Transfer {
  unsigned source = 4, destination = 4;
  Function function = Function::bit_and;
};

void transfer_case(Machine &machine, const Transfer move, const Word status) {
  const Input input{.function = move.function,
                    .source = (move.destination + 1) % 4,
                    .destination_b = (move.destination % 2) != 0};
  const Input suffix{.function = Function::complement, .destination_b = !input.destination_b};
  const auto opcode = bfu_model::opcode(input) | (move.source << 13) | (move.destination << 8);
  const std::array program{add_a, opcode, bfu_model::opcode(suffix)};
  machine.core.load(Space::program, start, program);
  auto initial = register_model::seed(status);
  initial.sr = status;
  auto expected = initial;
  arithmetic(expected, add_a);
  const auto old = expected;
  const auto word = register_model::read(expected, move.source);
  register_model::write(expected, move.destination, word);
  word_logic(expected, old, input);
  check(machine, initial, 2, expected);
  word_logic(expected, expected, suffix);
  check(machine, initial, 3, expected);
}

void transfer_modes(Machine &machine, const Transfer transfer) {
  for (const auto numeric : modes) {
    for (const bool sm : {false, true}) {
      for (const Word flags : {0U, status_byte}) {
        transfer_case(machine, transfer, numeric | flags | (sm ? bfu_model::sm : 0));
      }
    }
  }
}

void parallel_transfers(const bool optimize) {
  Machine machine(optimize);
  for (const auto source : registers) {
    for (const auto destination : registers) {
      for (const auto function : functions) {
        transfer_modes(machine,
                       {.source = source, .destination = destination, .function = function});
      }
    }
  }
}

void product_chains(const bool optimize) {
  constexpr Word mac = 0x2000d2;
  Machine machine(optimize);
  for (const auto function : functions) {
    const Input input{.function = function};
    const auto logic = bfu_model::opcode(input);
    const std::array program{mac, mac, mac, logic, mac, mac, mac, logic, mac, mac};
    machine.core.load(Space::program, start, program);
    for (const auto numeric : modes) {
      for (const bool sm : {false, true}) {
        const auto status = numeric | status_byte | (sm ? bfu_model::sm : 0);
        const auto initial = register_model::seed(status);
        auto expected = initial;
        Word budget = 0;
        for (const auto opcode : program) {
          if (opcode == logic) {
            word_logic(expected, expected, input);
          } else {
            arithmetic(expected, opcode);
          }
          check(machine, initial, ++budget, expected);
        }
      }
    }
  }
}

}
}

int main() {
  try {
    for (const bool optimize : {true, false}) {
      dsp56300::test::single_forms(optimize);
      dsp56300::test::parallel_transfers(optimize);
      dsp56300::test::product_chains(optimize);
    }
    std::cout << dsp56300::test::checks << " independent native word-logic checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
