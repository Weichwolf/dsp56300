// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word asr_a = 0x200022, asr_b = 0x20002a;
constexpr Word add_a_b = 0x18, add_b_a = 0x10, shift_b = 0x2a, shift_a = 0x22;
constexpr Word adc_x = 0x21, adc_y = 0x31, mac_a = 0xd2;
constexpr Word adc_x_a = 0x200021, mac_b = 0x2000fa, round_b = 0x200019;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array registers{4U, 5U, 6U, 7U, 14U, 15U};
constexpr std::array<std::uint64_t, 6> seeds{
    0, 1, sign56 - 1, sign56, accumulator_mask, 0x123456abcdef12};

struct Machine {
  explicit Machine(const bool optimize)
      : core({.program = p, .x = x, .y = y},
             nullptr,
             {.backend = Backend::jit, .optimize = optimize}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct ShiftInput {
  std::uint64_t raw = 0;
  Word mode = 0, flags = 0;
  bool destination_b = false, optimize = true;
};

void shift_case(Machine &machine, const ShiftInput input) {
  auto initial = register_model::seed(input.mode);
  initial.sr = input.mode | input.flags;
  (input.destination_b ? initial.b : initial.a) = input.raw;
  auto expected = initial;
  sequence_model::alu(expected, initial, input.destination_b ? asr_b : asr_a);
  ++expected.pc;
  machine.core.set_registers(initial);
  const auto before = machine.core.jit_statistics();
  const auto progress = machine.core.progress();
  const auto result = machine.core.run(1);
  const auto after = machine.core.jit_statistics();
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == expected &&
              result.progress.instructions == progress.instructions + 1 &&
              result.progress.estimated_cycles == progress.estimated_cycles + 1,
          "ASR preserves independent raw-56 floor division, shifted Carry and every status bit");
  require(after.native_alu_instructions ==
                  before.native_alu_instructions + (input.optimize ? 1U : 0U) &&
              after.helper_instructions == before.helper_instructions + (input.optimize ? 0U : 1U),
          "Every admitted single ASR actually executes native arithmetic or the selected fallback");
}

void single_shifts(const bool optimize) {
  Machine machine(optimize);
  std::vector<std::uint64_t> values(seeds.begin(), seeds.end());
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const auto position = std::uint64_t{1} << bit;
    values.push_back(position);
    values.push_back(accumulator_mask ^ position);
  }
  for (const bool destination_b : {false, true}) {
    machine.core.write(Space::program, start, destination_b ? asr_b : asr_a);
    for (const auto mode : modes) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto raw : values) {
          shift_case(machine,
                     {.raw = raw,
                      .mode = mode,
                      .flags = flags,
                      .destination_b = destination_b,
                      .optimize = optimize});
        }
      }
    }
  }
}

struct Transfer {
  unsigned source = 4, destination = 4, kind = 0;
};

Word parallel_alu(const Transfer input) {
  if (input.kind == 0) { return 0; }
  if (input.destination >= 14) {
    if (input.kind == 1) { return input.destination == 14 ? add_a_b : add_b_a; }
    if (input.kind == 2) { return input.destination == 14 ? shift_b : shift_a; }
    return input.destination == 14 ? 0x09U : 0x01U;
  }
  if (input.kind == 1) { return input.destination < 6 ? adc_x : adc_y; }
  if (input.kind == 2) { return mac_a; }
  constexpr std::array tfr{0x41U, 0x61U, 0x51U, 0x71U};
  return tfr.at(input.destination - 4);
}

std::array<Word, 6> chain(const Transfer input) {
  const auto move = parallel_none | (input.source << 13) | (input.destination << 8);
  return {move | parallel_alu(input), asr_a, asr_b, adc_x_a, mac_b, round_b};
}

Registers expected_chain(const Registers &initial, const Transfer input) {
  auto state = initial;
  const auto program = chain(input);
  for (std::size_t index = 0; index < program.size(); ++index) {
    const auto old = state;
    if (index == 0) {
      const auto word = register_model::read(state, input.source);
      register_model::write(state, input.destination, word);
    }
    sequence_model::alu(state, old, program[index]);
    ++state.pc;
  }
  return state;
}

struct ChainInput {
  Transfer transfer;
  Word mode = 0, flags = 0;
  std::size_t seed = 0;
  bool optimize = true;
};

void chain_case(Machine &machine, const ChainInput input) {
  constexpr Word length = 6;
  auto state = register_model::seed(input.mode);
  state.sr = input.mode | input.flags;
  state.a = seeds.at(input.seed);
  state.b = seeds.at((input.seed + 3) % seeds.size());
  const auto expected = expected_chain(state, input.transfer);
  machine.core.set_registers(state);
  const auto before = machine.core.jit_statistics();
  const auto progress = machine.core.progress();
  const auto result = machine.core.run(length);
  const auto after = machine.core.jit_statistics();
  require(
      result.dispatches == length && result.fault.kind == FaultKind::none &&
          machine.core.registers() == expected &&
          result.progress.instructions == progress.instructions + length &&
          result.progress.estimated_cycles == progress.estimated_cycles + length,
      "Register/ASR chains preserve independent clipping, sticky flags and old parallel operands");
  require(after.native_alu_instructions ==
                  before.native_alu_instructions + (input.optimize ? length : 0) &&
              after.helper_instructions ==
                  before.helper_instructions + (input.optimize ? 0 : length) &&
              after.native_move_instructions ==
                  before.native_move_instructions + (input.optimize ? 1U : 0U),
          "Every complete chain uses exactly six native retirements and one register transfer");
}

void register_chains(const bool optimize) {
  Machine machine(optimize);
  for (const auto source : registers) {
    for (const auto destination : registers) {
      for (unsigned kind = 0; kind < 4; ++kind) {
        const Transfer transfer{.source = source, .destination = destination, .kind = kind};
        const auto program = chain(transfer);
        machine.core.load(Space::program, start, program);
        for (const auto mode : modes) {
          for (Word flags = 0; flags <= status_byte; ++flags) {
            for (std::size_t seed = 0; seed < seeds.size(); ++seed) {
              chain_case(machine,
                         {.transfer = transfer,
                          .mode = mode,
                          .flags = flags,
                          .seed = seed,
                          .optimize = optimize});
            }
          }
        }
      }
    }
  }
  require(machine.x == std::array<Word, memory_words>{} && machine.y == machine.x,
          "Register-only lowering never mutates supplied X/Y RAM");
}

void mode_guards() {
  constexpr Word move_a_x0 = 0x21c400;
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down};
  for (const auto opcode : {asr_a, asr_b, move_a_x0}) {
    for (const auto mode : unsupported) {
      Machine machine(true);
      machine.core.write(Space::program, start, opcode);
      machine.core.set_registers(register_model::seed(mode));
      const auto result = machine.core.run(1);
      constexpr Word saturation = Word{1} << 20;
      if (mode == saturation && opcode != move_a_x0) {
        auto expected = register_model::seed(mode);
        const bool destination_b = opcode == asr_b;
        shift_model::apply(
            expected, expected, {.source_b = destination_b, .destination_b = destination_b});
        ++expected.pc;
        require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                    result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                    machine.core.registers() == expected &&
                    machine.core.jit_statistics().native_alu_instructions == 0 &&
                    machine.core.jit_statistics().helper_instructions == 1,
                "Qualified SM ASR uses exact general execution behind native rejection");
        continue;
      }
      if (mode == saturation && opcode == move_a_x0) {
        auto expected = register_model::seed(mode);
        const auto value = register_model::read(expected, 14);
        register_model::write(expected, 4, value);
        ++expected.pc;
        require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                    result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                    machine.core.registers() == expected &&
                    machine.core.jit_statistics().native_alu_instructions == 1 &&
                    machine.core.jit_statistics().helper_instructions == 0,
                "Qualified SM MOVE keeps exact transfer flags through native lowering");
        continue;
      }
      require(result.dispatches == 0 && result.progress == Progress{} &&
                  result.fault.kind == FaultKind::unsupported_mode &&
                  machine.core.jit_statistics().native_alu_instructions == 0,
              "Unqualified numerical modes retain the explicit general guard before retirement");
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const bool optimize : {true, false}) {
      dsp56300::test::single_shifts(optimize);
      dsp56300::test::register_chains(optimize);
    }
    dsp56300::test::mode_guards();
    std::cout << dsp56300::test::checks << " native ASR/register-transfer checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
