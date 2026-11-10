// SPDX-License-Identifier: GPL-3.0-only
// Independent public-API witnesses for held results and their flag obligations.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using enum isa::Opcode;
constexpr std::size_t storage_words = 512;
constexpr Word jump_extra_cycles = 2;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<Word, 6> words{0, 1, 2, sign24 - 1, sign24, word_mask};
constexpr std::array raw{
    std::uint64_t{0}, std::uint64_t{1}, sign56 - 1, sign56, sign56 + 1, accumulator_mask};
constexpr std::array averages{
    tfr_x0_a, add_y0_a, asr_a, rnd_a, tfr_x1_b, add_y1_b, asr_b, rnd_b, add_a_b};
constexpr std::array comparison{tfr_x0_a, cmp_y0_a, tfr_x1_b, add_y1_b};
constexpr std::array overflow{add_y0_a, clr_b, add_x0_b};
constexpr std::array carry_chain{tfr_x0_a, add_y0_a, rnd_a, clr_b, adc_x_b};
constexpr std::array unary{tfr_x0_a, neg_a, tfr_y0_b, abs_b, add_x1_a, add_y1_b};
constexpr std::array aligned_shifts{tfr_x0_a, add_y0_a, asr_a, asr_a, asr_a, rnd_a, rnd_a, adc_x_b};
constexpr std::array aligned_rounds{tfr_x0_a, rnd_a, rnd_a, tfr_y0_b, rnd_b, add_a_b};
constexpr std::array rounded_difference{tfr_x0_a, sub_y0_a, asr_a, rnd_a};
constexpr std::array raw_shifts{asr_a, asr_b, rnd_a, adc_x_b};
constexpr std::array fresh_products{mpy_register_a, mpyr_register_b, rnd_a, add_a_b};
constexpr std::array zero_products{clr_a, mac_register_a, clr_b, macr_register_b, adc_x_a};
constexpr std::array accumulating_products{
    tfr_x1_a, mpy_register_b, mac_register_a, clr_b, add_a_b};

struct Machine {
  explicit Machine(const std::span<const isa::Opcode> body)
      : core(Memory::separate({.program = p, .x = x, .y = y})) {
    std::vector<Word> code;
    for (const auto opcode : body) { code.push_back(static_cast<Word>(opcode)); }
    code.push_back(static_cast<Word>(jmp_absolute_short) | start);
    core.load(Space::program, start, code);
  }

  std::array<Word, storage_words> p{}, x{}, y{};
  Core core;
};

struct Input {
  Word mode = 0, left = 0, right = 0, flags = 0;
  std::size_t seed = 0;
  bool complement_y = false;
};

Registers initial(const Input input) {
  auto result = register_model::seed(input.mode);
  result.pc = start;
  result.sr = input.mode | input.flags;
  result.a = raw.at(input.seed % raw.size());
  result.b = raw.at((input.seed + 1) % raw.size());
  result.x = input.left | (std::uint64_t{input.right} << word_bits);
  result.y = input.right | (std::uint64_t{input.left} << word_bits);
  if (input.complement_y) {
    result.y = (input.left ^ word_mask) | (std::uint64_t{input.right} << word_bits);
  }
  return result;
}

void register_transfer(Registers &state, const Registers &old, const isa::Opcode opcode) {
  if (opcode == move_register_x0_y0) {
    state.y = (old.y & ~std::uint64_t{word_mask}) | (old.x & word_mask);
  } else if (opcode == move_register_x0_a_add_a_b || opcode == move_register_x0_b_add_b_a) {
    const auto word = static_cast<Word>(old.x) & word_mask;
    const auto value = signed24(word) * (std::int64_t{1} << word_bits);
    auto &destination = opcode == move_register_x0_a_add_a_b ? state.a : state.b;
    destination = static_cast<std::uint64_t>(value) & accumulator_mask;
  }
}

void arithmetic(Registers &state, const Registers &old, const isa::Opcode opcode) {
  register_transfer(state, old, opcode);
  if (opcode == asr_a || opcode == asr_b) {
    const bool destination_b = opcode == asr_b;
    shift_model::apply(state, old, {.source_b = destination_b, .destination_b = destination_b});
  } else if ((old.sr & saturation_model::mode) != 0) {
    saturation_model::alu(state, old, static_cast<Word>(opcode));
  } else {
    sequence_model::alu(state, old, static_cast<Word>(opcode));
  }
}

void check(Machine &machine,
           const std::span<const isa::Opcode> body,
           const Registers &state,
           const Word count) {
  auto expected = state;
  for (Word index = 0; index < count; ++index) {
    const auto position = index % (body.size() + 1);
    if (position == body.size()) {
      expected.pc = start;
      continue;
    }
    const auto old = expected;
    arithmetic(expected, old, body[position]);
    ++expected.pc;
  }
  machine.core.set_registers(state);
  const auto before = machine.core.state().progress;
  const auto memory = std::array{machine.p, machine.x, machine.y};
  const auto result = machine.core.run(count);
  const auto cycles = count + ((count / (body.size() + 1)) * jump_extra_cycles);
  const auto actual = machine.core.registers();
  if (actual != expected || result.retired != count || result.fault != Fault{}) {
    std::cerr << "count=" << count << " input SR/X/Y/A/B=" << state.sr << '/' << state.x << '/'
              << state.y << '/' << state.a << '/' << state.b
              << " expected SR/A/B/PC=" << expected.sr << '/' << expected.a << '/' << expected.b
              << '/' << expected.pc << " actual=" << actual.sr << '/' << actual.a << '/' << actual.b
              << '/' << actual.pc << " retired=" << result.retired
              << " fault=" << static_cast<unsigned>(result.fault.kind) << '\n';
  }
  require(result.retired == count && result.fault == Fault{} && !result.yielded &&
              result.state == ExecutionState::running && actual == expected &&
              result.progress == Progress{.instructions = before.instructions + count,
                                          .estimated_cycles = before.estimated_cycles + cycles} &&
              std::array{machine.p, machine.x, machine.y} == memory,
          "Held results preserve independent complete registers, sticky/live flags, RAM and every "
          "cold/warm prefix including compare and branch costs");
}

void numeric_mode(Machine &machine,
                  const std::span<const isa::Opcode> body,
                  const Word mode,
                  const bool complement_y) {
  const auto count = static_cast<Word>(body.size() + 1);
  for (std::size_t left = 0; left < words.size(); ++left) {
    for (std::size_t right = 0; right < words.size(); ++right) {
      for (const Word flags : {0U, status_byte}) {
        const auto state = initial({.mode = mode,
                                    .left = words[left],
                                    .right = words[right],
                                    .flags = flags,
                                    .seed = left + right,
                                    .complement_y = complement_y});
        check(machine, body, state, count);
        check(machine, body, state, count * 3);
        for (Word prefix = 0; prefix <= count; ++prefix) { check(machine, body, state, prefix); }
      }
    }
  }
}

void matrix(const std::span<const isa::Opcode> body, const bool complement_y = false) {
  Machine machine(body);
  for (const auto mode : modes) {
    numeric_mode(machine, body, mode, complement_y);
    numeric_mode(machine, body, mode | saturation_model::mode, complement_y);
  }
}

struct PairInput {
  bool subtract = false, high_first = false, y_first = false, b_first = false;
};

std::array<isa::Opcode, 8> pair_program(const PairInput input) {
  constexpr std::array sums{tfr_x0_a, add_x1_a, tfr_y0_b, add_y1_b, asr_a, asr_b, rnd_a, rnd_b};
  constexpr std::array differences{
      tfr_x0_a, sub_x1_a, tfr_y0_b, sub_y1_b, asr_a, asr_b, rnd_a, rnd_b};
  constexpr unsigned destination_shift = 3;
  constexpr unsigned bank_shift = 4;
  constexpr unsigned half_shift = 5;
  auto result = input.subtract ? differences : sums;
  const auto destination = Word{input.b_first} << destination_shift;
  const auto sources = (Word{input.y_first} << bank_shift) | (Word{input.high_first} << half_shift);
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto word = static_cast<Word>(result[index]);
    result[index] = static_cast<isa::Opcode>(word ^ destination ^ (index < 4 ? sources : 0));
  }
  return result;
}

void pair_reuse() {
  const auto first = pair_program({});
  const auto last =
      pair_program({.subtract = true, .high_first = true, .y_first = true, .b_first = true});
  for (const auto middle :
       {nop, move_register_x0_a_add_a_b, move_register_x0_b_add_b_a, move_register_x0_y0}) {
    std::vector<isa::Opcode> body(first.begin(), first.end());
    body.push_back(middle);
    body.insert(body.end(), last.begin(), last.end());
    matrix(body, true);
  }
}

void paired_words() {
  for (const bool subtract : {false, true}) {
    for (const bool high_first : {false, true}) {
      for (const bool y_first : {false, true}) {
        for (const bool b_first : {false, true}) {
          matrix(pair_program({.subtract = subtract,
                               .high_first = high_first,
                               .y_first = y_first,
                               .b_first = b_first}),
                 true);
        }
      }
    }
  }
  auto product = pair_program({});
  product[0] = mpyr_x0_y1_a;
  matrix(product, true);
  auto mixed = pair_program({});
  mixed[3] = sub_y1_b;
  matrix(mixed, true);
  auto reversed = pair_program({.subtract = true});
  reversed[2] = tfr_y1_b;
  reversed[3] = sub_y0_b;
  matrix(reversed, true);
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    paired_words();
    pair_reuse();
    matrix(averages);
    matrix(comparison);
    matrix(overflow);
    matrix(carry_chain);
    matrix(unary);
    matrix(aligned_shifts);
    matrix(aligned_rounds);
    matrix(rounded_difference);
    matrix(raw_shifts);
    matrix(fresh_products);
    matrix(zero_products);
    matrix(accumulating_products);
    std::cout << checks << " independent held-accumulator result checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
