// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-width integer/unsigned-compare models; no core execution/link.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word inc_base = 0x000008, dec_base = 0x00000a, cmpu_base = 0x0c1ff0;
constexpr Word repeat_base = 0x0600a0, loop_flag = Word{1} << 15;
constexpr Word saturation_mode = Word{1} << 20;
constexpr Word changed = flag_c | flag_v | flag_n | flag_z | flag_e | flag_u;
constexpr unsigned fractional_bits = 47;
constexpr Word stack_pointer_range = 64;
constexpr std::uint64_t negative_exp = 0x80, positive_exp = 0x7f;
constexpr std::uint64_t sign48 = std::uint64_t{1} << fractional_bits;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array values{std::uint64_t{0},
                            std::uint64_t{1},
                            std::uint64_t{2},
                            std::uint64_t{word_mask},
                            std::uint64_t{1} << word_bits,
                            sign48 - 1,
                            sign48,
                            sign56 - 2,
                            sign56 - 1,
                            sign56,
                            sign56 + 1,
                            accumulator_mask - 1,
                            accumulator_mask};
constexpr std::array long_values{std::uint64_t{0}, std::uint64_t{1}, sign48 - 1, sign48, long_mask};
constexpr std::array word_values{0U, 1U, sign24 - 1, sign24, word_mask};

struct UnitInput {
  std::uint64_t value = 0;
  Word status = 0;
  bool subtract = false, destination_b = false;
};

Word opcode(const UnitInput input) {
  return (input.subtract ? dec_base : inc_base) | (input.destination_b ? 1U : 0U);
}

void model_unit(Registers &state, const UnitInput input) {
  auto &destination = input.destination_b ? state.b : state.a;
  const auto old = destination;
  const auto full = signed56(old) + (input.subtract ? -1 : 1);
  destination = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto result = signed56(destination);
  const unsigned bits = fractional_bits + ((state.sr & mode_down) != 0 ? 1U : 0U) -
                        ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto boundary = std::int64_t{1} << bits;
  Word flags = result == 0 ? flag_z : 0;
  if (result < 0) { flags |= flag_n; }
  if (result < -boundary || result >= boundary) { flags |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((destination / divisor) % 2 == (destination / (divisor / 2)) % 2) { flags |= flag_u; }
  if (input.subtract ? old == 0 : old == accumulator_mask) { flags |= flag_c; }
  if (full < -static_cast<std::int64_t>(sign56) || full >= static_cast<std::int64_t>(sign56)) {
    flags |= flag_v | flag_l;
  }
  state.sr = (state.sr & ~changed) | flags;
}

BinaryCase unit(const UnitInput input, const Word repeat = 0) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.destination_b ? item.initial.b : item.initial.a) = input.value;
  item.expected = item.initial;
  std::vector<Word> program;
  const auto count = repeat == 0 ? 1 : repeat;
  if (repeat != 0) {
    program.push_back(repeat_base | ((repeat & status_byte) << 8) | ((repeat >> 8) & 15U));
  }
  program.push_back(opcode(input));
  for (Word iteration = 0; iteration < count; ++iteration) { model_unit(item.expected, input); }
  item.expected.pc += static_cast<Word>(program.size());
  item.budget = item.dispatches = count + (repeat == 0 ? 0U : 1U);
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = count + (repeat == 0 ? 0U : 5U)};
  item.before = {{.words = std::move(program), .address = start}};
  item.name = std::string(input.subtract ? "DEC" : "INC") +
              " raw-56 unit; repeat=" + std::to_string(repeat);
  return item;
}

struct LoopInput {
  UnitInput unit;
  Word pointer = 0, count = 1;
  bool outer = false;
};

BinaryCase loop(const LoopInput input) {
  auto item = unit(input.unit);
  auto &initial = item.initial;
  initial.sr |= loop_flag;
  initial.la = start;
  initial.lc = input.count;
  initial.sp = input.pointer;
  initial.sc = 2;
  constexpr Word saved_la = 0x70;
  constexpr Word saved_lc = 7;
  const auto saved_flag = input.outer ? loop_flag : 0;
  initial.stack[input.pointer % stack_entries] =
      (std::uint64_t{start} << word_bits) | saved_flag | flag_v;
  initial.stack[(input.pointer + stack_pointer_range - 1) % stack_entries] =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  model_unit(item.expected, input.unit);
  item.expected.pc = input.count > 1 ? start : start + 1;
  if (input.count > 1) {
    --item.expected.lc;
  } else {
    item.expected.lc = saved_lc;
    item.expected.la = saved_la;
    item.expected.sp = (input.pointer + stack_pointer_range - 2) % stack_pointer_range;
    item.expected.sc = 0;
    item.expected.sr = (item.expected.sr & ~loop_flag) | saved_flag;
  }
  item.name += "; hardware loop ending";
  return item;
}

struct CompareInput {
  Word selector = 0, status = 0;
  std::size_t left = 0, right = 0;
  bool destination_b = false;
};

BinaryCase compare(const CompareInput input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  const auto left = long_values.at(input.left);
  const auto right = input.selector == 0 ? long_values.at(input.right)
                                         : std::uint64_t{word_values.at(input.right)} << word_bits;
  auto &destination = input.destination_b ? item.initial.b : item.initial.a;
  destination = (negative_exp << (word_bits * 2)) | left;
  if (input.selector == 0) {
    (input.destination_b ? item.initial.a : item.initial.b) =
        (positive_exp << (word_bits * 2)) | right;
  } else {
    auto &bank = (input.selector & 1U) == 0 ? item.initial.x : item.initial.y;
    const auto shift = input.selector < 6 ? 0U : word_bits;
    bank = (bank & ~(std::uint64_t{word_mask} << shift)) |
           (std::uint64_t{word_values.at(input.right)} << shift);
  }
  item.expected = item.initial;
  const auto difference = static_cast<std::int64_t>(left) - static_cast<std::int64_t>(right);
  Word flags = difference == 0 ? flag_z : 0;
  if (difference < 0) { flags |= flag_n | flag_c; }
  // N is arithmetic bit 55; Z tests low 48 bits. EXP inputs are discarded.
  item.expected.sr = (item.expected.sr & ~(flag_n | flag_c | flag_v | flag_z)) | flags;
  ++item.expected.pc;
  const auto instruction = cmpu_base | (input.selector << 1) | (input.destination_b ? 1U : 0U);
  item.before = {{.words = {instruction}, .address = start}};
  item.name = "CMPU unsigned zero-extension; selector=" + std::to_string(input.selector);
  return item;
}

BinaryCase rejected(const Word instruction, const Word mode, const bool invalid = false) {
  BinaryCase item;
  item.initial = register_model::seed(mode);
  item.initial.sr = mode;
  item.expected = item.initial;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {.kind = invalid ? FaultKind::invalid_instruction : FaultKind::unsupported_mode,
                .space = Space::program,
                .address = start,
                .value = invalid ? instruction : mode,
                .pc = start};
  item.before = {{.words = {instruction}, .address = start}};
  item.name = "Explicit integer reserved-selector/unimplemented-mode rejection";
  return item;
}

void loop_cases(std::vector<BinaryCase> &cases, const UnitInput input) {
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const bool outer : {false, true}) {
        cases.push_back(loop({.unit = input, .pointer = pointer, .count = count, .outer = outer}));
      }
    }
  }
}

void unit_mode_cases(std::vector<BinaryCase> &cases, UnitInput input) {
  const auto mode = input.status;
  for (const auto status : {Word{0}, status_byte}) {
    input.status = mode | status;
    for (const auto value : values) {
      input.value = value;
      cases.push_back(unit(input));
    }
  }
  input.status = mode | flag_s;
  for (const auto value : {std::uint64_t{0}, sign56 - 1, sign56, accumulator_mask}) {
    input.value = value;
    for (const Word count : {1U, 2U, 7U}) { cases.push_back(unit(input, count)); }
    loop_cases(cases, input);
  }
}

void unit_cases(std::vector<BinaryCase> &cases) {
  for (const bool subtract : {false, true}) {
    for (const bool destination_b : {false, true}) {
      for (const auto mode : modes) {
        unit_mode_cases(cases,
                        {.status = mode, .subtract = subtract, .destination_b = destination_b});
      }
      constexpr Word repeat_zero = 65536;
      cases.push_back(
          unit({.value = sign56 - 1, .subtract = subtract, .destination_b = destination_b},
               repeat_zero));
      for (const Word mode : {0x2000U, 0x4000U, 0x20000U, saturation_mode, mode_up | mode_down}) {
        cases.push_back(
            rejected(opcode({.subtract = subtract, .destination_b = destination_b}), mode));
      }
    }
  }
}

void compare_operands(std::vector<BinaryCase> &cases, CompareInput input) {
  for (std::size_t left = 0; left < long_values.size(); ++left) {
    input.left = left;
    for (std::size_t right = 0; right < long_values.size(); ++right) {
      input.right = right;
      cases.push_back(compare(input));
    }
  }
}

void compare_status_cases(std::vector<BinaryCase> &cases, CompareInput input) {
  for (const bool destination_b : {false, true}) {
    input.destination_b = destination_b;
    for (const Word selector : {0U, 4U, 5U, 6U, 7U}) {
      input.selector = selector;
      compare_operands(cases, input);
    }
    for (const Word selector : {1U, 2U, 3U}) {
      cases.push_back(
          rejected(cmpu_base | (selector << 1) | (destination_b ? 1U : 0U), input.status, true));
    }
  }
}

void compare_cases(std::vector<BinaryCase> &cases) {
  for (const auto mode : modes) {
    for (const auto saturation : {Word{0}, saturation_mode}) {
      for (const auto status : {Word{0}, status_byte}) {
        compare_status_cases(cases, {.status = mode | saturation | status});
      }
    }
  }
  for (const Word mode : {0x2000U, 0x4000U, 0x20000U, mode_up | mode_down}) {
    cases.push_back(rejected(cmpu_base, mode));
  }
}

void model_cmpu_other(Registers &state) {
  const auto difference = static_cast<std::int64_t>(state.a & long_mask) -
                          static_cast<std::int64_t>(state.b & long_mask);
  Word flags = difference == 0 ? flag_z : 0;
  if (difference < 0) { flags |= flag_n | flag_c; }
  state.sr = (state.sr & ~(flag_n | flag_z | flag_v | flag_c)) | flags;
}

void mixed_sequences(std::vector<BinaryCase> &cases) {
  constexpr Word add_b_a = 0x200010;
  constexpr Word adc_x_a = 0x200021;
  constexpr Word mac_a = 0x2000d2;
  const std::vector program{add_b_a, inc_base, adc_x_a, dec_base | 1U, cmpu_base, mac_a};
  for (const auto mode : modes) {
    for (const auto value : values) {
      for (const auto status : {Word{0}, status_byte}) {
        BinaryCase item;
        item.initial = register_model::seed(mode | status);
        item.initial.sr = mode | status;
        item.initial.a = value;
        item.expected = item.initial;
        item.before = {{.words = program, .address = start}};
        item.name = "Native arithmetic/integer-helper/CMPU/native arithmetic prefixes";
        for (std::size_t index = 0; index < program.size(); ++index) {
          const auto instruction = program[index];
          if (index == 1 || index == 3) {
            model_unit(item.expected, {.subtract = index == 3, .destination_b = index == 3});
          } else if (index == 4) {
            model_cmpu_other(item.expected);
          } else {
            const auto old = item.expected;
            sequence_model::alu(item.expected, old, instruction);
          }
          ++item.expected.pc;
          item.budget = item.dispatches = static_cast<Word>(index + 1);
          item.progress = {.instructions = item.budget, .estimated_cycles = item.budget};
          cases.push_back(item);
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_integer OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::unit_cases(cases);
    dsp56300::test::compare_cases(cases);
    dsp56300::test::mixed_sequences(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent integer/unsigned-compare cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
