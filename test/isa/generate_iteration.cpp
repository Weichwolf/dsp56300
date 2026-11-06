// SPDX-License-Identifier: GPL-3.0-only
// Independent LSB-first adder circuit and raw-coordinate normalization model.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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
using Cases = std::vector<BinaryCase>;
using Bits = std::array<bool, accumulator_bits>;
enum class Kind : std::uint8_t { divide, normalize };
constexpr Word div_base = 0x018040, norm_base = 0x01d815, repeat_base = 0x0600a0;
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, pointer_range = 64;
constexpr Word ignored_modifier = 0xab8000;
constexpr Word arithmetic_flags = flag_n | flag_z | flag_e | flag_u | flag_v;
constexpr Word compatibility = Word{1} << 13;
constexpr std::uint64_t word_unit = std::uint64_t{1} << word_bits;
constexpr std::uint64_t long_sign = std::uint64_t{1} << 47, shift_sign = std::uint64_t{1} << 54;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array source_code{0U, 2U, 1U, 3U};
constexpr std::array<std::uint64_t, 16> values{0,
                                               1,
                                               word_mask,
                                               word_unit,
                                               long_sign - 1,
                                               long_sign,
                                               shift_sign - 1,
                                               shift_sign,
                                               shift_sign + 1,
                                               sign56 - 1,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask - 1,
                                               accumulator_mask,
                                               0x123456789abc,
                                               0xff123456789abc};
constexpr std::array divisors{0U, 1U, 2U, 0x123456U, sign24 - 1, sign24, sign24 + 1, word_mask};

struct Input {
  Kind kind = Kind::divide;
  std::uint64_t value = 0;
  Word source = 1, status = 0, exponent = word_mask;
  unsigned selector = 0;
  bool destination_b = false;
};

Bits bits(std::uint64_t value) {
  Bits result{};
  for (auto &bit : result) {
    bit = value % 2 != 0;
    value /= 2;
  }
  return result;
}

std::uint64_t number(const Bits &value) {
  std::uint64_t result = 0;
  for (std::size_t index = value.size(); index != 0; --index) {
    result = result * 2 + (value[index - 1] ? 1U : 0U);
  }
  return result;
}

struct Circuit {
  Bits left{}, right{};
  bool subtract = false;
};

Bits add(const Circuit &input) {
  Bits result{};
  int carry = 0;
  for (std::size_t index = 0; index < result.size(); ++index) {
    const int a = input.left[index] ? 1 : 0;
    const int b = input.right[index] ? 1 : 0;
    const int full = input.subtract ? a - b - carry : a + b + carry;
    result[index] = ((full + 2) % 2) != 0;
    carry = input.subtract ? static_cast<int>(full < 0) : full / 2;
  }
  return result;
}

Word source(const Registers &state, const unsigned selector) {
  const auto bank = selector < 2 ? state.x : state.y;
  return static_cast<Word>(bank >> (selector % 2 == 0 ? 0U : word_bits)) & word_mask;
}

void seed_source(Registers &state, const Input input) {
  auto &bank = input.selector < 2 ? state.x : state.y;
  const auto shift = input.selector % 2 == 0 ? 0U : word_bits;
  bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{input.source} << shift);
}

Word opcode(const Input input) {
  const auto base = input.kind == Kind::divide ? div_base | (source_code[input.selector] << 4)
                                               : norm_base | (input.selector << 8);
  return base | (input.destination_b ? 8U : 0U);
}

void divide_model(Registers &state, const Input input) {
  auto &destination = input.destination_b ? state.b : state.a;
  const auto old = bits(destination);
  const auto divisor = source(state, input.selector);
  Bits shifted{};
  shifted[0] = (state.sr & flag_c) != 0;
  for (std::size_t index = 1; index < shifted.size(); ++index) { shifted[index] = old[index - 1]; }
  Bits operand{};
  for (unsigned index = word_bits; index < accumulator_bits; ++index) {
    operand[index] =
        index < word_bits * 2 ? ((divisor >> (index - word_bits)) % 2) != 0 : divisor >= sign24;
  }
  const auto result =
      add({.left = shifted, .right = operand, .subtract = old.back() == (divisor >= sign24)});
  destination = number(result);
  state.sr &= ~(flag_c | flag_v);
  if (!result.back()) { state.sr |= flag_c; }
  if (old.back() != shifted.back()) { state.sr |= flag_v | flag_l; }
  const auto limited = saturation_model::apply(destination, {.status = state.sr});
  destination = limited.value;
  if (limited.limited) { state.sr |= flag_v | flag_l; }
}

Word result_flags(const Registers &state, const bool destination_b) {
  const auto raw = destination_b ? state.b : state.a;
  const unsigned point =
      47U + ((state.sr & mode_down) != 0 ? 1U : 0U) - ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto boundary = std::int64_t{1} << point;
  const auto value = signed56(raw);
  Word flags = raw == 0 ? flag_z : 0;
  if (value < 0) { flags |= flag_n; }
  if (value < -boundary || value >= boundary) { flags |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { flags |= flag_u; }
  return flags;
}

void normalize_model(Registers &state, const Input input) {
  const bool right = (state.sr & flag_e) != 0;
  const bool left = !right && (state.sr & flag_u) != 0 && (state.sr & flag_z) == 0;
  auto &destination = input.destination_b ? state.b : state.a;
  const auto original = destination;
  if (right) {
    const auto value = signed56(original);
    auto half = value / 2;
    if (value % 2 < 0) { --half; }
    destination = static_cast<std::uint64_t>(half) & accumulator_mask;
  } else if (left) {
    destination = (original * 2) % (std::uint64_t{1} << accumulator_bits);
  }
  const bool overflowed = left && (original >= sign56) != (destination >= sign56);
  const auto limited = saturation_model::apply(destination, {.status = state.sr});
  destination = limited.value;
  Word flags = result_flags(state, input.destination_b);
  if (overflowed || limited.limited) { flags |= flag_v | flag_l; }
  state.sr = (state.sr & ~arithmetic_flags) | flags;
  if (right || left) {
    const auto address_bits = (state.sr & compatibility) != 0 ? 16U : word_bits;
    const auto range = std::int64_t{1} << address_bits;
    const auto next = static_cast<std::int64_t>(state.r.at(input.selector)) + (left ? -1 : 1);
    state.r.at(input.selector) = static_cast<Word>((next + range) % range);
  }
}

void model(Registers &state, const Input input) {
  if (input.kind == Kind::divide) {
    divide_model(state, input);
  } else {
    normalize_model(state, input);
  }
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.destination_b ? item.initial.b : item.initial.a) = input.value;
  if (input.kind == Kind::divide) {
    seed_source(item.initial, input);
  } else {
    item.initial.r.at(input.selector) = input.exponent;
    item.initial.m.at(input.selector) = ignored_modifier;
    item.initial.n.at(input.selector) = sign24;
  }
  item.expected = item.initial;
  model(item.expected, input);
  ++item.expected.pc;
  item.progress.estimated_cycles = input.kind == Kind::divide ? 1U : 5U;
  item.before = {{.words = {opcode(input)}, .address = start}};
  item.name = std::string(input.kind == Kind::divide ? "DIV raw bit iteration"
                                                     : "NORM flags-controlled iteration") +
              "; selector=" + std::to_string(input.selector);
  return item;
}

void divide_operands(Cases &cases, Input input) {
  for (const auto value : values) {
    input.value = value;
    for (const auto divisor : divisors) {
      input.source = divisor;
      cases.push_back(single(input));
    }
  }
}

void divide_matrix(Cases &cases) {
  for (unsigned selector = 0; selector < 4; ++selector) {
    for (const bool destination_b : {false, true}) {
      for (const auto mode : modes) {
        for (const Word flags : {0U, status_byte, flag_c | flag_l, flag_s | flag_v}) {
          divide_operands(
              cases,
              {.status = mode | flags, .selector = selector, .destination_b = destination_b});
        }
      }
    }
  }
}

void normalize_values(Cases &cases, Input input) {
  for (const auto value : values) {
    input.value = value;
    auto item = single(input);
    // Seed E/U/Z/N from the operand for defined normalization-algorithm inputs.
    item.initial.sr = input.status | result_flags(item.initial, input.destination_b);
    item.expected = item.initial;
    normalize_model(item.expected, input);
    ++item.expected.pc;
    cases.push_back(std::move(item));
  }
}

void normalize_modes(Cases &cases, const Input input) {
  for (const auto mode : modes) {
    for (const Word flags :
         {0U, flag_c | flag_l, flag_s | flag_v, flag_s | flag_c | flag_v | flag_l}) {
      auto sample = input;
      sample.status |= mode | flags;
      normalize_values(cases, sample);
    }
  }
}

void normalize_matrix(Cases &cases) {
  for (unsigned selector = 0; selector < agu_registers; ++selector) {
    for (const bool destination_b : {false, true}) {
      for (const bool compatible : {false, true}) {
        normalize_modes(cases,
                        {.kind = Kind::normalize,
                         .status = compatible ? compatibility : 0U,
                         .exponent = selector % 2 == 0 ? 0U : word_mask,
                         .selector = selector,
                         .destination_b = destination_b});
      }
    }
  }
}

struct Repetition {
  Word count = 1, budget = 1;
};

BinaryCase repeated(const Input input, const Repetition repetition) {
  const auto count = repetition.count;
  const auto budget = repetition.budget;
  auto item = single(input);
  item.expected = item.initial;
  item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
  const auto iterations = budget - 1;
  for (Word step = 0; step < iterations; ++step) { model(item.expected, input); }
  const bool completed = iterations == count;
  item.expected.pc = start + (completed ? 2U : 1U);
  item.expected.lc = completed ? item.initial.lc : count - iterations;
  item.interruptible = completed;
  item.budget = item.dispatches = budget;
  const auto cost = input.kind == Kind::divide ? 1U : 5U;
  item.progress = {.instructions = budget, .estimated_cycles = 5 + (cost * iterations)};
  item.name += "; exact REP prefix and current Carry/CCR dependence";
  return item;
}

void quotient_cases(Cases &cases) {
  constexpr std::array magnitudes{1U, 2U, 3U, 0x123456U, 0x400000U, sign24 - 1};
  constexpr Word precision = 24;
  for (const auto magnitude : magnitudes) {
    for (const bool negative : {false, true}) {
      const Word divisor = negative ? static_cast<Word>(word_unit - magnitude) : magnitude;
      for (const Word denominator : {2U, 3U, 7U}) {
        const auto dividend = (std::uint64_t{magnitude} * word_unit) / denominator;
        const Input input{.value = dividend,
                          .source = divisor,
                          .status = flag_s | flag_z,
                          .selector = magnitude % 4,
                          .destination_b = negative};
        for (const Word budget : {1U, 2U, 8U, precision + 1}) {
          auto item = repeated(input, {.count = precision, .budget = budget});
          if (budget == precision + 1) {
            const auto raw = input.destination_b ? item.expected.b : item.expected.a;
            const auto quotient_with_next_bit =
                ((raw % word_unit) * 2) + (item.expected.sr & flag_c);
            require(quotient_with_next_bit == dividend / magnitude,
                    "24 DIV iterations agree with ordinary integer division including the next "
                    "Carry bit");
          }
          cases.push_back(std::move(item));
        }
      }
    }
  }
}

void normalize_repeats(Cases &cases) {
  constexpr Word count = 64;
  for (const auto mode : modes) {
    for (const auto value : {std::uint64_t{0}, std::uint64_t{1}, accumulator_mask, sign56}) {
      Input input{.kind = Kind::normalize, .value = value, .status = mode | flag_c | flag_l};
      auto seeded = single(input);
      input.status |= result_flags(seeded.initial, false);
      for (const Word budget : {1U, 2U, 8U, count + 1}) {
        cases.push_back(repeated(input, {.count = count, .budget = budget}));
      }
    }
  }
}

void control_flag_cases(Cases &cases) {
  for (const Word control : {0U, flag_e, flag_u, flag_u | flag_z}) {
    for (const auto value : {std::uint64_t{0}, shift_sign, sign56, accumulator_mask}) {
      for (const bool destination_b : {false, true}) {
        cases.push_back(single({.kind = Kind::normalize,
                                .value = value,
                                .status = control | flag_c | flag_s | flag_v,
                                .destination_b = destination_b}));
      }
    }
  }
}

struct Frame {
  Word pointer = 0, count = 1, outer = 0;
};

BinaryCase ending(const Input input, const Frame frame) {
  constexpr Word saved_la = 0x70;
  constexpr Word saved_lc = 7;
  auto item = single(input);
  auto &initial = item.initial;
  initial.sr |= lf;
  initial.la = start;
  initial.lc = frame.count;
  initial.sp = frame.pointer;
  initial.sc = 2;
  initial.stack[frame.pointer % stack_entries] =
      (std::uint64_t{start} << word_bits) | frame.outer | flag_v;
  initial.stack[(frame.pointer + pointer_range - 1) % stack_entries] =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  model(item.expected, input);
  item.expected.pc = frame.count > 1 ? start : start + 1;
  if (frame.count > 1) {
    --item.expected.lc;
  } else {
    item.expected.la = saved_la;
    item.expected.lc = saved_lc;
    item.expected.sp = (frame.pointer + pointer_range - 2) % pointer_range;
    item.expected.sc = 0;
    item.expected.sr = (item.expected.sr & ~(lf | fv)) | frame.outer;
  }
  return item;
}

void interactions(Cases &cases) {
  for (const auto kind : {Kind::divide, Kind::normalize}) {
    const Input input{.kind = kind,
                      .value = shift_sign,
                      .source = sign24 + 1,
                      .status = flag_u | flag_c,
                      .selector = 2,
                      .destination_b = true};
    for (const Word pointer : {0U, 15U, 31U, 63U}) {
      for (const Word count : {1U, 2U}) {
        for (const Word outer : {0U, lf, fv, lf | fv}) {
          cases.push_back(ending(input, {.pointer = pointer, .count = count, .outer = outer}));
        }
      }
    }
  }
}

struct Step {
  Input input{};
  Word instruction = 0;
  bool iterative = false;
};

void sequence_prefixes(Cases &cases) {
  constexpr Word mpy_a = 0x2000d0;
  constexpr Word adc_x_a = 0x200021;
  constexpr Word adc_y_b = 0x200039;
  constexpr Word mac_a = 0x2000d2;
  const std::array steps{
      Step{.instruction = mpy_a},
      Step{.input = {.kind = Kind::normalize}, .iterative = true},
      Step{.instruction = adc_x_a},
      Step{.input = {.kind = Kind::divide, .selector = 2}, .iterative = true},
      Step{.instruction = adc_y_b},
      Step{.input = {.kind = Kind::normalize, .selector = 1, .destination_b = true},
           .iterative = true},
      Step{.instruction = mac_a}};
  std::vector<Word> program;
  program.reserve(steps.size() * 2);
  for (const auto step : steps) {
    program.push_back(step.iterative ? opcode(step.input) : step.instruction);
  }
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      BinaryCase item;
      item.initial = register_model::seed(mode | flags);
      item.initial.sr = mode | flags;
      item.initial.m.fill(ignored_modifier);
      item.expected = item.initial;
      item.budget = 0;
      item.progress = {};
      item.before = {{.words = program, .address = start}};
      item.name =
          "Native multiply/NORM/DIV/Carry consumers: every prefix, flags and exponent updates";
      for (const auto step : steps) {
        if (step.iterative) {
          model(item.expected, step.input);
        } else {
          const auto previous = item.expected;
          sequence_model::alu(item.expected, previous, step.instruction);
        }
        ++item.expected.pc;
        ++item.budget;
        item.dispatches = item.budget;
        item.progress.instructions = item.budget;
        item.progress.estimated_cycles +=
            step.iterative && step.input.kind == Kind::normalize ? 5U : 1U;
        cases.push_back(item);
      }
    }
  }
}

void saturation_cases(Cases &cases) {
  for (const auto kind : {Kind::divide, Kind::normalize}) {
    for (const bool destination_b : {false, true}) {
      for (const auto value : values) {
        for (const Word flags : {0U, status_byte, flag_e | flag_c, flag_u | flag_l}) {
          for (const auto scaling : modes) {
            cases.push_back(single({.kind = kind,
                                    .value = value,
                                    .source = sign24,
                                    .status = saturation_model::mode | scaling | flags,
                                    .destination_b = destination_b}));
          }
        }
      }
    }
  }
}

void faults(Cases &cases) {
  for (const auto kind : {Kind::divide, Kind::normalize}) {
    for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down}) {
      BinaryCase item;
      const Input input{.kind = kind, .status = mode};
      item.initial = register_model::seed(mode);
      item.initial.sr = mode;
      item.expected = item.initial;
      item.before = {{.words = {opcode(input)}, .address = start}};
      item.dispatches = 0;
      item.progress = {};
      item.interruptible = false;
      item.fault = {.kind = FaultKind::unsupported_mode,
                    .space = Space::program,
                    .address = start,
                    .value = mode,
                    .pc = start};
      item.name = "DIV/NORM unimplemented numeric mode is explicit";
      if (mode == saturation_model::mode || (kind == Kind::normalize && mode == compatibility)) {
        model(item.expected, input);
        ++item.expected.pc;
        item.dispatches = 1;
        item.progress = {.instructions = 1, .estimated_cycles = kind == Kind::divide ? 1U : 5U};
        item.interruptible = true;
        item.fault = {};
        item.name = "DIV/NORM qualified SM or SC mode preserves architectural semantics";
      }
      cases.push_back(std::move(item));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_iteration OUTPUT.d56"); }
    dsp56300::test::Cases cases;
    dsp56300::test::divide_matrix(cases);
    dsp56300::test::normalize_matrix(cases);
    dsp56300::test::quotient_cases(cases);
    dsp56300::test::normalize_repeats(cases);
    dsp56300::test::control_flag_cases(cases);
    dsp56300::test::interactions(cases);
    dsp56300::test::sequence_prefixes(cases);
    dsp56300::test::saturation_cases(cases);
    dsp56300::test::faults(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent DIV/NORM cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
