// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturated_normalization_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word division = 0x018040, normalization = 0x01d815, fast = 0x0c1e20;
constexpr Word compatibility = Word{1} << 13;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array<std::uint64_t, 14> values{0,
                                               1,
                                               word_mask,
                                               std::uint64_t{1} << word_bits,
                                               saturation_model::positive,
                                               saturation_model::positive + 1,
                                               saturation_model::negative - 1,
                                               saturation_model::negative,
                                               sign56 - 1,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask,
                                               0x123456abcdef12,
                                               0xff123456789abc};
constexpr std::array divisors{0U, 1U, 2U, 0x123456U, sign24 - 1, sign24, sign24 + 1, word_mask};
constexpr std::array selected_flags{0U, flag_c | flag_v | flag_s, status_byte};

Registers seed(const Word status) {
  auto result = register_model::seed(status);
  result.sr = status;
  return result;
}

struct Coordinate {
  Word word;
  unsigned bit;
};

bool operand_bit(const Coordinate input) {
  if (input.bit < word_bits) { return false; }
  if (input.bit >= word_bits * 2) { return input.word >= sign24; }
  return ((input.word >> (input.bit - word_bits)) % 2) != 0;
}

void divide_model(Registers &state, const unsigned source, const bool destination_b) {
  auto &target = destination_b ? state.b : state.a;
  std::array<bool, accumulator_bits> old{};
  for (std::size_t bit = 0; bit < old.size(); ++bit) { old[bit] = ((target >> bit) % 2) != 0; }
  const auto bank = source < 2 ? state.x : state.y;
  const auto operand = static_cast<Word>(bank >> ((source % 2) * word_bits)) & word_mask;
  const bool subtract = old.back() == (operand >= sign24);
  bool carry = false;
  bool sign = false;
  target = 0;
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const bool left = bit == 0 ? (state.sr & flag_c) != 0 : old[bit - 1];
    const bool right = operand_bit({.word = operand, .bit = bit});
    const int a = left ? 1 : 0;
    const int b = right ? 1 : 0;
    const int c = carry ? 1 : 0;
    const int full = subtract ? a - b - c : a + b + c;
    const bool output = ((full + 2) % 2) != 0;
    carry = subtract ? full < 0 : full >= 2;
    if (output) { target |= std::uint64_t{1} << bit; }
    sign = output;
  }
  state.sr &= ~(flag_c | flag_v);
  if (!sign) { state.sr |= flag_c; }
  if (old.back() != old.at(old.size() - 2)) { state.sr |= flag_v | flag_l; }
  const auto limited = saturation_model::apply(target, {.status = state.sr});
  target = limited.value;
  if (limited.limited) { state.sr |= flag_v | flag_l; }
}

void check(const a64::Program &program,
           const Registers &initial,
           const a64::Expected &expected,
           const Word opcode) {
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, opcode);
}

struct Division {
  unsigned source;
  std::uint64_t value;
  Word divisor;
  bool destination_b;
};

void divide_case(const a64::Program &program, Registers initial, const Division input) {
  (input.destination_b ? initial.b : initial.a) = input.value;
  auto &bank = input.source < 2 ? initial.x : initial.y;
  const auto shift = (input.source % 2) * word_bits;
  bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{input.divisor} << shift);
  a64::Expected expected{.reg = initial};
  divide_model(expected.reg, input.source, input.destination_b);
  ++expected.reg.pc;
  check(program, initial, expected, division);
}

void divide(const Word mode) {
  constexpr std::array codes{0U, 2U, 1U, 3U};
  for (unsigned source = 0; source < codes.size(); ++source) {
    for (const bool destination_b : {false, true}) {
      const auto opcode = division | (codes.at(source) << 4) | (destination_b ? 8U : 0U);
      const a64::Program program({.opcode = opcode, .mode = mode});
      for (const auto flags : selected_flags) {
        for (const auto value : values) {
          for (const auto divisor : divisors) {
            divide_case(program,
                        seed(mode | flags),
                        {.source = source,
                         .value = value,
                         .divisor = divisor,
                         .destination_b = destination_b});
          }
        }
      }
    }
  }
}

void normalize(const Word mode) {
  constexpr std::array exponents{0U, 0xffffU, 0x10000U, word_mask};
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (const bool destination_b : {false, true}) {
      const normalization_model::Input input{.kind = normalization_model::Kind::iteration,
                                             .selector = index,
                                             .destination_b = destination_b};
      const auto opcode = normalization_model::opcode(input);
      const a64::Program program({.opcode = opcode, .mode = mode, .cycles = 5});
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto value : values) {
          auto initial = seed(mode | flags);
          (destination_b ? initial.b : initial.a) = value;
          initial.r.at(index) = exponents.at(flags % exponents.size());
          constexpr Word ignored_modifier = 0xabc001;
          initial.m.at(index) = ignored_modifier;
          a64::Expected expected{.reg = initial, .cycles = 5};
          normalization_model::apply(expected.reg, input);
          ++expected.reg.pc;
          check(program, initial, expected, opcode);
        }
      }
    }
  }
}

struct Count {
  unsigned selector;
  int value;
};

void count_word(Registers &state, const Count input) {
  constexpr std::array banks{0U, 1U, 2U, 2U, 3U, 3U};
  const std::array targets{&state.a, &state.b, &state.x, &state.y};
  auto &target = *targets.at(banks.at(input.selector));
  const auto shift = input.selector < 2 || (input.selector & 1U) != 0 ? word_bits : 0U;
  target = (target & ~(std::uint64_t{word_mask} << shift)) |
           (std::uint64_t{static_cast<Word>(input.value) & word_mask} << shift);
}

void fast_case(const a64::Program &program,
               const normalization_model::Input input,
               const Registers &initial) {
  const auto count = signed24(normalization_model::source(initial, input.selector));
  a64::Expected expected{.reg = initial};
  if (count >= normalization_model::minimum_count && count <= normalization_model::maximum_count) {
    normalization_model::apply(expected.reg, input);
  } else {
    expected.retired = 0;
    expected.cycles = 0;
    expected.repeat_completed = true;
    expected.boundary = detail::NativeBoundary::owner;
    expected.failure = {.kind = FaultKind::unsupported_mode,
                        .space = Space::program,
                        .address = start,
                        .value = normalization_model::opcode(input),
                        .pc = start};
  }
  ++expected.reg.pc;
  check(program, initial, expected, normalization_model::opcode(input));
}

void normalize_fast(const Word mode) {
  constexpr std::array counts{-8388608, -56, -55, -54, -48, -32, -24, -1, 0,  1,      23,
                              24,       31,  32,  47,  48,  54,  55,  56, 57, 8388607};
  for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      const normalization_model::Input input{.selector = selector, .destination_b = destination_b};
      const a64::Program program({.opcode = normalization_model::opcode(input), .mode = mode});
      for (const auto flags : selected_flags) {
        for (const auto value : values) {
          for (const int count : counts) {
            auto initial = seed(mode | flags);
            (destination_b ? initial.b : initial.a) = value;
            count_word(initial, {.selector = selector, .value = count});
            fast_case(program, input, initial);
          }
        }
        for (int count = normalization_model::minimum_count;
             count <= normalization_model::maximum_count;
             ++count) {
          auto initial = seed(mode | flags);
          count_word(initial, {.selector = selector, .value = count});
          fast_case(program, input, initial);
        }
      }
    }
  }
}

void sequence_step(Registers &state, const std::size_t index) {
  switch (index) {
    case 0: saturation_model::unit(state, false, false); break;
    case 1: normalization_model::apply(state, {.selector = 0, .destination_b = true}); break;
    case 2: divide_model(state, 2, false); break;
    case 3:
      normalization_model::apply(
          state,
          {.kind = normalization_model::Kind::iteration, .selector = 3, .destination_b = true});
      break;
    case 4: saturation_model::unit(state, true, false); break;
    default: require(false, "Unknown authored iteration sequence step"); break;
  }
  ++state.pc;
}

void sequences(const Word mode) {
  constexpr Word inc = 8;
  constexpr Word dec = 10;
  const std::array inputs{
      a64::Instruction{.opcode = inc, .pc = start, .mode = mode},
      a64::Instruction{.opcode = fast | 5U, .pc = start + 1, .mode = mode},
      a64::Instruction{.opcode = division | 16U, .pc = start + 2, .mode = mode},
      a64::Instruction{
          .opcode = normalization | (3U << 8) | 8U, .pc = start + 3, .mode = mode, .cycles = 5},
      a64::Instruction{.opcode = dec, .pc = start + 4, .mode = mode}};
  const a64::Program program(inputs);
  for (const auto flags : selected_flags) {
    for (const auto value : values) {
      auto initial = seed(mode | flags);
      initial.a = word_mask;
      initial.b = value;
      initial.r[3] = word_mask;
      for (const Word budget : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 32U}) {
        const auto count = std::min<std::size_t>(budget, inputs.size());
        a64::Expected expected{.reg = initial,
                               .retired = static_cast<Word>(count),
                               .cycles = 0,
                               .inhibited = count == 0,
                               .repeat_completed = count == 0};
        for (std::size_t index = 0; index < count; ++index) {
          sequence_step(expected.reg, index);
          expected.cycles += inputs.at(index).cycles;
        }
        a64::Invocation invocation(initial);
        invocation.run(program, budget);
        invocation.expect(expected, inputs.at(count == 0 ? 0 : count - 1).opcode);
      }
    }
  }
}

void count_failure_prefix(const Word mode) {
  constexpr Word inc = 8;
  constexpr int invalid_count = 57;
  const auto opcode = normalization_model::opcode({.selector = 2});
  const std::array inputs{a64::Instruction{.pc = start, .mode = mode},
                          a64::Instruction{.opcode = opcode, .pc = start + 1, .mode = mode},
                          a64::Instruction{.opcode = inc, .pc = start + 2, .mode = mode}};
  const a64::Program program(inputs);
  auto initial = seed(mode | status_byte);
  count_word(initial, {.selector = 2, .value = invalid_count});
  for (const Word budget : {0U, 1U, 2U, 3U, 32U}) {
    const bool reached = budget >= 2;
    const Word count = budget == 0 ? 0U : 1U;
    a64::Expected expected{.reg = initial,
                           .retired = count,
                           .cycles = count,
                           .inhibited = count == 0,
                           .repeat_completed = count == 0};
    expected.reg.pc += reached ? 2 : count;
    if (reached) {
      expected.failure = {.kind = FaultKind::unsupported_mode,
                          .space = Space::program,
                          .address = start + 1,
                          .value = opcode,
                          .pc = start + 1};
      expected.boundary = detail::NativeBoundary::owner;
    }
    a64::Invocation invocation(initial);
    invocation.run(program, budget);
    invocation.expect(expected, opcode);
  }
}

void faults() {
  constexpr std::array modes{compatibility, Word{1} << 14, Word{1} << 17, mode_up | mode_down};
  for (const Word mode : modes) {
    for (const auto opcode : {division, normalization, fast}) {
      if (opcode == normalization && mode == compatibility) { continue; }
      const a64::Program program(
          {.opcode = opcode, .mode = mode, .cycles = opcode == normalization ? 5U : 1U});
      const auto initial = seed(mode | status_byte);
      a64::Expected expected{.reg = initial,
                             .retired = 0,
                             .cycles = 0,
                             .failure = {.kind = FaultKind::unsupported_mode,
                                         .space = Space::program,
                                         .address = start,
                                         .value = initial.sr,
                                         .pc = start},
                             .boundary = detail::NativeBoundary::owner,
                             .repeat_completed = true};
      ++expected.reg.pc;
      check(program, initial, expected, opcode);
    }
  }
  for (const Word selector : {0U, 1U}) {
    for (const Word mode : {0U, saturation_model::mode}) {
      const auto opcode = fast | (selector * 2);
      const a64::Program program({.opcode = opcode, .mode = mode});
      const auto initial = seed(mode | status_byte);
      a64::Expected expected{.reg = initial,
                             .retired = 0,
                             .cycles = 0,
                             .failure = {.kind = FaultKind::invalid_instruction,
                                         .space = Space::program,
                                         .address = start,
                                         .value = opcode,
                                         .pc = start},
                             .boundary = detail::NativeBoundary::owner,
                             .repeat_completed = true};
      ++expected.reg.pc;
      check(program, initial, expected, opcode);
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        const auto mode = scale | saturation;
        divide(mode);
        normalize(mode);
        normalize(mode | compatibility);
        normalize_fast(mode);
        sequences(mode);
        count_failure_prefix(mode);
      }
    }
    faults();
    std::cout << checks << " independent native A64 iteration checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
