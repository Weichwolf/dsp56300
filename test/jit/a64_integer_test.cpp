// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word increment = 8, decrement = 10, compare = 0x0c1ff0;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array<std::uint64_t, 10> boundaries{0,
                                                   1,
                                                   sign56 - 1,
                                                   sign56,
                                                   accumulator_mask,
                                                   saturation_model::positive,
                                                   saturation_model::positive + 1,
                                                   saturation_model::negative - 1,
                                                   saturation_model::negative,
                                                   0x123456abcdef12};

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status;
  return state;
}

void units(const Word mode) {
  std::vector<std::uint64_t> values(boundaries.begin(), boundaries.end());
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    values.push_back(std::uint64_t{1} << bit);
    values.push_back(accumulator_mask ^ (std::uint64_t{1} << bit));
  }
  for (const bool subtract : {false, true}) {
    for (const bool destination_b : {false, true}) {
      const auto opcode = (subtract ? decrement : increment) | (destination_b ? 1U : 0U);
      const a64::Program program({.opcode = opcode, .mode = mode});
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto value : values) {
          auto initial = seed(mode | flags);
          (destination_b ? initial.b : initial.a) = value;
          a64::Expected expected{.reg = initial};
          saturation_model::unit(expected.reg, subtract, destination_b);
          ++expected.reg.pc;
          a64::Invocation invocation(initial);
          invocation.run(program);
          invocation.expect(expected, opcode);
        }
      }
    }
  }
}

struct Comparison {
  unsigned selector;
  bool destination_b;
};

struct Operands {
  std::uint64_t left, right;
};

void compare_case(const a64::Program &program,
                  Registers initial,
                  const Comparison form,
                  const Operands values) {
  constexpr std::uint64_t exp_left = 0x80000000000000;
  constexpr std::uint64_t exp_right = 0x7f000000000000;
  (form.destination_b ? initial.b : initial.a) = exp_left | values.left;
  const auto right =
      form.selector == 0 ? values.right : (values.right & word_mask) * (word_mask + 1ULL);
  if (form.selector == 0) {
    (form.destination_b ? initial.a : initial.b) = exp_right | right;
  } else {
    auto &bank = (form.selector & 1U) == 0 ? initial.x : initial.y;
    const auto shift = form.selector < 6 ? 0U : word_bits;
    bank = (bank & ~(std::uint64_t{word_mask} << shift)) | ((values.right & word_mask) << shift);
  }
  a64::Expected expected{.reg = initial};
  const auto difference = static_cast<std::int64_t>(values.left) - static_cast<std::int64_t>(right);
  Word result = difference == 0 ? flag_z : 0;
  if (difference < 0) { result |= flag_c | flag_n; }
  expected.reg.sr = (expected.reg.sr & ~(flag_c | flag_v | flag_n | flag_z)) | result;
  ++expected.reg.pc;
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, compare | (form.selector * 2) | (form.destination_b ? 1U : 0U));
}

void comparisons(const Word mode) {
  constexpr std::uint64_t long_sign = std::uint64_t{1} << 47;
  constexpr std::array values{
      std::uint64_t{0}, std::uint64_t{1}, long_sign - 1, long_sign, long_mask};
  for (const bool destination_b : {false, true}) {
    for (const unsigned selector : {0U, 4U, 5U, 6U, 7U}) {
      const auto opcode = compare | (selector * 2) | (destination_b ? 1U : 0U);
      const a64::Program program({.opcode = opcode, .mode = mode});
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto left : values) {
          for (const auto input : values) {
            compare_case(program,
                         seed(mode | flags),
                         {.selector = selector, .destination_b = destination_b},
                         {.left = left, .right = input});
          }
        }
      }
    }
  }
}

struct Rejected {
  Word opcode, mode = 0;
  bool invalid = false;
};

void rejection(const Rejected input) {
  const std::array operations{
      a64::Instruction{.pc = start, .mode = input.mode},
      a64::Instruction{.opcode = input.opcode, .pc = start + 1, .mode = input.mode},
      a64::Instruction{.opcode = increment, .pc = start + 2, .mode = input.mode}};
  const a64::Program program(operations);
  for (const Word budget : {0U, 1U, 2U, 3U, 32U}) {
    const auto initial = seed(input.mode | status_byte);
    const bool reached = budget >= 2;
    a64::Expected expected{.reg = initial,
                           .retired = budget == 0 ? 0U : 1U,
                           .cycles = budget == 0 ? 0U : 1U,
                           .inhibited = budget == 0,
                           .repeat_completed = budget == 0};
    expected.reg.pc += reached ? 2 : expected.retired;
    if (reached) {
      expected.failure = {.kind = input.invalid ? FaultKind::invalid_instruction
                                                : FaultKind::unsupported_mode,
                          .space = Space::program,
                          .address = start + 1,
                          .value = input.invalid ? input.opcode : initial.sr,
                          .pc = start + 1};
      expected.boundary = detail::NativeBoundary::owner;
    }
    a64::Invocation invocation(initial);
    invocation.run(program, budget);
    invocation.expect(expected, input.opcode);
  }
}

void faults() {
  for (const Word mode : {Word{1} << 13, Word{1} << 14, Word{1} << 17, mode_up | mode_down}) {
    for (const auto opcode : {increment, decrement, compare}) {
      rejection({.opcode = opcode, .mode = mode});
    }
  }
  for (unsigned selector = 1; selector <= 3; ++selector) {
    for (const bool destination_b : {false, true}) {
      const auto opcode = compare | (selector * 2) | (destination_b ? 1U : 0U);
      rejection({.opcode = opcode, .invalid = true});
      rejection({.opcode = opcode, .mode = saturation_model::mode, .invalid = true});
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
        units(scale | saturation);
        comparisons(scale | saturation);
      }
    }
    faults();
    std::cout << checks << " independent native A64 integer checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
