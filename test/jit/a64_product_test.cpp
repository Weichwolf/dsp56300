// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "saturation_products.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
enum class Family : std::uint8_t { ordinary, mixed };
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array words{0U, 1U, 2U, sign24 - 1, sign24, sign24 + 1, word_mask - 1, word_mask};
constexpr std::array<std::uint64_t, 12> accumulators{0,
                                                     1,
                                                     word_mask,
                                                     std::uint64_t{1} << word_bits,
                                                     sign56 - 1,
                                                     sign56,
                                                     accumulator_mask - 1,
                                                     accumulator_mask,
                                                     saturation_model::positive,
                                                     saturation_model::positive + 1,
                                                     saturation_model::negative - 1,
                                                     saturation_model::negative};
constexpr std::array flags{0U, flag_c | flag_s | flag_l, status_byte};

struct Shape {
  Family family = Family::ordinary;
  saturation_products::Operation operation;
  Word mode = 0;
};

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status;
  return state;
}

BinaryCase model(const Registers &initial, const Shape point) {
  std::vector<BinaryCase> cases;
  if (point.family == Family::ordinary) {
    saturation_products::ordinary(cases, initial, point.operation);
  } else {
    saturation_products::mixed(cases, initial, point.operation);
  }
  require(cases.size() == 1, "Independent product model must author one complete instruction");
  return cases.front();
}

a64::Instruction instruction(const Shape point) {
  const auto item = model(seed(point.mode), point);
  const auto &program = item.before.front().words;
  return {.opcode = program.front(),
          .extension = program.size() == 2 ? program.back() : 0U,
          .mode = point.mode,
          .words = static_cast<unsigned>(program.size()),
          .cycles = static_cast<unsigned>(program.size())};
}

void check(const a64::Program &program, const Registers &initial, const Shape point) {
  const auto item = model(initial, point);
  const a64::Expected expected{.reg = item.expected, .cycles = item.progress.estimated_cycles};
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, item.before.front().words.front());
}

void ordinary_operands(const Shape point) {
  const a64::Program program(instruction(point));
  for (const auto status : flags) {
    for (const auto word : words) {
      for (const auto accumulator : accumulators) {
        auto initial = seed(point.mode | status);
        (point.operation.destination_b ? initial.b : initial.a) = accumulator;
        saturation_products::put(initial, {.selector = point.operation.left, .value = word});
        check(program, initial, point);
      }
    }
  }
}

void ordinary_coefficients(Shape point) {
  for (const auto coefficient : words) {
    point.operation.coefficient = coefficient;
    ordinary_operands(point);
  }
  constexpr unsigned maximum_power = 23;
  for (unsigned power = 1; power <= maximum_power; ++power) {
    point.operation.power = power;
    ordinary_operands(point);
  }
}

void ordinary(const Word mode) {
  for (unsigned source = 0; source < 4; ++source) {
    for (unsigned variant = 0; variant < 8; ++variant) {
      for (const bool destination_b : {false, true}) {
        ordinary_coefficients(
            {.operation = {.left = source, .variant = variant, .destination_b = destination_b},
             .mode = mode});
      }
    }
  }
}

void mixed_operands(const Shape point) {
  const a64::Program program(instruction(point));
  for (const auto status : flags) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      for (const auto accumulator : accumulators) {
        auto initial = seed(point.mode | status);
        (point.operation.destination_b ? initial.b : initial.a) = accumulator;
        saturation_products::put(initial,
                                 {.selector = point.operation.left, .value = words.at(index)});
        saturation_products::put(
            initial,
            {.selector = point.operation.right, .value = words.at((index + 3) % words.size())});
        check(program, initial, point);
      }
    }
  }
}

void mixed_signs(Shape point) {
  point.operation.signs = 0;
  point.operation.double_product = true;
  mixed_operands(point);
  for (unsigned signs = 1; signs <= 2; ++signs) {
    point.operation.signs = signs;
    for (const bool double_product : {false, true}) {
      point.operation.double_product = double_product;
      for (const bool accumulate : {false, true}) {
        point.operation.accumulate = accumulate;
        mixed_operands(point);
      }
    }
  }
}

void mixed(const Word mode) {
  for (unsigned left = 0; left < 4; ++left) {
    for (unsigned right = 0; right < 4; ++right) {
      for (const bool destination_b : {false, true}) {
        for (const Word variant : {0U, 4U}) {
          mixed_signs({.family = Family::mixed,
                       .operation = {.left = left,
                                     .right = right,
                                     .variant = variant,
                                     .destination_b = destination_b},
                       .mode = mode});
        }
      }
    }
  }
}

void rounding(const Word mode) {
  const unsigned bits =
      word_bits + ((mode & mode_down) != 0 ? 1U : 0U) - ((mode & mode_up) != 0 ? 1U : 0U);
  const auto half = std::uint64_t{1} << (bits - 1);
  const auto negative_half = accumulator_mask - half + 1;
  const std::array boundaries{half - 2,
                              half - 1,
                              half,
                              half + 1,
                              half + 2,
                              3 * half,
                              negative_half - 2,
                              negative_half,
                              negative_half + 2,
                              sign56 - 1,
                              sign56,
                              accumulator_mask};
  for (unsigned variant = 1; variant < 8; variant += 2) {
    for (const bool destination_b : {false, true}) {
      const Shape point{
          .operation = {.coefficient = 1, .variant = variant, .destination_b = destination_b},
          .mode = mode};
      const a64::Program program(instruction(point));
      for (const auto accumulator : boundaries) {
        auto initial = seed(mode | status_byte);
        (destination_b ? initial.b : initial.a) = accumulator;
        saturation_products::put(initial, {.selector = 0, .value = 1});
        check(program, initial, point);
      }
    }
  }
}

void sequences(const Word mode) {
  const std::array shapes{
      Shape{.operation = {.left = 1, .coefficient = sign24, .variant = 3}, .mode = mode},
      Shape{.family = Family::mixed,
            .operation = {.left = 2, .right = 3, .signs = 0, .variant = 4, .double_product = true},
            .mode = mode},
      Shape{.family = Family::mixed,
            .operation =
                {.left = 0, .right = 2, .signs = 2, .destination_b = true, .accumulate = false},
            .mode = mode},
      Shape{.operation = {.left = 3, .power = 1, .variant = 7, .destination_b = true},
            .mode = mode},
      Shape{.operation = {.left = 0, .coefficient = 1, .variant = 4}, .mode = mode}};
  std::vector<a64::Instruction> inputs;
  Word cursor = start;
  for (const auto point : shapes) {
    auto input = instruction(point);
    input.pc = cursor;
    cursor += input.words;
    inputs.push_back(input);
  }
  const a64::Program program(inputs);
  for (const auto accumulator : accumulators) {
    auto initial = seed(mode | status_byte);
    initial.a = accumulator;
    initial.b = accumulator_mask ^ accumulator;
    for (const Word budget : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 32U}) {
      const auto count = std::min<std::size_t>(budget, shapes.size());
      a64::Expected expected{.reg = initial,
                             .retired = static_cast<Word>(count),
                             .cycles = 0,
                             .inhibited = count == 0,
                             .repeat_completed = count == 0};
      for (std::size_t index = 0; index < count; ++index) {
        const auto item = model(expected.reg, shapes.at(index));
        expected.reg = item.expected;
        expected.cycles += item.progress.estimated_cycles;
      }
      a64::Invocation invocation(initial);
      invocation.run(program, budget);
      invocation.expect(expected, inputs.front().opcode);
    }
  }
}

struct Rejected {
  Word opcode, mode = 0;
  unsigned words = 1, cycles = 1;
  FaultKind kind = FaultKind::invalid_instruction;
  bool mode_failure = false;
};

void rejected(const Rejected input) {
  constexpr Word increment = 8;
  const std::array inputs{
      a64::Instruction{.pc = start, .mode = input.mode},
      a64::Instruction{.opcode = input.opcode,
                       .pc = start + 1,
                       .mode = input.mode,
                       .words = input.words,
                       .cycles = input.cycles},
      a64::Instruction{.opcode = increment, .pc = start + 1 + input.words, .mode = input.mode}};
  const a64::Program program(inputs);
  const auto initial = seed(input.mode | status_byte);
  for (const Word budget : {0U, 1U, 2U, 3U, 32U}) {
    const bool reached = budget >= 2;
    const Word count = budget == 0 ? 0U : 1U;
    a64::Expected expected{.reg = initial,
                           .retired = count,
                           .cycles = count,
                           .inhibited = count == 0,
                           .repeat_completed = count == 0};
    expected.reg.pc += reached ? 1 + input.words : count;
    if (reached) {
      expected.failure = {.kind = input.kind,
                          .space = Space::program,
                          .address = start + 1,
                          .value = input.mode_failure ? initial.sr : input.opcode,
                          .pc = start + 1};
      expected.boundary = detail::NativeBoundary::owner;
    }
    a64::Invocation invocation(initial);
    invocation.run(program, budget);
    invocation.expect(expected, input.opcode);
  }
}

void faults() {
  for (const Word mode : {0U, saturation_model::mode}) {
    for (const Word power : {0U, 24U, 25U, 31U}) {
      for (Word fields = 0; fields < 64; ++fields) {
        const auto opcode = saturation_products::power_base | (power << 8) | (fields & 63U);
        rejected(
            {.opcode = opcode,
             .mode = mode,
             .kind = power == 0 ? FaultKind::unsupported_mode : FaultKind::invalid_instruction});
      }
    }
    constexpr Word reserved_signs = 0x40;
    for (Word fields = 0; fields < 64; ++fields) {
      rejected(
          {.opcode = saturation_products::double_base | reserved_signs | fields, .mode = mode});
    }
  }
  for (const Word mode : {Word{1} << 13, Word{1} << 14, Word{1} << 17, mode_up | mode_down}) {
    for (const auto base : {saturation_products::immediate_base,
                            saturation_products::power_base,
                            saturation_products::mixed_mac,
                            saturation_products::double_base}) {
      const bool immediate = base == saturation_products::immediate_base;
      rejected({.opcode = base,
                .mode = mode,
                .words = immediate ? 2U : 1U,
                .cycles = immediate ? 2U : 1U,
                .kind = FaultKind::unsupported_mode,
                .mode_failure = true});
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
        ordinary(mode);
        mixed(mode);
        rounding(mode);
        sequences(mode);
      }
    }
    faults();
    std::cout << checks << " independent native A64 extended product checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
