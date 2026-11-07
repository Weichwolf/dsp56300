// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "bit_field_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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
using Kind = field_model::Kind;
using Input = field_model::Input;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::uint64_t pattern = 0xa572e19c46b38d;
constexpr std::array raw{std::uint64_t{0}, pattern, accumulator_mask ^ pattern, accumulator_mask};

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status;
  return state;
}

a64::Instruction instruction(const Input input, const Word mode) {
  return {.opcode = field_model::opcode(input),
          .extension = input.immediate ? field_model::control(input) : 0U,
          .mode = mode,
          .words = input.immediate ? 2U : 1U,
          .cycles = input.immediate ? 2U : 1U};
}

void probe(const a64::Program &program, const Registers &initial, const Input input) {
  const auto authored = instruction(input, initial.sr);
  a64::Expected expected{.reg = initial, .cycles = authored.cycles};
  expected.failure = field_model::apply(expected.reg, input);
  expected.reg.pc += authored.words;
  if (expected.failure.kind != FaultKind::none) {
    expected.retired = 0;
    expected.cycles = 0;
    expected.repeat_completed = true;
    expected.boundary = detail::NativeBoundary::owner;
  }
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, authored.opcode);
}

void operands(const Input input, const Word mode) {
  const a64::Program program(instruction(input, mode));
  for (const auto value : raw) {
    for (const Word flags : {0U, status_byte}) {
      auto initial = seed(mode | flags);
      initial.a = value;
      initial.b = accumulator_mask ^ value;
      if (input.kind == Kind::insert && input.source >= 2) {
        field_model::put(initial,
                         {.selector = input.source, .main = false},
                         static_cast<Word>(value) & word_mask);
      }
      if (!input.immediate && input.kind != Kind::leading && input.controller >= 2) {
        field_model::put(initial, {.selector = input.controller}, field_model::control(input));
      }
      probe(program, initial, input);
    }
  }
}

void windows() {
  for (const auto kind : {Kind::extract, Kind::extract_unsigned, Kind::insert}) {
    for (Word width = 0; width < 64; ++width) {
      for (Word offset = 0; offset < 64; ++offset) {
        const auto mode = scales.at((width + offset) % scales.size()) |
                          (width % 2 == 0 ? saturation_model::mode : 0U);
        const Input input{.kind = kind,
                          .source = 2 + ((width + offset) % 6),
                          .width = width,
                          .offset = offset,
                          .immediate = true,
                          .source_b = width % 2 != 0,
                          .destination_b = offset % 2 != 0};
        operands(input, mode);
      }
    }
  }
}

void aliases(const Word mode) {
  constexpr std::array<field_model::Window, 8> windows{{{.width = 0, .offset = 56},
                                                        {.width = 1, .offset = 55},
                                                        {.width = 24, .offset = 24},
                                                        {.width = 24, .offset = 32},
                                                        {.width = 56, .offset = 0},
                                                        {.width = 12, .offset = 12},
                                                        {.width = 63, .offset = 63},
                                                        {.width = 0, .offset = 0}}};
  for (const auto kind : {Kind::extract, Kind::extract_unsigned, Kind::insert, Kind::merge}) {
    for (unsigned controller = 2; controller < 8; ++controller) {
      for (unsigned source = 2; source < 8; ++source) {
        for (const bool destination_b : {false, true}) {
          for (const auto window : windows) {
            operands({.kind = kind,
                      .controller = controller,
                      .source = source,
                      .width = window.width,
                      .offset = window.offset,
                      .source_b = (source & 1U) != 0,
                      .destination_b = destination_b},
                     mode);
          }
        }
      }
    }
  }
}

void dynamic_fields(const Word mode) {
  for (const auto kind : {Kind::extract, Kind::extract_unsigned, Kind::insert}) {
    const Input base{.kind = kind};
    const a64::Program program(instruction(base, mode));
    for (Word width = 0; width < 64; ++width) {
      for (Word offset = 0; offset < 64; ++offset) {
        auto point = base;
        point.width = width;
        point.offset = offset;
        auto initial = seed(mode | status_byte);
        initial.a = pattern;
        field_model::put(initial, {.selector = 4}, field_model::control(point));
        probe(program, initial, point);
      }
    }
  }
}

void merge_bits(const Word mode) {
  for (unsigned controller = 2; controller < 8; ++controller) {
    for (const bool destination_b : {false, true}) {
      const Input input{
          .kind = Kind::merge, .controller = controller, .destination_b = destination_b};
      const a64::Program program(instruction(input, mode));
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        for (const auto value : raw) {
          auto initial = seed(mode | status_byte);
          initial.a = value;
          initial.b = accumulator_mask ^ value;
          const auto single = Word{1} << bit;
          field_model::put(initial, {.selector = controller}, single);
          probe(program, initial, input);
          field_model::put(initial, {.selector = controller}, word_mask ^ single);
          probe(program, initial, input);
          field_model::put(initial, {.selector = controller}, 0);
          probe(program, initial, input);
        }
      }
    }
  }
}

void leading(const Word mode) {
  for (const bool source_b : {false, true}) {
    for (const bool destination_b : {false, true}) {
      const Input input{
          .kind = Kind::leading, .source_b = source_b, .destination_b = destination_b};
      const a64::Program program(instruction(input, mode));
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
          auto initial = seed(mode | flags);
          const auto value = std::uint64_t{1} << bit;
          initial.a = value;
          initial.b = accumulator_mask ^ value;
          probe(program, initial, input);
        }
        for (const auto value : {std::uint64_t{0}, accumulator_mask}) {
          auto initial = seed(mode | flags);
          (source_b ? initial.b : initial.a) = value;
          probe(program, initial, input);
        }
      }
    }
  }
}

void faults() {
  for (const auto kind :
       {Kind::extract, Kind::extract_unsigned, Kind::insert, Kind::merge, Kind::leading}) {
    for (const Word mode : {Word{1} << 13, Word{1} << 14, Word{1} << 17, mode_up | mode_down}) {
      operands({.kind = kind}, mode);
    }
    if (kind == Kind::leading) { continue; }
    for (unsigned selector = 0; selector < 2; ++selector) {
      operands({.kind = kind, .controller = selector}, 0);
      if (kind == Kind::insert) {
        operands({.kind = kind, .source = selector, .immediate = true}, 0);
      }
    }
  }
}

void sequences(const Word mode) {
  const std::array points{
      Input{.kind = Kind::merge},
      Input{.kind = Kind::extract_unsigned,
            .width = 24,
            .offset = 12,
            .immediate = true,
            .destination_b = true},
      Input{.kind = Kind::insert,
            .source = 2,
            .width = 24,
            .offset = 32,
            .immediate = true,
            .destination_b = true},
      Input{.kind = Kind::leading, .source_b = true},
      Input{.kind = Kind::extract_unsigned, .controller = 2, .destination_b = true},
      Input{.kind = Kind::merge, .controller = 3}};
  std::vector<a64::Instruction> inputs;
  Word cursor = start;
  for (const auto point : points) {
    auto input = instruction(point, mode);
    input.pc = cursor;
    cursor += input.words;
    inputs.push_back(input);
  }
  const a64::Program program(inputs);
  for (const auto value : raw) {
    auto initial = seed(mode | status_byte);
    initial.a = value;
    initial.b = accumulator_mask ^ value;
    for (const Word budget : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 32U}) {
      a64::Expected expected{.reg = initial,
                             .retired = 0,
                             .cycles = 0,
                             .inhibited = budget == 0,
                             .repeat_completed = true};
      const auto count = std::min<std::size_t>(budget, points.size());
      for (std::size_t index = 0; index < count; ++index) {
        expected.failure = field_model::apply(expected.reg, points.at(index));
        const auto pc = expected.reg.pc;
        expected.reg.pc += inputs.at(index).words;
        if (expected.failure.kind != FaultKind::none) {
          expected.failure.address = expected.failure.pc = pc;
          expected.boundary = detail::NativeBoundary::owner;
          break;
        }
        ++expected.retired;
        expected.cycles += inputs.at(index).cycles;
        expected.repeat_completed = false;
      }
      a64::Invocation invocation(initial);
      invocation.run(program, budget);
      invocation.expect(expected, inputs.front().opcode);
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    windows();
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        const auto mode = scale | saturation;
        aliases(mode);
        dynamic_fields(mode);
        merge_bits(mode);
        leading(mode);
        sequences(mode);
      }
    }
    faults();
    std::cout << checks << " independent native A64 bit-field checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
