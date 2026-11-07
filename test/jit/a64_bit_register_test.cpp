// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "bit_access_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
using Input = bit_model::Input;
using Flow = bit_model::Flow;
constexpr std::array actions{BitAction::test, BitAction::clear, BitAction::set, BitAction::change};
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};

Registers seed(const Word mode) {
  auto state = register_model::seed(mode);
  state.sr = mode | status_byte;
  return state;
}

void probe(const a64::Program &program, const Registers &initial, const Input input) {
  a64::Expected expected{.reg = initial, .cycles = bit_model::cycles(input)};
  expected.failure = bit_model::apply_register(expected.reg, input);
  if (expected.failure.kind != FaultKind::none) {
    expected.retired = 0;
    expected.cycles = 0;
    expected.repeat_completed = true;
    expected.boundary = detail::NativeBoundary::owner;
  } else if (bit_model::owner(input)) {
    expected.boundary = detail::NativeBoundary::owner;
  }
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, bit_model::opcode(input));
}

void point(const Input input, const Word mode) {
  const a64::Program program({.opcode = bit_model::opcode(input),
                              .extension = input.extension,
                              .mode = mode,
                              .words = bit_model::words(input),
                              .cycles = bit_model::cycles(input)});
  auto initial = seed(mode);
  probe(program, initial, input);
  initial.sr = mode;
  initial.a = initial.b = initial.x = initial.y = 0;
  initial.r.fill(0);
  initial.n.fill(0);
  initial.stack.fill(0);
  probe(program, initial, input);
}

void registers(const Word mode) {
  for (unsigned number = 0; number < 64; ++number) {
    for (unsigned bit = 0; bit < 32; ++bit) {
      for (const auto action : actions) {
        point({.action = action, .selector = number, .bit = bit}, mode);
      }
    }
  }
}

void flow(const Word mode) {
  constexpr Word backwards = 0xffff00;
  constexpr Word forwards = 0xfedcba;
  for (const auto kind : {Flow::jump, Flow::relative}) {
    for (const bool call : {false, true}) {
      for (const bool wanted : {false, true}) {
        for (unsigned number = 0; number < 64; ++number) {
          for (const auto bit : {0U, 7U, 15U, 23U, 24U, 31U}) {
            point({.flow = kind,
                   .selector = number,
                   .bit = bit,
                   .extension = kind == Flow::relative ? backwards : forwards,
                   .call = call,
                   .wanted = wanted},
                  mode);
          }
        }
      }
    }
  }
}

void stack_faults() {
  for (const unsigned number : {4U, 14U, 15U, static_cast<unsigned>(register_model::reg_ssh)}) {
    for (const Word count :
         {0U, 1U, static_cast<Word>(stack_entries - 1), static_cast<Word>(stack_entries)}) {
      for (const Word pointer : {0U, 15U, 31U, 63U}) {
        const Input input{
            .flow = Flow::relative, .selector = number, .bit = 0, .call = true, .wanted = true};
        const a64::Program program({.opcode = bit_model::opcode(input),
                                    .extension = input.extension,
                                    .words = 2,
                                    .cycles = 5});
        auto initial = seed(0);
        initial.sc = count;
        initial.sp = pointer;
        initial.x = word_mask;
        initial.a = initial.b = accumulator_mask;
        initial.stack.fill(long_mask);
        probe(program, initial, input);
      }
    }
  }
  for (const auto action : actions) {
    const Input input{.action = action, .selector = register_model::reg_ssh};
    const a64::Program program({.opcode = bit_model::opcode(input), .cycles = 2});
    auto initial = seed(0);
    initial.sc = 0;
    initial.sp = register_model::pointer_mask;
    probe(program, initial, input);
  }
}

void sequences(const Word mode) {
  const std::array inputs{
      Input{.action = BitAction::change},
      Input{.action = BitAction::test},
      Input{.action = BitAction::set, .selector = register_model::reg_ssh, .bit = 7},
      Input{.action = BitAction::test, .selector = register_model::reg_ssh, .bit = 7},
      Input{.flow = Flow::jump, .extension = start + 6, .call = true},
      Input{.action = BitAction::clear, .selector = register_model::reg_ssl}};
  std::vector<a64::Instruction> operations;
  Word pc = start;
  for (const auto input : inputs) {
    operations.push_back({.opcode = bit_model::opcode(input),
                          .extension = input.extension,
                          .pc = pc,
                          .mode = mode,
                          .words = bit_model::words(input),
                          .cycles = bit_model::cycles(input)});
    pc += bit_model::words(input);
  }
  const a64::Program program(operations);
  auto initial = seed(mode);
  initial.x |= 1U;
  for (const Word budget : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 32U}) {
    const auto count = std::min<std::size_t>(budget, inputs.size());
    a64::Expected expected{.reg = initial,
                           .retired = static_cast<Word>(count),
                           .cycles = 0,
                           .inhibited = count == 0,
                           .repeat_completed = count == 0};
    for (std::size_t index = 0; index < count; ++index) {
      require(bit_model::apply_register(expected.reg, inputs.at(index)).kind == FaultKind::none,
              "Authored bit sequence must execute without a fault");
      expected.cycles += bit_model::cycles(inputs.at(index));
    }
    a64::Invocation invocation(initial);
    invocation.run(program, budget);
    invocation.expect(expected, operations.front().opcode);
  }
}

void failure_prefix(const Input input) {
  const Input prefix{.action = BitAction::change};
  const Input successor{.action = BitAction::clear, .selector = register_model::reg_ssl};
  const std::array points{prefix, input, successor};
  std::vector<a64::Instruction> operations;
  Word pc = start;
  for (const auto point : points) {
    operations.push_back({.opcode = bit_model::opcode(point),
                          .extension = point.extension,
                          .pc = pc,
                          .words = bit_model::words(point),
                          .cycles = bit_model::cycles(point)});
    pc += bit_model::words(point);
  }
  const a64::Program program(operations);
  const auto initial = seed(0);
  for (const Word budget : {0U, 1U, 2U, 3U, 32U}) {
    a64::Expected expected{.reg = initial,
                           .retired = 0,
                           .cycles = 0,
                           .inhibited = budget == 0,
                           .repeat_completed = true};
    if (budget != 0) {
      expected.failure = bit_model::apply_register(expected.reg, prefix);
      expected.retired = 1;
      expected.cycles = bit_model::cycles(prefix);
      expected.repeat_completed = false;
    }
    if (budget >= 2) {
      expected.failure = bit_model::apply_register(expected.reg, input);
      require(expected.failure.kind != FaultKind::none,
              "Authored invalid bit instruction must fail");
      expected.failure.address = expected.failure.pc = start + 1;
      expected.boundary = detail::NativeBoundary::owner;
    }
    a64::Invocation invocation(initial);
    invocation.run(program, budget);
    invocation.expect(expected, bit_model::opcode(input));
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    using dsp56300::Word;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        registers(scale | saturation);
        flow(scale | saturation);
        sequences(scale | saturation);
      }
    }
    for (const Word mode : {Word{1} << 13, Word{1} << 14, Word{1} << 17, mode_up | mode_down}) {
      registers(mode);
      flow(mode);
    }
    stack_faults();
    failure_prefix({.selector = 0});
    failure_prefix({.bit = word_bits});
    failure_prefix({.flow = Flow::jump, .bit = word_bits});
    std::cout << checks << " independent native A64 register-bit checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
