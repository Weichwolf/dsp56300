// SPDX-License-Identifier: GPL-3.0-only
#include "a64_native.hpp"
#include "address_model.hpp"
#include "architecture.hpp"
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "return_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace dsp56300::test {
namespace {
struct Selection {
  Word status;
  unsigned code;
};

bool selected(const Selection input) {
  const auto status = input.status;
  const bool c = status % 2 != 0;
  const bool v = (status / 2) % 2 != 0;
  const bool z = (status / 4) % 2 != 0;
  const bool n = (status / 8) % 2 != 0;
  const bool u = (status / 16) % 2 != 0;
  const bool e = (status / 32) % 2 != 0;
  const bool l = (status / 64) % 2 != 0;
  const std::array table{!c,
                         n == v,
                         !z,
                         !n,
                         !z && (u || e),
                         !e,
                         !l,
                         !z && (n == v),
                         c,
                         n != v,
                         z,
                         n,
                         z || (!u && !e),
                         e,
                         l,
                         z || (n != v)};
  return table.at(input.code);
}

struct Jump {
  a64::Instruction instruction;
  Word target = 0;
  bool call = false;
  std::optional<unsigned> condition = std::nullopt;
};

void jump_model(a64::Expected &expected, const Jump point) {
  auto &state = expected.reg;
  state.pc = (point.instruction.pc + point.instruction.words) % address_words;
  expected.cycles = point.instruction.cycles;
  if (point.condition && !selected({.status = state.sr, .code = *point.condition})) { return; }
  if (point.call) {
    if (state.sc >= stack_entries - 1) {
      expected.retired = 0;
      expected.cycles = 0;
      expected.repeat_completed = true;
      expected.boundary = detail::NativeBoundary::owner;
      expected.failure = {.kind = FaultKind::stack,
                          .space = Space::program,
                          .address = point.instruction.pc,
                          .value = state.sp,
                          .pc = point.instruction.pc};
    } else {
      const auto frame = (std::uint64_t{state.pc} << word_bits) | state.sr;
      state.sp = (state.sp + 1) % return_model::pointer_range;
      ++state.sc;
      state.stack.at(state.sp % stack_entries) = frame;
    }
  }
  state.pc = point.target;
}

void jump_case(const a64::Program &program, const Jump point, const Registers &initial) {
  a64::Expected expected{.reg = initial};
  jump_model(expected, point);
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect(expected, point.instruction.opcode);
}

Registers seed(const Word flags) {
  auto state = register_model::seed(0);
  state.pc = start;
  state.sr = flags;
  state.sp = 0;
  state.sc = 0;
  return state;
}

void short_jumps() {
  constexpr Word target = 0x345;
  for (const bool call : {false, true}) {
    for (unsigned selector = 0; selector < 16; ++selector) {
      constexpr Word jump_base = 0x0e0000;
      constexpr Word call_base = 0x0f0000;
      const Jump point{
          .instruction = {.opcode = (call ? call_base : jump_base) | (selector << 12) | target,
                          .cycles = 4},
          .target = target,
          .call = call,
          .condition = selector};
      const a64::Program program(point.instruction);
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const Word pointer : {0U, 15U, 31U, 63U}) {
          for (const Word count : {0U, 14U, 15U, 31U}) {
            auto initial = seed(flags);
            initial.sp = pointer;
            initial.sc = count;
            jump_case(program, point, initial);
          }
        }
      }
    }
    constexpr Word jump_base = 0x0c0000;
    constexpr Word call_base = 0x0d0000;
    const Jump point{
        .instruction = {.opcode = (call ? call_base : jump_base) | target, .cycles = 3},
        .target = target,
        .call = call};
    const a64::Program program(point.instruction);
    for (Word pointer = 0; pointer < return_model::pointer_range; ++pointer) {
      for (Word count = 0; count < stack_entries * 2; ++count) {
        auto initial = seed(status_byte);
        initial.sp = pointer;
        initial.sc = count;
        jump_case(program, point, initial);
      }
    }
    a64::Invocation empty(seed(status_byte));
    empty.run(program, 0);
    empty.expect({.reg = seed(status_byte),
                  .retired = 0,
                  .cycles = 0,
                  .inhibited = true,
                  .repeat_completed = true},
                 point.instruction.opcode);
  }
}

struct Relative {
  enum class Form : std::uint8_t { short_, long_, reg };
  Form form;
  Word displacement = 0;
  unsigned selector = 0, index = 0;
  bool call = false, conditional = true;
};

Jump relative(const Relative input) {
  Jump result{.instruction = {.pc = word_mask - 2, .cycles = 4},
              .call = input.call,
              .condition = input.conditional ? std::optional{input.selector} : std::nullopt};
  if (input.form == Relative::Form::short_) {
    constexpr Word group = 0x050000;
    constexpr Word upper = 0x1e0;
    constexpr Word lower = 0x1f;
    result.instruction.opcode =
        group | ((input.displacement & upper) * 2) | (input.displacement & lower) |
        (input.conditional ? input.selector * 4096U : 2048U) | (input.call ? 0U : 1024U);
    const auto signed_offset = input.displacement < 256U ? std::int64_t{input.displacement}
                                                         : std::int64_t{input.displacement} - 512;
    result.target = address_model::wrap24(std::int64_t{result.instruction.pc} + signed_offset);
  } else {
    constexpr Word long_base = 0x0d1000;
    constexpr Word register_base = 0x0d1800;
    result.instruction.opcode =
        (input.form == Relative::Form::reg ? register_base | (input.index << 8) : long_base) |
        (input.conditional ? input.selector : 128U) | (input.call ? 0U : 64U);
    result.instruction.extension = input.displacement;
    result.instruction.words = input.form == Relative::Form::reg ? 1 : 2;
    if (result.instruction.words == 2) {
      result.instruction.cycles = 4U + static_cast<unsigned>(input.conditional != input.call);
    }
    result.target = static_cast<Word>((std::uint64_t{result.instruction.pc} + input.displacement) %
                                      address_words);
  }
  return result;
}

void relative_case(const Relative input) {
  auto point = relative(input);
  const a64::Program program(point.instruction);
  for (Word flags = 0; flags <= status_byte; ++flags) {
    auto initial = seed(flags);
    initial.pc = point.instruction.pc;
    initial.sp = return_model::pointer_range - 1;
    initial.sc = 14;
    initial.r.at(input.index) = input.displacement;
    jump_case(program, point, initial);
    initial.sc = 15;
    jump_case(program, point, initial);
  }
}

void relative_branches() {
  constexpr std::array<Word, 5> short_offsets{0, 1, 255, 256, 511};
  constexpr std::array<Word, 5> wide_offsets{0, 1, sign24 - 1, sign24, word_mask};

  for (const auto form : {Relative::Form::short_, Relative::Form::long_, Relative::Form::reg}) {
    for (const bool call : {false, true}) {
      for (unsigned selector = 0; selector < 16; ++selector) {
        for (const auto displacement :
             (form == Relative::Form::short_ ? short_offsets : wide_offsets)) {
          relative_case({.form = form,
                         .displacement = displacement,
                         .selector = selector,
                         .index = selector % static_cast<unsigned>(agu_registers),
                         .call = call});
        }
      }
      relative_case({.form = form, .displacement = 0, .call = call, .conditional = false});
    }
  }
}

void returns() {
  for (const bool restore_status : {false, true}) {
    const a64::Instruction input{.opcode = restore_status ? return_model::rti : return_model::rts,
                                 .cycles = 3};
    const a64::Program program(input);
    for (Word pointer = 0; pointer < return_model::pointer_range; ++pointer) {
      for (const Word count : {0U, 1U, 15U, 31U}) {
        for (const Word status : {0U, word_mask, mode_rm | flag_c, mode_up | mode_down}) {
          auto initial = return_model::seed(
              {.pc = word_mask, .status = status, .pointer = pointer, .count = count});
          initial.pc = start;
          initial.sr &= ~return_model::lf;
          a64::Expected expected{.reg = initial, .cycles = 3};
          if (count == 0) {
            ++expected.reg.pc;
            expected.retired = 0;
            expected.cycles = 0;
            expected.failure = {.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = start,
                                .value = pointer,
                                .pc = start};
            expected.boundary = detail::NativeBoundary::owner;
            expected.repeat_completed = true;
          } else {
            return_model::apply(expected.reg, restore_status);
            if (restore_status) {
              expected.inhibited = true;
              expected.boundary = detail::NativeBoundary::owner;
            }
          }
          a64::Invocation invocation(initial);
          invocation.run(program);
          invocation.expect(expected, input.opcode);
        }
      }
    }
  }
}

void control_bytes() {
  for (const auto field : {byte_model::Field::mr,
                           byte_model::Field::ccr,
                           byte_model::Field::com,
                           byte_model::Field::eom}) {
    for (const bool inclusive_or : {false, true}) {
      for (Word immediate = 0; immediate <= status_byte; ++immediate) {
        const byte_model::Input point{
            .field = field, .immediate = immediate, .inclusive_or = inclusive_or};
        const a64::Instruction input{.opcode = byte_model::opcode(point), .cycles = 3};
        const a64::Program program(input);
        for (Word old = 0; old <= status_byte; ++old) {
          auto initial = byte_model::seed(field, old);
          a64::Expected expected{.reg = initial, .cycles = 3};
          byte_model::apply(expected.reg, point);
          ++expected.reg.pc;
          if (field == byte_model::Field::mr) {
            expected.boundary = detail::NativeBoundary::context;
          }
          a64::Invocation invocation(initial);
          invocation.run(program);
          invocation.expect(expected, input.opcode);
        }
      }
    }
  }
}

struct Agu {
  Word modifier = address_model::linear, cursor = 0, offset = 0;
  unsigned mode = 4, index = 0;
  bool valid = true;
};

a64::Expected address_expected(const Registers &initial, Jump point, const Agu input) {
  a64::Expected expected{.reg = initial};
  expected.reg.pc = (point.instruction.pc + point.instruction.words) % address_words;
  bool valid = true;
  FaultKind failure = FaultKind::none;
  Word value = 0;
  if (input.mode == 6) {
    valid = input.index == 0;
    failure = FaultKind::invalid_instruction;
    value = point.instruction.opcode;
    point.target = point.instruction.extension;
  } else if (input.mode == 4) {
    point.target = input.cursor;
  } else if (!input.valid) {
    valid = false;
    failure = FaultKind::unsupported_mode;
    constexpr Word modulus = 0x10000;
    value = input.modifier % modulus;
  } else {
    const auto next = address_model::updated({.modifier = input.modifier,
                                              .cursor = input.cursor,
                                              .offset = input.offset,
                                              .mode = input.mode,
                                              .index = input.index});
    if (input.mode != 5) { expected.reg.r.at(input.index) = next; }
    point.target = input.mode == 5 || input.mode == 7 ? next : input.cursor;
  }
  if (!valid) {
    expected.retired = 0;
    expected.cycles = 0;
    expected.failure = {.kind = failure,
                        .space = Space::program,
                        .address = point.instruction.pc,
                        .value = value,
                        .pc = point.instruction.pc};
    expected.boundary = detail::NativeBoundary::owner;
    expected.repeat_completed = true;
  } else {
    jump_model(expected, point);
  }
  return expected;
}

void address_case(const a64::Program &program, const Jump point, const Agu input) {
  for (const Word flags : {0U, flag_c, status_byte}) {
    auto initial = seed(flags | point.instruction.mode);
    initial.m.at(input.index) = input.modifier;
    initial.r.at(input.index) = input.cursor;
    initial.n.at(input.index) = input.offset;
    for (const Word count : {14U, 15U}) {
      initial.sc = count;
      initial.sp = return_model::pointer_range - 1;
      const auto expected = address_expected(initial, point, input);
      a64::Invocation invocation(initial);
      invocation.run(program);
      invocation.expect(expected, point.instruction.opcode);
    }
  }
}

void address_operands(const a64::Program &program, const Jump point, Agu input) {
  const auto modifier = input.modifier % 0x10000;
  if (!input.valid || modifier == 0 || modifier == address_model::linear) {
    constexpr std::array<Word, 5> pointers{0, 1, 0x123456, sign24, word_mask};
    constexpr std::array<Word, 6> offsets{0, 1, 2, sign24, 0x55aaaa, word_mask};
    for (const auto cursor : pointers) {
      for (const auto offset : offsets) {
        input.cursor = cursor;
        input.offset = offset;
        address_case(program, point, input);
      }
    }
    return;
  }
  const auto length = modifier >= 0x8000 ? (modifier % 0x4000) + 1 : modifier + 1;
  const auto span = address_model::span(length);
  const Word base = static_cast<Word>(address_words) - span;
  const Word stride = span * 2;
  const Word negative_stride = static_cast<Word>(address_words) - stride;
  for (const Word position : {0U, length / 2, length - 1}) {
    input.cursor = base + position;
    for (const Word offset : {0U,
                              1U,
                              length - 1,
                              length,
                              word_mask,
                              static_cast<Word>(address_words - length),
                              stride,
                              negative_stride}) {
      input.offset = offset;
      address_case(program, point, input);
    }
  }
}

struct Effective {
  unsigned mode = 4, index = 0;
  bool call = false, conditional = true;
};

Jump effective_jump(const Effective input) {
  constexpr Word jump_base = 0x0ac0a0;
  constexpr Word call_base = 0x0bc0a0;
  constexpr Word ignored_numeric = detail::compatibility_mode | detail::multiply_mode |
                                   detail::arithmetic_16 | detail::scale_down | detail::scale_up;
  const auto base = input.call ? call_base : jump_base;
  const Word operation = input.conditional ? base : base & ~Word{0x20};
  return {.instruction = {.opcode = operation | (input.mode << 11) | (input.index << 8),
                          .extension = word_mask,
                          .mode = ignored_numeric,
                          .words = input.mode == 6 ? 2U : 1U,
                          .cycles =
                              input.conditional ? 4U : 3U + static_cast<unsigned>(input.mode >= 5)},
          .call = input.call,
          .condition = input.conditional ? std::optional{0U} : std::nullopt};
}

void effective_jumps() {
  struct Modifier {
    Word value;
    bool valid = true;
  };

  constexpr std::array modifiers{Modifier{.value = 0xffff},
                                 Modifier{.value = 0xff0000},
                                 Modifier{.value = 0x12ffff},
                                 Modifier{.value = 1},
                                 Modifier{.value = 2},
                                 Modifier{.value = 3},
                                 Modifier{.value = 6},
                                 Modifier{.value = 7},
                                 Modifier{.value = 255},
                                 Modifier{.value = 256},
                                 Modifier{.value = 32767},
                                 Modifier{.value = 0x8001},
                                 Modifier{.value = 0x8003},
                                 Modifier{.value = 0x807f},
                                 Modifier{.value = 0x83ff},
                                 Modifier{.value = 0xffbfff},
                                 Modifier{.value = 0xc001, .valid = false},
                                 Modifier{.value = 0x8000, .valid = false},
                                 Modifier{.value = 0x8002, .valid = false}};
  for (const bool call : {false, true}) {
    for (const bool conditional : {false, true}) {
      for (unsigned mode = 0; mode < 8; ++mode) {
        for (unsigned index = 0; index < agu_registers; ++index) {
          const auto point = effective_jump(
              {.mode = mode, .index = index, .call = call, .conditional = conditional});
          const a64::Program program(point.instruction);
          for (const auto modifier : modifiers) {
            address_operands(program,
                             point,
                             {.modifier = modifier.value,
                              .mode = mode,
                              .index = index,
                              .valid = modifier.valid});
          }
        }
      }
    }
  }
}

void context_boundaries() {
  constexpr Word clear = 0x200013;
  const byte_model::Input control{
      .field = byte_model::Field::mr, .immediate = 4, .inclusive_or = true};
  const std::array mode_steps{a64::Instruction{.opcode = byte_model::opcode(control), .cycles = 3},
                              a64::Instruction{.opcode = clear, .pc = start + 1}};
  const a64::Program mode_program(mode_steps);
  const auto before = seed(status_byte);
  auto after = before;
  byte_model::apply(after, control);
  ++after.pc;
  a64::Invocation mode_invocation(before);
  mode_invocation.run(mode_program, 2);
  mode_invocation.expect({.reg = after, .cycles = 3, .boundary = detail::NativeBoundary::context},
                         mode_steps.front().opcode);
  const std::array return_steps{a64::Instruction{.opcode = return_model::rti, .cycles = 3},
                                a64::Instruction{.opcode = clear, .pc = start + 1}};
  const a64::Program return_program(return_steps);
  auto initial = return_model::seed({.pc = start + 1, .status = mode_up | flag_c});
  initial.pc = start;
  initial.sr = status_byte;
  auto expected = initial;
  return_model::apply(expected);
  a64::Invocation returned(initial);
  returned.run(return_program, 2);
  returned.expect(
      {.reg = expected, .cycles = 3, .boundary = detail::NativeBoundary::owner, .inhibited = true},
      return_steps.front().opcode);
}

void boundaries() {
  constexpr Word clear = 0x200013;
  for (Word flags = 0; flags <= status_byte; ++flags) {
    constexpr Word target = 0x500;
    constexpr Word branch = 0x0ea000;
    const std::array steps{a64::Instruction{.opcode = branch | target, .cycles = 4},
                           a64::Instruction{.opcode = clear, .pc = start + 1}};
    const a64::Program program(steps);
    auto initial = seed(flags);
    a64::Expected expected{.reg = initial, .cycles = 4};
    if (selected({.status = flags, .code = 10})) {
      expected.reg.pc = target;
    } else {
      sequence_model::alu(expected.reg, expected.reg, clear);
      expected.reg.pc = start + 2;
      expected.retired = 2;
      expected.cycles = 5;
    }
    a64::Invocation invocation(initial);
    invocation.run(program, 2);
    invocation.expect(expected, steps.front().opcode);
  }
  for (const auto state : {ExecutionState::waiting, ExecutionState::stopped}) {
    constexpr Word wait = 0x86;
    constexpr Word stop = 0x87;
    const std::array steps{
        a64::Instruction{.opcode = state == ExecutionState::waiting ? wait : stop, .cycles = 10},
        a64::Instruction{.opcode = clear, .pc = start + 1}};
    const a64::Program program(steps);
    const auto initial = seed(status_byte);
    auto expected = initial;
    ++expected.pc;
    a64::Invocation invocation(initial);
    invocation.run(program, 2);
    invocation.expect({.reg = expected,
                       .cycles = 10,
                       .processing = state,
                       .boundary = detail::NativeBoundary::owner},
                      steps.front().opcode);
  }
  constexpr Word unknown = 0xff;
  const a64::Program program(a64::Instruction{.opcode = unknown});
  const auto initial = seed(status_byte);
  auto expected = initial;
  ++expected.pc;
  a64::Invocation invocation(initial);
  invocation.run(program);
  invocation.expect({.reg = expected,
                     .retired = 0,
                     .cycles = 0,
                     .failure = {.kind = FaultKind::unsupported_instruction,
                                 .space = Space::program,
                                 .address = start,
                                 .value = unknown,
                                 .pc = start},
                     .boundary = detail::NativeBoundary::owner,
                     .repeat_completed = true},
                    unknown);
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    short_jumps();
    relative_branches();
    returns();
    control_bytes();
    boundaries();
    effective_jumps();
    context_boundaries();
    std::cout << "A64 control-flow, stack, byte logic and boundaries passed: " << checks
              << " checks\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
