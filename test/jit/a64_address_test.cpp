// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
enum class Form : std::uint8_t {
  relative_register,
  relative_long,
  updated,
  displacement,
  parallel
};
constexpr Word cm = Word{1} << 13, mul = Word{1} << 14, a16 = Word{1} << 17;
constexpr std::array modes{0U,
                           mode_down,
                           mode_up,
                           mode_rm,
                           saturation_model::mode,
                           saturation_model::mode | mode_down | mode_rm,
                           cm | mul | a16,
                           mode_up | mode_down};

struct Address {
  Form form = Form::updated;
  unsigned index = 0, mode = 0, target = 4;
  Word offset = 0, status = 0, alu = 0;
};

a64::Instruction instruction(const Address point) {
  constexpr Word relative_register = 0x04c000;
  constexpr Word relative_long = 0x044040;
  constexpr Word updated = 0x044000;
  constexpr Word displacement = 0x040000;
  constexpr Word parallel = 0x204000;
  a64::Instruction result{.mode = point.status, .cycles = 3};
  const auto index = point.index << 8;
  switch (point.form) {
    case Form::relative_register: result.opcode = relative_register | index | point.target; break;
    case Form::relative_long:
      result.opcode = relative_long | point.target;
      result.extension = point.offset;
      result.words = 2;
      break;
    case Form::updated: result.opcode = updated | index | (point.mode << 11) | point.target; break;
    case Form::displacement: {
      constexpr Word upper = 0x70;
      result.opcode = displacement | ((point.offset & upper) << 7) | index |
                      ((point.offset & 15U) << 4) | (point.target - 16);
      break;
    }
    case Form::parallel:
      result.opcode = parallel | index | (point.mode << 11) | point.alu;
      result.cycles = 1;
      break;
    default: break;
  }
  return result;
}

struct Modifier {
  Word value;
  bool valid = true;
};

struct Seed {
  Word modifier, status;
  unsigned position;
};

Registers seed(const Address point, const Seed input) {
  auto state = register_model::seed(0);
  state.sr = input.status;
  const auto modifier = input.modifier % 0x10000;
  const bool ring = modifier > 0 && modifier < 0x8000;
  const auto length = ring ? modifier + 1 : 256U;
  const auto span = address_model::span(length);
  const Word base = word_mask - span + 1;
  const std::array cursors{base, base + (length / 2), base + length - 1};
  const std::array offsets{0U, length, word_mask};
  state.r.at(point.index) = cursors.at(input.position);
  state.n.at(point.index) = offsets.at(input.position);
  state.m.at(point.index) = input.modifier;
  if (point.form == Form::relative_register) { state.r.at(point.index) = point.offset; }
  return state;
}

void fail(a64::Expected &expected, const FaultKind kind, const Word value) {
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
  expected.failure = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

a64::Expected model(const Registers &initial, const Address point, const Modifier modifier) {
  const auto input = instruction(point);
  a64::Expected result{.reg = initial, .cycles = input.cycles};
  result.reg.pc = (start + input.words) % static_cast<Word>(address_words);
  const bool relative = point.form == Form::relative_long || point.form == Form::relative_register;
  const bool invalid_alu = point.alu == 4 || point.alu == 8 || point.alu == 12;
  if ((initial.sr & (cm | mul | a16)) != 0 ||
      (initial.sr & (mode_down | mode_up)) == (mode_down | mode_up) ||
      (invalid_alu && (initial.sr & saturation_model::mode) != 0)) {
    fail(result, FaultKind::unsupported_mode, initial.sr);
    return result;
  }
  if (point.target < 4 && point.form != Form::parallel) {
    fail(result, FaultKind::invalid_instruction, input.opcode);
    return result;
  }
  if (!relative && !modifier.valid) {
    fail(result, FaultKind::unsupported_mode, modifier.value % 0x10000);
    return result;
  }
  Word value = 0;
  if (relative) {
    value = address_model::wrap24(start + signed24(point.offset));
  } else {
    auto offset = initial.n.at(point.index);
    auto mode = point.mode;
    if (point.form == Form::displacement) {
      offset = point.offset < 64 ? point.offset : point.offset + word_mask + 1 - 128;
      mode = 1;
    }
    value = address_model::updated({.modifier = modifier.value,
                                    .cursor = initial.r.at(point.index),
                                    .offset = offset,
                                    .mode = mode});
  }
  if (point.form == Form::parallel) {
    result.reg.r.at(point.index) = value;
    if (invalid_alu) {
      fail(result,
           point.alu == 8 ? FaultKind::invalid_instruction : FaultKind::unsupported_instruction,
           point.alu);
    } else {
      a64_alu_model::apply(result.reg, initial, point.alu);
    }
  } else {
    register_model::write(result.reg, point.target, value);
  }
  return result;
}

void verify(const Address point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  constexpr std::array modifiers{Modifier{.value = 0xffff},
                                 Modifier{.value = 0},
                                 Modifier{.value = 0xab0000},
                                 Modifier{.value = 255},
                                 Modifier{.value = 256},
                                 Modifier{.value = 32767},
                                 Modifier{.value = 0x8001},
                                 Modifier{.value = 0x807f},
                                 Modifier{.value = 0x8000, .valid = false},
                                 Modifier{.value = 0xc001, .valid = false}};
  const auto empty_state = seed(point, {.modifier = 0xffff, .status = point.status, .position = 0});
  a64::Invocation empty(empty_state);
  empty.run(program, 0);
  empty.expect(
      {.reg = empty_state, .retired = 0, .cycles = 0, .inhibited = true, .repeat_completed = true},
      input.opcode);
  for (const auto modifier : modifiers) {
    for (unsigned position = 0; position < 3; ++position) {
      for (const Word flags : {0U, status_byte}) {
        const auto initial = seed(
            point,
            {.modifier = modifier.value, .status = point.status | flags, .position = position});
        a64::Invocation invocation(initial);
        invocation.run(program);
        invocation.expect(model(initial, point, modifier), input.opcode);
      }
    }
  }
}

void relative() {
  for (const auto form : {Form::relative_register, Form::relative_long}) {
    for (unsigned target = 0; target < 32; ++target) {
      for (const Word offset : {0U, 1U, sign24 - 1, sign24, word_mask}) {
        for (const auto mode : modes) {
          verify({.form = form,
                  .index = target % 8,
                  .target = target,
                  .offset = offset,
                  .status = mode});
        }
      }
    }
  }
}

void updated() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (unsigned target = 0; target < 32; ++target) {
        for (const auto status : modes) {
          verify({.index = index, .mode = mode, .target = target, .status = status});
        }
      }
    }
  }
}

void displacement() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (unsigned target = 16; target < 32; ++target) {
      for (const Word offset : {0U, 1U, 63U, 64U, 127U}) {
        for (const auto status : modes) {
          verify({.form = Form::displacement,
                  .index = index,
                  .target = target,
                  .offset = offset,
                  .status = status});
        }
      }
    }
  }
}

void parallel() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (const Word alu : {0U, 4U, 8U, 12U, 0x13U, 0x41U, 0xd2U, 0x17U, 0x32U}) {
        for (const auto status : modes) {
          verify(
              {.form = Form::parallel, .index = index, .mode = mode, .status = status, .alu = alu});
        }
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::relative();
    dsp56300::test::updated();
    dsp56300::test::displacement();
    dsp56300::test::parallel();
    std::cout << dsp56300::test::checks << " A64 address calculation/update checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
