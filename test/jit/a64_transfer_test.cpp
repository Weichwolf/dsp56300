// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_native.hpp"
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
constexpr Word cm = Word{1} << 13, mul = Word{1} << 14, a16 = Word{1} << 17;
constexpr std::array modes{0U,
                           mode_down,
                           mode_up,
                           mode_rm,
                           saturation_model::mode,
                           saturation_model::mode | mode_up,
                           cm | mul | a16,
                           mode_up | mode_down};
constexpr std::array raw{std::uint64_t{0},
                         std::uint64_t{1},
                         saturation_model::positive,
                         saturation_model::positive + 1,
                         saturation_model::negative - 1,
                         saturation_model::negative,
                         sign56 - 1,
                         sign56,
                         accumulator_mask};

struct Seed {
  Word status;
  std::size_t index;
};

Registers seed(const Seed input) {
  auto result = register_model::seed(0);
  result.sr = input.status;
  result.a = raw.at(input.index % raw.size());
  result.b = raw.at((input.index + 3) % raw.size());
  constexpr std::array bank_values{std::uint64_t{0},
                                   std::uint64_t{0x800000800000},
                                   std::uint64_t{0x7fffff7fffff},
                                   std::uint64_t{0xffffff000001}};
  result.x = bank_values.at(input.index % bank_values.size());
  result.y = bank_values.at((input.index + 1) % bank_values.size());
  constexpr std::array counts{0U, 1U, 14U, 15U, 31U};
  constexpr std::array pointers{0U, 15U, 31U, 63U};
  result.sc = counts.at(input.index % counts.size());
  result.sp = pointers.at(input.index % pointers.size());
  return result;
}

void fail(a64::Expected &expected, const FaultKind kind, const Word value) {
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
  expected.failure = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

bool read(a64::Expected &expected, const unsigned number, Word &value) {
  if (number == register_model::reg_ssh && expected.reg.sc == 0) {
    fail(expected, FaultKind::stack, expected.reg.sp);
    return false;
  }
  value = register_model::read(expected.reg, number);
  return true;
}

void write(a64::Expected &expected, const unsigned number, const Word value) {
  if (number == register_model::reg_ssh && expected.reg.sc >= stack_entries - 1) {
    fail(expected, FaultKind::stack, expected.reg.sp);
    return;
  }
  register_model::write(expected.reg, number, value);
  if (number == register_model::reg_sr || number == register_model::reg_omr ||
      number == register_model::reg_la) {
    expected.boundary = detail::NativeBoundary::owner;
  }
}

bool numeric(const Word status) {
  return (status & (cm | mul | a16)) == 0 &&
         (status & (mode_down | mode_up)) != (mode_down | mode_up);
}

struct Move {
  enum class Kind : std::uint8_t { parallel, immediate, control, control_immediate };
  Kind kind = Kind::parallel;
  unsigned source = 4, target = 4;
  Word alu = 0, immediate = 0, mode = 0;
  bool load = false;
};

a64::Instruction instruction(const Move point) {
  Word opcode = 0;
  if (point.kind == Move::Kind::parallel) {
    opcode = parallel_none | (point.source << 13) | (point.target << 8) | point.alu;
  } else if (point.kind == Move::Kind::immediate) {
    opcode = parallel_none | (point.target << 16) | (point.immediate << 8) | point.alu;
  } else if (point.kind == Move::Kind::control) {
    constexpr Word base = 0x0440a0;
    opcode = base | (point.source << 8) | (point.target - 32) | (point.load ? 32768U : 0U);
  } else {
    constexpr Word base = 0x0500a0;
    opcode = base | (point.immediate << 8) | (point.target - 32);
  }
  return {.opcode = opcode, .mode = point.mode};
}

a64::Expected control_expected(const Registers &initial, const Move point) {
  a64::Expected result{.reg = initial};
  ++result.reg.pc;
  if (!register_model::valid(point.target) ||
      (point.kind == Move::Kind::control && !register_model::valid(point.source))) {
    fail(result, FaultKind::invalid_instruction, instruction(point).opcode);
  } else if (point.kind == Move::Kind::control_immediate) {
    write(result, point.target, point.immediate);
  } else if ((point.source == 14 || point.source == 15) && !numeric(initial.sr)) {
    fail(result, FaultKind::unsupported_mode, initial.sr);
  } else {
    Word value = 0;
    if (read(result, point.load ? point.source : point.target, value)) {
      write(result, point.load ? point.target : point.source, value);
    }
  }
  return result;
}

a64::Expected parallel_expected(const Registers &initial, const Move point) {
  a64::Expected result{.reg = initial};
  ++result.reg.pc;
  const bool invalid_alu = point.alu == 4 || point.alu == 8 || point.alu == 12;
  if (!numeric(initial.sr) || (invalid_alu && (initial.sr & saturation_model::mode) != 0)) {
    fail(result, FaultKind::unsupported_mode, initial.sr);
    return result;
  }
  if (point.kind == Move::Kind::immediate) {
    const bool shifted =
        (point.target >= 4 && point.target <= 7) || point.target == 14 || point.target == 15;
    register_model::write(result.reg, point.target, point.immediate * (shifted ? 65536U : 1U));
  } else if (point.source == 0 && point.target == 0) {
  } else if (point.source < 4) {
    if (point.target >= 4) { register_model::write(result.reg, point.target, 0); }
    fail(result, FaultKind::invalid_instruction, point.source);
    return result;
  } else {
    const auto value = register_model::read(result.reg, point.source);
    if (point.target < 4) {
      fail(result, FaultKind::invalid_instruction, point.target);
      return result;
    }
    register_model::write(result.reg, point.target, value);
  }
  if (invalid_alu) {
    fail(result,
         point.alu == 8 ? FaultKind::invalid_instruction : FaultKind::unsupported_instruction,
         point.alu);
  } else {
    a64_alu_model::apply(result.reg, initial, point.alu);
  }
  return result;
}

void verify(const Move point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  const auto empty_state = seed({.status = point.mode | status_byte, .index = 0});
  a64::Invocation empty(empty_state);
  empty.run(program, 0);
  empty.expect(
      {.reg = empty_state, .retired = 0, .cycles = 0, .inhibited = true, .repeat_completed = true},
      input.opcode);
  const bool control =
      point.kind == Move::Kind::control || point.kind == Move::Kind::control_immediate;
  for (const Word flags : {0U, flag_c | flag_v, flag_s | flag_l, status_byte}) {
    for (std::size_t index = 0; index < raw.size(); ++index) {
      const auto initial = seed({.status = point.mode | flags, .index = index});
      const auto expected =
          control ? control_expected(initial, point) : parallel_expected(initial, point);
      a64::Invocation invocation(initial);
      invocation.run(program);
      invocation.expect(expected, input.opcode);
    }
  }
}

void register_moves() {
  for (unsigned source = 4; source < 32; ++source) {
    for (unsigned target = 0; target < 32; ++target) {
      for (const auto mode : modes) { verify({.source = source, .target = target, .mode = mode}); }
    }
  }
  for (const unsigned source : {0U, 3U}) {
    for (unsigned target = 0; target < 32; ++target) {
      for (const auto mode : modes) { verify({.source = source, .target = target, .mode = mode}); }
    }
  }
}

void immediate_moves() {
  for (unsigned target = 4; target < 32; ++target) {
    for (Word immediate = 0; immediate <= status_byte; ++immediate) {
      for (const Word mode : {0U, saturation_model::mode, mode_up, mode_down}) {
        verify({.kind = Move::Kind::immediate,
                .target = target,
                .immediate = immediate,
                .mode = mode});
      }
    }
  }
}

void alu_latching() {
  struct Pair {
    unsigned source, target;
  };

  constexpr std::array pairs{Pair{.source = 14, .target = 4},
                             Pair{.source = 4, .target = 14},
                             Pair{.source = 15, .target = 14},
                             Pair{.source = 14, .target = 15},
                             Pair{.source = 8, .target = 12},
                             Pair{.source = 10, .target = 9},
                             Pair{.source = 4, .target = 8},
                             Pair{.source = 4, .target = 10},
                             Pair{.source = 4, .target = 12},
                             Pair{.source = 6, .target = 5}};
  for (Word code = 0; code <= status_byte; ++code) {
    for (const auto pair : pairs) {
      for (const auto mode : modes) {
        verify({.source = pair.source, .target = pair.target, .alu = code, .mode = mode});
      }
    }
  }
}

void control_moves() {
  for (unsigned target = 32; target < 64; ++target) {
    for (unsigned source = 0; source < 64; ++source) {
      for (const bool load : {false, true}) {
        for (const auto mode : modes) {
          verify({.kind = Move::Kind::control,
                  .source = source,
                  .target = target,
                  .mode = mode,
                  .load = load});
        }
      }
    }
    for (Word immediate = 0; immediate <= status_byte; ++immediate) {
      for (const Word mode : {0U, cm | mul | a16 | mode_up | mode_down}) {
        verify({.kind = Move::Kind::control_immediate,
                .target = target,
                .immediate = immediate,
                .mode = mode});
      }
    }
  }
}

void owner_boundaries() {
  for (const unsigned target : {unsigned{register_model::reg_sr},
                                unsigned{register_model::reg_omr},
                                unsigned{register_model::reg_la}}) {
    const Move point{.kind = Move::Kind::control_immediate, .target = target, .immediate = 7};
    const std::array steps{instruction(point),
                           a64::Instruction{.opcode = 0x200013, .pc = start + 1}};
    const a64::Program program(steps);
    const auto initial = seed({.status = status_byte, .index = 0});
    a64::Invocation invocation(initial);
    invocation.run(program, 2);
    invocation.expect(control_expected(initial, point), steps.front().opcode);
  }
}

void prefixes() {
  constexpr std::array forms{
      Move{.source = 4, .target = 14, .alu = 0x13},
      Move{.source = 15, .target = 4, .alu = 0xd2},
      Move{.source = 14, .target = 15, .alu = 0x17},
      Move{.kind = Move::Kind::immediate, .target = 16, .alu = 0x41, .immediate = 255},
      Move{.kind = Move::Kind::control, .source = 16, .target = 32, .load = true},
      Move{.kind = Move::Kind::control, .source = 8, .target = 61, .load = true},
      Move{.source = 8, .target = 12, .alu = 0x27},
      Move{.kind = Move::Kind::control, .source = 5, .target = 57},
      Move{.source = 4, .target = 12, .alu = 0x4f}};
  for (const Word mode : {0U, saturation_model::mode | mode_up, mode_down, mode_rm}) {
    std::array<a64::Instruction, forms.size()> steps{};
    for (std::size_t index = 0; index < steps.size(); ++index) {
      auto point = forms[index];
      point.mode = mode;
      steps[index] = instruction(point);
      steps[index].pc = start + static_cast<Word>(index);
    }
    const a64::Program program(steps);
    for (Word budget = 0; budget <= steps.size(); ++budget) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        const auto initial = seed({.status = mode | flags, .index = flags});
        auto expected = initial;
        for (Word index = 0; index < budget; ++index) {
          const auto point = forms.at(index);
          const auto result = point.kind == Move::Kind::control
                                  ? control_expected(expected, point)
                                  : parallel_expected(expected, point);
          require(result.failure.kind == FaultKind::none, "Authored transfer prefix must be legal");
          expected = result.reg;
        }
        a64::Invocation invocation(initial);
        invocation.run(program, budget);
        invocation.expect({.reg = expected,
                           .retired = budget,
                           .cycles = budget,
                           .inhibited = budget == 0,
                           .repeat_completed = budget == 0},
                          steps.front().opcode);
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::register_moves();
    dsp56300::test::immediate_moves();
    dsp56300::test::alu_latching();
    dsp56300::test::control_moves();
    dsp56300::test::owner_boundaries();
    dsp56300::test::prefixes();
    std::cout << dsp56300::test::checks
              << " A64 register/parallel/control transfer checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
