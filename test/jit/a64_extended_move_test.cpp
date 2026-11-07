// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "move_model.hpp"
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
using move_model::Form;
using move_model::Kind;
constexpr Word high = 0x923456, low = 0x765432;
constexpr Word cm = Word{1} << 13, mul = Word{1} << 14, sa = Word{1} << 17;
constexpr std::array modes{0U,
                           mode_down,
                           mode_up,
                           mode_rm,
                           saturation_model::mode,
                           saturation_model::mode | mode_up,
                           saturation_model::mode | mode_down,
                           cm | mul | sa,
                           mode_down | mode_up};
constexpr std::array raw{std::uint64_t{0},
                         std::uint64_t{1},
                         sign56 - 1,
                         sign56,
                         accumulator_mask,
                         saturation_model::positive,
                         saturation_model::positive + 1,
                         saturation_model::negative,
                         saturation_model::negative - 1};

struct Point {
  Form form{};
  Word alu = 0, status = 0;
  bool aliases = false;
};

a64::Instruction instruction(const Point point) {
  return {.opcode = move_model::opcode(point.form, point.alu),
          .extension = point.form.extension,
          .mode = point.status,
          .words = point.form.kind == Kind::mixed_two ? 1U : move_model::words(point.form),
          .cycles = move_model::cycles(point.form)};
}

void fail(a64::Expected &expected, const FaultKind kind, const Word value) {
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
  expected.failure = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

bool validate_form(a64::Expected &expected, const Point point) {
  const auto form = point.form;
  if (form.short_form || form.mode != 6) { return true; }
  if (form.kind == Kind::mixed_two) {
    fail(expected, FaultKind::invalid_instruction, instruction(point).opcode);
    return false;
  }
  const bool immediate = form.kind == Kind::mixed_one && form.index == 4 && form.load;
  if (form.index != 0 && !immediate) {
    fail(expected, FaultKind::invalid_instruction, form.index);
    return false;
  }
  return true;
}

a64::Expected model(const Registers &initial,
                    const Point point,
                    std::array<Word, memory_words> &x,
                    std::array<Word, memory_words> &y) {
  const auto input = instruction(point);
  const auto form = point.form;
  a64::Expected expected{.reg = initial, .cycles = input.cycles};
  expected.reg.pc += input.words;
  const bool invalid_alu = point.alu == 4 || point.alu == 8 || point.alu == 12;
  const auto status = initial.sr;
  if ((status & (cm | mul | sa)) != 0 ||
      (status & (mode_up | mode_down)) == (mode_up | mode_down) ||
      (invalid_alu && (status & saturation_model::mode) != 0)) {
    fail(expected, FaultKind::unsupported_mode, status);
    return expected;
  }
  if (!validate_form(expected, point)) { return expected; }
  const move_model::Data data{
      .high = form.kind == Kind::mixed_one && form.y && !point.aliases ? low : high,
      .low = point.aliases ? high : low};
  const auto output = move_model::apply(expected.reg, form, data);
  if (output.stores != 0) {
    auto &target = output.space == Space::x || point.aliases ? x : y;
    target.at(output.address) = output.first;
    if (output.stores == 2) { (point.aliases ? x : y).at(output.address) = output.second; }
  }
  if (invalid_alu) {
    fail(expected,
         point.alu == 8 ? FaultKind::invalid_instruction : FaultKind::unsupported_instruction,
         point.alu);
  } else {
    a64_alu_model::apply(expected.reg, initial, point.alu);
  }
  return expected;
}

void probe(const Point point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  for (const Word flags : {0U, status_byte}) {
    for (std::size_t index = 0; index < raw.size(); ++index) {
      auto initial = register_model::seed(0);
      initial.sr = point.status | flags;
      initial.a = raw.at(index);
      initial.b = raw.at((index + 3) % raw.size());
      initial.r.at(point.form.index) = move_model::default_address;
      initial.n.at(point.form.index) = 1;
      std::array<Word, memory_words> x{};
      std::array<Word, memory_words> y{};
      constexpr Word high_fill = 0xab923456;
      constexpr Word low_fill = 0xcd765432;
      x.fill(high_fill);
      y.fill(low_fill);
      auto expected_x = x;
      auto expected_y = y;
      const auto expected = model(initial, point, expected_x, expected_y);
      a64::Invocation invocation(initial);
      a64::Bus bus(invocation, {});
      invocation.bind({{{.data = x.data(), .words = x.size()},
                        {.data = point.aliases ? x.data() : y.data(), .words = y.size()}}},
                      bus.binding());
      invocation.run(program);
      invocation.expect(expected, input.opcode);
      bus.expect();
      require(x == expected_x && y == expected_y,
              "Long/mixed moves preserve every P/X/Y word, including aliases");
    }
  }
}

void long_moves() {
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const bool load : {false, true}) {
      for (unsigned addressing = 0; addressing < 8; ++addressing) {
        const Form form{.selector = selector,
                        .mode = addressing,
                        .index = addressing == 6 ? 0U : (selector + addressing) % 8,
                        .load = load};
        for (const auto status : modes) {
          for (const bool aliases : {false, true}) {
            probe({.form = form, .status = status, .aliases = aliases});
          }
        }
      }
      for (const auto status : modes) {
        constexpr Word short_address = 31;
        probe({.form = {.selector = selector,
                        .extension = short_address,
                        .load = load,
                        .short_form = true},
               .status = status});
      }
    }
  }
}

void mixed_variants(const Form form) {
  constexpr Word mac = 0xd2;
  for (const bool load : {false, true}) {
    auto variant = form;
    variant.load = load;
    for (const Word status : {0U, mode_down, mode_up, saturation_model::mode}) {
      probe({.form = variant, .alu = mac, .status = status});
    }
  }
}

void class_two_variants(const bool y) {
  for (unsigned selector = 0; selector < 2; ++selector) {
    for (unsigned addressing = 0; addressing < 8; ++addressing) {
      mixed_variants({.kind = Kind::mixed_two, .selector = selector, .mode = addressing, .y = y});
    }
  }
}

void mixed_moves() {
  for (const bool y : {false, true}) {
    for (unsigned selector = 0; selector < 4; ++selector) {
      for (const bool source_b : {false, true}) {
        for (const bool second_high : {false, true}) {
          for (unsigned addressing = 0; addressing < 8; ++addressing) {
            mixed_variants({.kind = Kind::mixed_one,
                            .selector = selector,
                            .mode = addressing,
                            .index = addressing == 6 ? 0U : (selector + addressing) % 8,
                            .y = y,
                            .source_b = source_b,
                            .second_high = second_high});
          }
          mixed_variants({.kind = Kind::mixed_one,
                          .selector = selector,
                          .mode = 6,
                          .index = 4,
                          .y = y,
                          .source_b = source_b,
                          .second_high = second_high});
        }
      }
    }
    class_two_variants(y);
  }
}

void alu_latching() {
  constexpr std::array forms{Form{.selector = 0},
                             Form{.selector = 2},
                             Form{.selector = 4},
                             Form{.selector = 6, .load = false},
                             Form{.kind = Kind::mixed_one, .selector = 2, .source_b = true},
                             Form{.kind = Kind::mixed_one, .selector = 3, .load = false, .y = true},
                             Form{.kind = Kind::mixed_two, .selector = 0}};
  for (const auto form : forms) {
    for (Word alu = 0; alu <= status_byte; ++alu) {
      probe({.form = form, .alu = alu});
      probe({.form = form, .alu = alu, .status = saturation_model::mode | mode_up});
    }
    for (unsigned index = 1; index < agu_registers; ++index) {
      auto invalid = form;
      invalid.mode = 6;
      invalid.index = index;
      probe({.form = invalid});
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::long_moves();
    dsp56300::test::mixed_moves();
    dsp56300::test::alu_latching();
    std::cout << dsp56300::test::checks << " independent A64 long/mixed/ALU checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
