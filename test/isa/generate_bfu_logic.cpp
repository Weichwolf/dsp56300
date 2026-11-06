// SPDX-License-Identifier: GPL-3.0-only
// Original BFU SM archives; only the portable fixture codec is linked.
#include "bfu_logic_model.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using bfu_model::Form;
using bfu_model::Function;
using bfu_model::Input;
using Cases = std::vector<BinaryCase>;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array functions{Function::bit_and,
                               Function::bit_or,
                               Function::bit_xor,
                               Function::complement,
                               Function::left,
                               Function::right,
                               Function::rotate_left,
                               Function::rotate_right};
constexpr std::array<Word, 8> values{
    0, 1, sign24 - 1, sign24, word_mask, 0x555555, 0xaaaaaa, 0x123456};
constexpr Word conditional = 0x202000, update = 0x1000, load_x0 = 0x44d800, datum = 0x987654;

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status | bfu_model::sm;
  const auto flags = status & status_byte;
  state.a = (std::uint64_t{flags} << register_model::extension_shift) |
            (std::uint64_t{values[flags % values.size()]} << word_bits) | word_mask;
  state.b = accumulator_mask ^ state.a;
  state.x ^= std::uint64_t{flags} << word_bits;
  return state;
}

std::vector<Input> shapes(const bool shifts) {
  std::vector<Input> result;
  for (const auto function : functions) {
    if (bfu_model::logical(function) == shifts) { continue; }
    const auto sources = function < Function::complement ? 4U : 1U;
    for (unsigned source = 0; source < sources; ++source) {
      for (const bool destination_b : {false, true}) {
        result.push_back({.function = function, .source = source, .destination_b = destination_b});
      }
    }
  }
  return result;
}

void append(Cases &cases, const Registers &initial, const Input input) {
  BinaryCase item;
  item.initial = initial;
  item.expected = initial;
  bfu_model::apply(item.expected, initial, input);
  item.expected.pc += bfu_model::words(input);
  std::vector<Word> program{bfu_model::opcode(input)};
  if (input.form == Form::long_immediate) { program.push_back(input.value); }
  item.before = {{.words = std::move(program), .address = start}};
  item.progress.estimated_cycles = bfu_model::words(input);
  item.name = "BFU SM bypass: raw middle24, preserved outer bits and independent flags";
  cases.push_back(std::move(item));
}

void flags(Cases &cases, const std::vector<Input> &inputs) {
  for (const auto mode : modes) {
    for (const auto input : inputs) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        append(cases, seed(mode | flags), input);
      }
    }
  }
}

void bits(Cases &cases, const std::vector<Input> &inputs) {
  for (const auto input : inputs) {
    for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
      for (const bool inverse : {false, true}) {
        auto state = seed(status_byte);
        state.a = state.b = (inverse ? accumulator_mask : 0) ^ (std::uint64_t{1} << bit);
        append(cases, state, input);
      }
    }
  }
}

void immediates(Cases &cases) {
  for (const auto mode : modes) {
    for (const auto function : {Function::bit_and, Function::bit_or, Function::bit_xor}) {
      for (const bool destination_b : {false, true}) {
        for (const auto value : values) {
          append(cases,
                 seed(mode | status_byte),
                 {.function = function,
                  .form = Form::long_immediate,
                  .value = value,
                  .destination_b = destination_b});
        }
        for (const Word value : {0U, 1U, 31U, 32U, 63U}) {
          append(cases,
                 seed(mode | flag_c | flag_u | flag_l),
                 {.function = function,
                  .form = Form::short_immediate,
                  .value = value,
                  .destination_b = destination_b});
        }
      }
    }
  }
}

void truth_patterns(Cases &cases, const std::vector<Input> &inputs) {
  constexpr std::array<std::array<Word, 2>, 8> pairs{{{0, 0},
                                                      {0, word_mask},
                                                      {word_mask, 0},
                                                      {word_mask, word_mask},
                                                      {0x555555, 0xaaaaaa},
                                                      {0xaaaaaa, 0x555555},
                                                      {0x123456, 0x123456},
                                                      {sign24, sign24}}};
  for (const auto mode : modes) {
    for (const auto input : inputs) {
      for (const auto pair : pairs) {
        auto state = seed(mode | status_byte);
        register_model::write(state, input.destination_b ? 13 : 12, pair[0]);
        register_model::write(state, 4 + input.source, pair[1]);
        append(cases, state, input);
      }
    }
  }
}

void condition_case(Cases &cases, const Input input, const Word status) {
  for (unsigned selector = 0; selector < 16; ++selector) {
    for (const bool publish : {false, true}) {
      BinaryCase item;
      item.initial = seed(status);
      item.expected = item.initial;
      if (selection_model::test_condition({.status = status, .selector = selector})) {
        bfu_model::apply(item.expected, item.initial, input);
        if (!publish) { item.expected.sr = item.initial.sr; }
      }
      ++item.expected.pc;
      item.before = {{.words = {conditional | (selector << 8) | (publish ? update : 0U) |
                                (bfu_model::opcode(input) & status_byte)},
                      .address = start}};
      item.name =
          "BFU IFcc/IFcc.U: old predicate, selected raw result and optional CCR publication";
      cases.push_back(std::move(item));
    }
  }
}

void conditionals(Cases &cases, const std::vector<Input> &inputs) {
  for (Word flags = 0; flags <= status_byte; ++flags) {
    condition_case(cases, {.function = Function::bit_or}, flags | bfu_model::sm);
  }
  for (const auto input : inputs) {
    for (const Word flags : {0U, flag_c | flag_n, flag_z | flag_v, status_byte}) {
      condition_case(cases, input, flags | mode_rm | mode_down | bfu_model::sm);
    }
  }
}

void parallel_loads(Cases &cases, const std::vector<Input> &inputs) {
  for (const auto mode : modes) {
    for (const auto input : inputs) {
      auto state = seed(mode | flag_s | flag_c | flag_e);
      append(cases, state, input);
      auto &item = cases.back();
      item.before.front().words = {load_x0 | (bfu_model::opcode(input) & status_byte)};
      item.before.push_back({.words = {datum}, .address = state.r[0], .space = Space::x});
      // Rebuild from old state after the bus write to prove latched source use.
      item.expected = state;
      register_model::write(item.expected, 4, datum);
      ++item.expected.r[0];
      bfu_model::apply(item.expected, state, input);
      ++item.expected.pc;
    }
  }
}

void counts(Cases &cases) {
  constexpr Word high = 0xa5a5e0;
  for (const auto mode : modes) {
    for (const auto function : {Function::left, Function::right}) {
      for (unsigned count = 0; count <= word_bits; ++count) {
        for (const bool destination_b : {false, true}) {
          append(cases,
                 seed(mode | status_byte),
                 {.function = function,
                  .form = Form::shift_immediate,
                  .count = count,
                  .destination_b = destination_b});
          for (unsigned source = 0; source < bfu_model::controls.size(); ++source) {
            auto state = seed(mode | flag_c | flag_e | flag_u);
            register_model::write(state, bfu_model::controls[source], high | count);
            append(cases,
                   state,
                   {.function = function,
                    .form = Form::shift_register,
                    .source = source,
                    .destination_b = destination_b});
          }
        }
      }
    }
  }
}

void repeated(Cases &cases, const Input input, const Word status) {
  constexpr Word repeat = 0x0603a0;
  for (Word budget = 1; budget <= 4; ++budget) {
    BinaryCase item;
    item.initial = seed(status);
    if (input.form == Form::shift_register) {
      register_model::write(item.initial, bfu_model::controls[input.source], 1);
    }
    item.expected = item.initial;
    for (Word step = 1; step < budget; ++step) {
      bfu_model::apply(item.expected, item.expected, input);
    }
    const bool finished = budget == 4;
    item.expected.pc = start + (finished ? 2U : 1U);
    item.expected.lc = finished ? item.initial.lc : 4 - budget;
    item.before = {{.words = {repeat, bfu_model::opcode(input)}, .address = start}};
    item.budget = item.dispatches = budget;
    item.interruptible = finished;
    item.progress = {.instructions = budget, .estimated_cycles = 4 + budget};
    item.name = "REP BFU current count/Carry/word values, every exact partial prefix";
    cases.push_back(std::move(item));
  }
}

void repetitions(Cases &cases, const std::vector<Input> &inputs, const bool shifts) {
  for (const auto mode : modes) {
    for (const auto input : inputs) { repeated(cases, input, mode | status_byte); }
    if (!shifts) { continue; }
    for (const auto function : {Function::left, Function::right}) {
      for (unsigned source = 0; source < bfu_model::controls.size(); ++source) {
        for (const bool destination_b : {false, true}) {
          repeated(cases,
                   {.function = function,
                    .form = Form::shift_register,
                    .source = source,
                    .destination_b = destination_b},
                   mode | status_byte);
        }
      }
    }
  }
}

void loop_endings(Cases &cases, const std::vector<Input> &inputs) {
  constexpr Word lf = Word{1} << 15;
  constexpr Word fv = Word{1} << 16;
  constexpr Word saved_la = 0x70;
  constexpr Word pointer_range = 64;
  for (const auto input : inputs) {
    for (const Word count : {1U, 2U}) {
      for (const Word pointer : {0U, 15U, 31U, 63U}) {
        for (const Word saved : {0U, lf, fv, lf | fv}) {
          auto state = seed(lf | status_byte);
          state.la = start;
          state.lc = count;
          state.sp = pointer;
          state.sc = 2;
          state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
          state.stack[(pointer + pointer_range - 1) % stack_entries] =
              (std::uint64_t{saved_la} << word_bits) | 7U;
          append(cases, state, input);
          auto &expected = cases.back().expected;
          expected.pc = count == 1 ? start + 1 : start;
          if (count == 1) {
            expected.la = saved_la;
            expected.lc = 7;
            expected.sp = (pointer + pointer_range - 2) % pointer_range;
            expected.sc = 0;
            expected.sr = (expected.sr & ~(lf | fv)) | saved;
          } else {
            --expected.lc;
          }
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: generate_bfu_logic logic|shifts OUTPUT.d56");
    }
    const std::string_view kind = argv[1];
    if (kind != "logic" && kind != "shifts") {
      throw std::runtime_error("Unknown BFU fixture kind");
    }
    const bool shifts = kind == "shifts";
    const auto shapes = dsp56300::test::shapes(shifts);
    dsp56300::test::Cases cases;
    dsp56300::test::flags(cases, shapes);
    dsp56300::test::bits(cases, shapes);
    dsp56300::test::parallel_loads(cases, shapes);
    dsp56300::test::loop_endings(cases, shapes);
    if (shifts) {
      dsp56300::test::counts(cases);
      dsp56300::test::repetitions(cases, shapes, true);
      for (const auto input : shapes) {
        for (const dsp56300::Word flags : {0U, dsp56300::test::status_byte}) {
          dsp56300::test::condition_case(cases, input, flags | dsp56300::test::bfu_model::sm);
        }
      }
    } else {
      dsp56300::test::immediates(cases);
      dsp56300::test::repetitions(cases, shapes, false);
      dsp56300::test::truth_patterns(cases, shapes);
      dsp56300::test::conditionals(cases, shapes);
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent BFU " << kind << " SM cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
