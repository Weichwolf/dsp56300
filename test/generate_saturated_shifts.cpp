// SPDX-License-Identifier: GPL-3.0-only
// Original SM shift fixtures: no DSP implementation is linked or executed.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 8> values{0,
                                              1,
                                              saturation_model::positive,
                                              saturation_model::positive + 1,
                                              saturation_model::negative,
                                              sign56 - 1,
                                              sign56,
                                              accumulator_mask};
constexpr std::array controls{12U, 13U, 4U, 6U, 5U, 7U}; // A1/B1/X0/Y0/X1/Y1.

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status | saturation_model::mode;
  return state;
}

void append(Cases &cases, const Registers &state, const shift_model::Control input, const Word op) {
  BinaryCase item;
  item.initial = state;
  item.expected = state;
  shift_model::apply(item.expected, state, input);
  ++item.expected.pc;
  item.before = {{.words = {op}, .address = start}};
  item.name = "SM bit-serial shift, exact flags, saturation and old control/operand aliases";
  cases.push_back(std::move(item));
}

Word register_opcode(const shift_model::Control input, const unsigned selector) {
  constexpr Word left_op = 0x0c1e40;
  constexpr Word right_op = 0x0c1e60;
  return (input.left ? left_op : right_op) | (selector * 2) | (input.source_b ? 16U : 0U) |
         (input.destination_b ? 1U : 0U);
}

void variants(Cases &cases,
              const Registers &state,
              shift_model::Control input,
              const unsigned selector = 0) {
  for (const bool source_b : {false, true}) {
    for (const bool destination_b : {false, true}) {
      input.source_b = source_b;
      input.destination_b = destination_b;
      const auto opcode =
          selector == 0 ? shift_model::immediate(input) : register_opcode(input, selector);
      append(cases, state, input, opcode);
    }
  }
}

void immediate(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned count = 0; count < accumulator_bits; ++count) {
      for (const auto value : values) {
        auto state = seed(mode | status_byte);
        state.a = value;
        state.b = accumulator_mask ^ value;
        for (const bool left : {false, true}) {
          variants(cases, state, {.count = count, .left = left});
        }
      }
    }
  }
}

void register_controls(Cases &cases) {
  constexpr Word high = 0xa5a580;
  for (const auto mode : modes) {
    for (unsigned selector = 2; selector < 8; ++selector) {
      for (unsigned count = 0; count < accumulator_bits; ++count) {
        auto state = seed(mode | flag_s | flag_v | flag_c);
        register_model::write(state, controls.at(selector - 2), high | count);
        for (const bool left : {false, true}) {
          variants(cases, state, {.count = count, .left = left}, selector);
        }
      }
    }
  }
}

void decisions(Cases &cases) {
  constexpr std::array<unsigned, 4> counts{0, 1, 24, 55};
  constexpr unsigned extension_shift = 48;
  constexpr unsigned msp_sign = 47;
  constexpr std::uint64_t low = 0x123456abcdef;
  for (const auto mode : modes) {
    for (Word extension = 0; extension <= status_byte; ++extension) {
      for (const bool high : {false, true}) {
        for (const bool left : {false, true}) {
          auto state = seed(mode | (extension ^ status_byte));
          state.a = (std::uint64_t{extension} << extension_shift) |
                    (high ? std::uint64_t{1} << msp_sign : 0) | low;
          const shift_model::Control input{
              .count = counts.at(extension % counts.size()), .left = left, .destination_b = high};
          append(cases, state, input, shift_model::immediate(input));
        }
      }
    }
  }
}

void flags(Cases &cases, const Word mode) {
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (const bool left : {false, true}) {
      for (const bool destination_b : {false, true}) {
        const auto state = seed(mode | flags);
        const shift_model::Control input{
            .left = left, .source_b = destination_b, .destination_b = destination_b};
        append(cases, state, input, shift_model::parallel(input));
      }
    }
  }
}

void bits(Cases &cases, const Word mode) {
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    for (const bool inverse : {false, true}) {
      for (const bool left : {false, true}) {
        auto state = seed(mode | flag_c | flag_l);
        const auto raw = (inverse ? accumulator_mask : 0) ^ (std::uint64_t{1} << bit);
        state.a = raw;
        state.b = raw;
        for (const bool destination_b : {false, true}) {
          const shift_model::Control input{
              .left = left, .source_b = destination_b, .destination_b = destination_b};
          append(cases, state, input, shift_model::parallel(input));
        }
      }
    }
  }
}

void repetitions(Cases &cases, const Word mode) {
  constexpr Word repeat_three = 0x0603a0;
  constexpr std::uint64_t main_one = std::uint64_t{1} << word_bits;
  for (const unsigned selector : {2U, 4U}) {
    for (const bool left : {false, true}) {
      BinaryCase item;
      item.initial = seed(mode | flag_s | flag_l | flag_c);
      item.initial.a = selector == 2 ? main_one : sign56 - 1;
      register_model::write(item.initial, 4, 1);
      item.expected = item.initial;
      for (unsigned i = 0; i < 3; ++i) {
        const auto count = register_model::read(item.expected, controls.at(selector - 2)) % 64;
        shift_model::apply(item.expected, item.expected, {.count = count, .left = left});
      }
      item.expected.pc = start + 2;
      item.before = {
          {.words = {repeat_three, register_opcode({.left = left}, selector)}, .address = start}};
      item.budget = 4;
      item.dispatches = 4;
      item.progress = {.instructions = 4, .estimated_cycles = 8};
      item.name = "SM REP reevaluates old count aliases and saturates after every shift";
      cases.push_back(std::move(item));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_saturated_shifts OUTPUT"); }
    dsp56300::test::Cases cases;
    dsp56300::test::immediate(cases);
    dsp56300::test::register_controls(cases);
    dsp56300::test::decisions(cases);
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::flags(cases, mode);
      dsp56300::test::bits(cases, mode);
      dsp56300::test::repetitions(cases, mode);
    }
    std::cout << cases.size() << " independent saturated-shift cases\n";
    dsp56300::test::write_fixture(argv[1], std::move(cases));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
