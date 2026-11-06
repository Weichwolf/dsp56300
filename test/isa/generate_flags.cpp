// SPDX-License-Identifier: GPL-3.0-only
// Flag-dependent sequences modeled independently, without executing a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000fa, multiply_b = 0x2000d8;
constexpr Word clear_a = 0x200013, clear_b = 0x20001b, round_b = 0x200019;
constexpr Word add_b_a = 0x200010, adc_x_a = 0x200021, adc_y_b = 0x200039;
constexpr Word sbc_x_b = 0x20002d, compare_b_a = 0x200005, subtract_b_a = 0x200014;
constexpr Word transfer_x0_b = 0x200049, transfer_a_b = 0x200009;
constexpr Word negate_a = 0x200036, absolute_a = 0x200026, add_immediate = 0x0140c0;
constexpr Word immediate = 0x800001;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array word_values{Word{0}, Word{1}, sign24, word_mask};
constexpr std::array accumulator_values{std::uint64_t{0}, sign56 - 1, sign56, accumulator_mask};

struct Step {
  Word opcode = 0;
  bool immediate = false;
};

using Sequence = std::vector<Step>;

std::array<Sequence, 7> sequences() {
  return {
      Sequence{{.opcode = add_b_a}, {.opcode = clear_a}},
      Sequence{
          {.opcode = add_b_a}, {.opcode = multiply_b}, {.opcode = adc_x_a}, {.opcode = clear_b}},
      Sequence{{.opcode = subtract_b_a},
               {.opcode = transfer_x0_b},
               {.opcode = 0},
               {.opcode = mac_a},
               {.opcode = sbc_x_b},
               {.opcode = round_b}},
      Sequence{{.opcode = negate_a}, {.opcode = absolute_a}, {.opcode = clear_a}},
      Sequence{{.opcode = compare_b_a},
               {.opcode = clear_b},
               {.opcode = mac_a},
               {.opcode = transfer_a_b}},
      Sequence{{.opcode = add_b_a},
               {.opcode = add_immediate, .immediate = true},
               {.opcode = mac_b},
               {.opcode = clear_a}},
      Sequence{{.opcode = mac_a},
               {.opcode = mac_b},
               {.opcode = add_b_a},
               {.opcode = adc_y_b},
               {.opcode = compare_b_a},
               {.opcode = transfer_a_b},
               {.opcode = 0}}};
}

struct Input {
  Word mode = 0;
  std::size_t seed = 0, prefix = 1;
};

BinaryCase generate(const Sequence &sequence, const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.mode);
  item.initial.a = accumulator_values.at(input.seed);
  item.initial.b = input.seed == 2 ? sign56 : 1;
  item.initial.x =
      (std::uint64_t{word_values.at((input.seed + 1) % word_values.size())} << word_bits) |
      word_values.at(input.seed);
  item.initial.y = (std::uint64_t{word_values.at(input.seed)} << word_bits) |
                   word_values.at((input.seed + 1) % word_values.size());
  item.initial.sr = input.mode | flag_s | (status_byte & ~(flag_c | flag_l));
  item.expected = item.initial;
  std::vector<Word> program;
  std::uint64_t cycles = 0;
  for (std::size_t index = 0; index < sequence.size(); ++index) {
    const auto step = sequence[index];
    program.push_back(step.opcode);
    if (step.immediate) { program.push_back(immediate); }
    if (index >= input.prefix) { continue; }
    auto old = item.expected;
    if (step.immediate) { old.x = immediate; }
    sequence_model::alu(item.expected, old, step.immediate ? 0x40 : step.opcode);
    item.expected.pc += step.immediate ? 2U : 1U;
    cycles += step.immediate ? 2U : 1U;
  }
  item.budget = item.retired = static_cast<Word>(input.prefix);
  item.progress = {.instructions = item.budget, .estimated_cycles = cycles};
  item.before = {{.words = std::move(program), .address = start}};
  item.name = "Independent overwritten flags seed=" + std::to_string(input.seed) +
              " prefix=" + std::to_string(input.prefix) + " mode=" + std::to_string(input.mode);
  return item;
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_flags OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    for (const auto &sequence : dsp56300::test::sequences()) {
      for (const auto mode : dsp56300::test::modes) {
        for (const auto carry : {dsp56300::Word{0}, dsp56300::test::flag_c}) {
          for (const auto limit : {dsp56300::Word{0}, dsp56300::test::flag_l}) {
            for (std::size_t seed = 0; seed < dsp56300::test::accumulator_values.size(); ++seed) {
              for (std::size_t prefix = 1; prefix <= sequence.size(); ++prefix) {
                cases.push_back(dsp56300::test::generate(
                    sequence, {.mode = mode | carry | limit, .seed = seed, .prefix = prefix}));
              }
            }
          }
        }
      }
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent flag sequences written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
