// SPDX-License-Identifier: GPL-3.0-only
// Directed register-dependency sequences, modeled without executing a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000fa;
constexpr Word add_b_a = 0x200010, add_a_b = 0x200018;
constexpr Word xdatum = 0x923456, ydatum = 0x765432, address = 0x180;
constexpr unsigned register_high_shift = 17, x_register_shift = 18;
constexpr std::array data_registers{4U, 5U, 6U, 7U, 14U, 15U};
constexpr std::array x_registers{4U, 5U, 14U, 15U}, y_registers{6U, 7U, 14U, 15U};

struct Transfer {
  unsigned reg = 4;
  Space space = Space::x;
  bool load = false;
};

Word capture(BinaryCase &item, const Transfer input) {
  if (input.load) { return input.space == Space::x ? xdatum : ydatum; }
  return register_model::read(item.expected, input.reg);
}

void commit(BinaryCase &item, const Transfer input, const Word value) {
  if (input.load) {
    register_model::write(item.expected, input.reg, value);
  } else {
    item.after.push_back({.words = {value}, .address = address, .space = input.space});
  }
}

struct Input {
  std::array<Transfer, 2> transfers;
  Word count = 1, prefix = 0, alu = 0, status = 0;
};

BinaryCase sequence(const Input input) {
  BinaryCase item;
  item.name = "Independent native dataflow opcode " + std::to_string(input.prefix | input.alu);
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status | (status_byte & ~(flag_c | flag_s | flag_l));
  item.initial.r.fill(address);
  item.expected = item.initial;
  // Seed both host register candidates, then test a conflicting transfer and
  // consume both architectural results across subsequent native instructions.
  for (const auto prime : {mac_a, mac_b}) {
    const auto old = item.expected;
    sequence_model::alu(item.expected, old, prime);
  }
  const auto old = item.expected;
  std::array<Word, 2> values{};
  for (Word index = 0; index < input.count; ++index) {
    values.at(index) = capture(item, input.transfers.at(index));
  }
  for (Word index = 0; index < input.count; ++index) {
    commit(item, input.transfers.at(index), values.at(index));
  }
  sequence_model::alu(item.expected, old, input.alu);
  for (const auto follow : {add_b_a, add_a_b}) {
    const auto preceding = item.expected;
    sequence_model::alu(item.expected, preceding, follow);
  }
  const std::vector program{mac_a, mac_b, input.prefix | input.alu, add_b_a, add_a_b};
  item.budget = static_cast<Word>(program.size());
  item.dispatches = item.budget;
  item.progress = {.instructions = item.budget, .estimated_cycles = item.budget};
  item.expected.pc = start + item.budget;
  item.before = {{.words = program, .address = start},
                 {.words = {xdatum}, .address = address, .space = Space::x},
                 {.words = {ydatum}, .address = address, .space = Space::y}};
  return item;
}

std::vector<Word> alu_forms() {
  std::vector<Word> result;
  for (Word opcode = 0x80; opcode <= status_byte; ++opcode) { result.push_back(opcode); }
  constexpr std::array<Word, 23> additions{0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x14,
                                           0x24, 0x34, 0x44, 0x54, 0x64, 0x74, 0x05, 0x45,
                                           0x55, 0x65, 0x75, 0x21, 0x31, 0x25, 0x35};
  constexpr std::array other{sequence_model::clear,
                             sequence_model::round,
                             sequence_model::absolute,
                             sequence_model::negate,
                             sequence_model::tfr_other,
                             sequence_model::tfr_x0,
                             sequence_model::tfr_y0,
                             sequence_model::tfr_x1,
                             sequence_model::tfr_y1};
  for (const Word destination : {Word{0}, Word{8}}) {
    for (const auto opcode : additions) { result.push_back(opcode | destination); }
    for (const auto opcode : other) { result.push_back(opcode | destination); }
  }
  return result;
}

struct SingleInput {
  unsigned reg = 4;
  Word status = 0, alu = 0;
  Space space = Space::x;
  bool load = false;
};

BinaryCase single(const SingleInput input) {
  const Word prefix = 0x406000 | ((input.reg & 7U) << 16) |
                      ((input.reg & 0x18U) << register_high_shift) |
                      (input.space == Space::y ? 0x80000U : 0U) | (input.load ? 0x8000U : 0U);
  return sequence(
      {.transfers = {{{.reg = input.reg, .space = input.space, .load = input.load}, {}}},
       .prefix = prefix,
       .alu = input.alu,
       .status = input.status});
}

struct PairInput {
  unsigned xreg = 0, yreg = 0;
  Word directions = 0, status = 0, alu = 0;
};

BinaryCase paired(const PairInput input) {
  const bool xload = (input.directions & 1U) != 0;
  const bool yload = (input.directions & 2U) != 0;
  const Word prefix = 0x800000 | (input.xreg << x_register_shift) | (input.yreg << 16) |
                      (xload ? 0x8000U : 0U) | (yload ? 0x400000U : 0U);
  return sequence(
      {.transfers = {{{.reg = x_registers.at(input.xreg), .space = Space::x, .load = xload},
                      {.reg = y_registers.at(input.yreg), .space = Space::y, .load = yload}}},
       .count = 2,
       .prefix = prefix,
       .alu = input.alu,
       .status = input.status});
}

void matrix(std::vector<BinaryCase> &cases, const Word mode, const Word alu) {
  for (const auto reg : data_registers) {
    for (const bool load : {false, true}) {
      for (const auto space : {Space::x, Space::y}) {
        const auto flags =
            ((reg + (load ? 1U : 0U)) & 1U) != 0 ? mode_rm | flag_c : flag_s | flag_l;
        cases.push_back(
            single({.reg = reg, .status = mode | flags, .alu = alu, .space = space, .load = load}));
      }
    }
  }
  for (unsigned xreg = 0; xreg < 4; ++xreg) {
    for (unsigned yreg = 0; yreg < 4; ++yreg) {
      for (Word directions = 0; directions < 4; ++directions) {
        const auto flags =
            ((xreg + yreg + directions) & 1U) != 0 ? mode_rm | flag_c : flag_s | flag_l;
        cases.push_back(paired({.xreg = xreg,
                                .yreg = yreg,
                                .directions = directions,
                                .status = mode | flags,
                                .alu = alu}));
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_dataflow OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    for (const auto mode :
         {dsp56300::Word{0}, dsp56300::test::mode_down, dsp56300::test::mode_up}) {
      for (const auto alu : dsp56300::test::alu_forms()) {
        dsp56300::test::matrix(cases, mode, alu);
      }
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent native dataflow sequences written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
