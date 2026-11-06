// SPDX-License-Identifier: GPL-3.0-only
// Independent linear-address and bus model; no emulator implementation is linked.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
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
constexpr Word single_base = 0x400000, paired_base = 0x800000;
constexpr unsigned high_reg_shift = 17, xreg_shift = 18, ymode_shift = 20;
constexpr Word load_x = 0x8000, load_y = 0x400000, y_space = 0x080000;
constexpr Word effective_bit = 0x4000, linear_high = 0xabffff;
constexpr Word location = 0x180, xdatum = 0x923456, ydatum = 0x765432;
constexpr Word short_address = 31, ignored_modifier = 0xab4000, high_reg_mask = 0x18;
constexpr Word tfr_x0_a = 0x41, tfr_y0_b = 0x59;
constexpr std::array data_regs{4U, 5U, 6U, 7U, 14U, 15U};
constexpr std::array xregs{4U, 5U, 14U, 15U}, yregs{6U, 7U, 14U, 15U};
constexpr std::array modes{0U,
                           mode_down,
                           mode_up,
                           flag_s | flag_l,
                           mode_down | flag_s | flag_l,
                           mode_up | flag_s | flag_l};
constexpr std::array offsets{0U, 1U, word_mask, sign24 - 1, sign24, sign24 + 1, word_mask - 1, 7U};
constexpr std::array alu_codes{0U, tfr_x0_a, tfr_y0_b}; // No ALU, TFR X0,A, TFR Y0,B.

Word wrap(const std::int64_t value) {
  constexpr auto size = std::int64_t{1} << word_bits;
  return static_cast<Word>((value % size + size) % size);
}

struct Transfer {
  unsigned reg = 4, mode = 4, index = 0;
  Space space = Space::x;
  bool load = true, short_form = false, immediate = false;
};

Word transfer_address(BinaryCase &item, const Transfer input) {
  if (input.short_form) { return short_address; }
  if (input.mode == 6) { return location; }
  auto &cursor = item.expected.r.at(input.index);
  const auto original = cursor;
  const auto offset = signed24(item.expected.n.at(input.index));
  if (input.mode == 0) { cursor = wrap(cursor - offset); }
  if (input.mode == 1) { cursor = wrap(cursor + offset); }
  if (input.mode == 2 || input.mode == 7) { cursor = wrap(std::int64_t{cursor} - 1); }
  if (input.mode == 3) { cursor = wrap(std::int64_t{cursor} + 1); }
  if (input.mode == 5) { return wrap(cursor + offset); }
  return input.mode == 7 ? cursor : original;
}

void seed_address(BinaryCase &item, const Transfer input) {
  const auto stride = offsets.at(input.index);
  item.initial.n.at(input.index) = stride;
  item.initial.r.at(input.index) = input.mode == 5 ? wrap(location - signed24(stride)) : location;
  // No-update addressing ignores even reserved M values; updating ignores high bits.
  item.initial.m.at(input.index) = input.mode == 4 ? ignored_modifier : linear_high;
}

Word capture(BinaryCase &item, const Transfer input, const Word address) {
  const auto datum = input.space == Space::x ? xdatum : ydatum;
  if (!input.immediate) {
    item.before.push_back({.words = {datum}, .address = address, .space = input.space});
  }
  return input.load ? datum : register_model::read(item.expected, input.reg);
}

void commit(BinaryCase &item, const Transfer input, const Word address, const Word value) {
  if (input.load) {
    register_model::write(item.expected, input.reg, value);
  } else {
    item.after.push_back({.words = {value}, .address = address, .space = input.space});
  }
}

void append(Cases &cases, BinaryCase item, std::vector<Word> program, const Word alu) {
  // ALU inputs are latched before every move, including loads into those inputs.
  if (alu == tfr_x0_a) {
    register_model::write(item.expected, 14, static_cast<Word>(item.initial.x) & word_mask);
  } else if (alu == tfr_y0_b) {
    register_model::write(item.expected, 15, static_cast<Word>(item.initial.y) & word_mask);
  }
  item.expected.pc = start + static_cast<Word>(program.size());
  item.name = "parallel bus opcode " + std::to_string(program.at(0));
  item.before.push_back({.words = std::move(program), .address = start});
  cases.push_back(std::move(item));
}

struct SingleInput {
  Transfer transfer;
  Word mode = 0, alu = 0;
  std::uint64_t accumulator = register_model::seed_accumulator_a;
};

void single(Cases &cases, const SingleInput input) {
  const auto transfer = input.transfer;
  BinaryCase item;
  item.initial = register_model::seed(input.mode);
  item.initial.a = input.accumulator;
  item.initial.b = input.accumulator;
  seed_address(item, transfer);
  item.expected = item.initial;
  const auto address = transfer_address(item, transfer);
  const auto value = capture(item, transfer, address);
  commit(item, transfer, address, value);
  Word opcode = single_base | ((transfer.reg & 7U) << 16) |
                ((transfer.reg & high_reg_mask) << high_reg_shift) | input.alu;
  if (transfer.load) { opcode |= load_x; }
  if (transfer.space == Space::y) { opcode |= y_space; }
  if (transfer.short_form) {
    opcode |= short_address << 8;
    // FM A-6 lists distinct one-clock reads and two-clock short-address stores.
    if (!transfer.load) { ++item.progress.estimated_cycles; }
  } else {
    opcode |= effective_bit | (transfer.mode << 11) | (transfer.index << 8);
    if (transfer.mode >= 5) { ++item.progress.estimated_cycles; }
  }
  std::vector<Word> program{opcode};
  if (!transfer.short_form && transfer.mode == 6) {
    program.push_back(transfer.immediate ? value : address);
  }
  append(cases, std::move(item), std::move(program), input.alu);
}

void single_variants(Cases &cases, SingleInput input) {
  for (unsigned address_mode = 0; address_mode < 8; ++address_mode) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (address_mode == 6 && index != 0 && (!input.transfer.load || index != 4)) { continue; }
      input.transfer.mode = address_mode;
      input.transfer.index = index;
      input.transfer.immediate = address_mode == 6 && index == 4;
      input.alu = alu_codes.at(index % alu_codes.size());
      single(cases, input);
    }
  }
  input.transfer.short_form = true;
  input.transfer.immediate = false;
  input.alu = 0;
  single(cases, input);
}

void singles(Cases &cases) {
  for (const auto mode : modes) {
    for (const auto reg : data_regs) {
      for (const auto space : {Space::x, Space::y}) {
        for (const bool load : {false, true}) {
          single_variants(cases,
                          {.transfer = {.reg = reg, .space = space, .load = load}, .mode = mode});
        }
      }
    }
  }
}

struct PairInput {
  unsigned xreg = 0, yreg = 0, xmode = 0, ymode = 0, index = 0;
  Word mode = 0, alu = 0;
  bool xload = false, yload = false;
};

void paired(Cases &cases, const PairInput input) {
  const auto yindex = input.index < 4 ? input.index + 4 : input.index - 4;
  const Transfer x{.reg = xregs.at(input.xreg),
                   .mode = input.xmode == 0 ? 4 : input.xmode,
                   .index = input.index,
                   .space = Space::x,
                   .load = input.xload};
  const Transfer y{.reg = yregs.at(input.yreg),
                   .mode = input.ymode == 0 ? 4 : input.ymode,
                   .index = yindex,
                   .space = Space::y,
                   .load = input.yload};
  BinaryCase item;
  item.initial = register_model::seed(input.mode);
  seed_address(item, x);
  seed_address(item, y);
  item.expected = item.initial;
  const auto xa = transfer_address(item, x);
  const auto ya = transfer_address(item, y);
  const auto xv = capture(item, x, xa);
  const auto yv = capture(item, y, ya);
  commit(item, x, xa, xv);
  commit(item, y, ya, yv);
  const Word opcode = paired_base | (input.xreg << xreg_shift) | (input.yreg << 16) |
                      (input.xmode << 11) | (input.ymode << ymode_shift) | (input.index << 8) |
                      ((yindex % 4) << 13) | (input.xload ? load_x : 0) |
                      (input.yload ? load_y : 0) | input.alu;
  append(cases, std::move(item), {opcode}, input.alu);
}

void pairs(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned xreg = 0; xreg < 4; ++xreg) {
      for (unsigned yreg = 0; yreg < 4; ++yreg) {
        for (unsigned direction = 0; direction < 4; ++direction) {
          for (unsigned addressing = 0; addressing < 16; ++addressing) {
            const auto index =
                static_cast<unsigned>((xreg + yreg + direction + addressing) % agu_registers);
            paired(cases,
                   {.xreg = xreg,
                    .yreg = yreg,
                    .xmode = addressing % 4,
                    .ymode = addressing / 4,
                    .index = index,
                    .mode = mode,
                    .alu = alu_codes.at(addressing % alu_codes.size()),
                    .xload = (direction & 1U) != 0,
                    .yload = (direction & 2U) != 0});
          }
        }
      }
    }
  }
}

void accumulator_boundaries(Cases &cases) {
  constexpr std::array<std::uint64_t, 16> values{0,
                                                 1,
                                                 sign56,
                                                 sign56 - 1,
                                                 accumulator_mask,
                                                 0x7fffff000000,
                                                 0x800000000000,
                                                 0xff800000000000,
                                                 0xff7fffffffffff,
                                                 0x400000000000,
                                                 0x3fffffffffff,
                                                 0xffc00000000000,
                                                 0xffbfffffffffff,
                                                 0xffffff000000,
                                                 0x1000000000000,
                                                 0xfeffffffffffff};
  for (const auto mode : modes) {
    for (const auto value : values) {
      for (const auto reg : {14U, 15U}) {
        for (const auto space : {Space::x, Space::y}) {
          single(cases,
                 {.transfer = {.reg = reg, .space = space, .load = false},
                  .mode = mode,
                  .accumulator = value});
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: dsp56300_generate_parallel_moves OUTPUT.d56");
    }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::singles(cases);
    dsp56300::test::pairs(cases);
    dsp56300::test::accumulator_boundaries(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent parallel-move cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
