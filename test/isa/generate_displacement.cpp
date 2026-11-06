// SPDX-License-Identifier: GPL-3.0-only
// Independent serial AGU coordinates and raw-register / quotient-remainder bus model.
#include "address_model.hpp"
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
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
constexpr Word saturation_mode = Word{1} << 20;
constexpr Word location = 384, datum = 0x923456, repeat_base = 0x0600a0;
constexpr Word short_base = 0x020080, long_base = 0x0a7080, ignored_n = 0xa54321;
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, saved_la = 0x70;
constexpr Word ignored_high = 0xab0000, pointer_range = 64;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, tfr_x_a = 0x200041, tfr_y_b = 0x200059;
constexpr Word adc_x_a = 0x200021, negate_b = 0x20003e;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array offsets{
    0U, 1U, 63U, 64U, sign24 - 1, sign24, sign24 + 1, word_mask - 63, word_mask - 1, word_mask};
constexpr std::array<std::uint64_t, 12> values{0,
                                               1,
                                               sign56 - 1,
                                               sign56,
                                               accumulator_mask,
                                               0x7fffff000000,
                                               0x800000000000,
                                               0xff800000000000,
                                               0xff7fffffffffff,
                                               0x123456abcdef,
                                               0xff123456abcdef,
                                               0x400000000000};

struct Input {
  unsigned index = 0, reg = 4;
  Word displacement = 0, cursor = location, modifier = 0xabffff, status = 0, data = datum;
  std::uint64_t accumulator = register_model::seed_accumulator_a;
  bool long_form = false, y = false, load = true;
};

Word opcode(const Input input) {
  Word result = (input.long_form ? long_base : short_base) | (input.index << 8) | input.reg;
  if (input.y) { result |= Word{1} << (input.long_form ? 16U : 5U); }
  if (input.load) { result |= Word{1} << (input.long_form ? 6U : 4U); }
  if (!input.long_form) {
    // Place displacement bits individually, independently of the decoder extraction.
    for (unsigned bit = 0; bit < 7; ++bit) {
      if (((input.displacement >> bit) & 1U) != 0) {
        result |= Word{1} << (bit == 0 ? 6U : bit + 10);
      }
    }
  }
  return result;
}

Word address(const Registers &state, const Input input) {
  return address_model::updated({.modifier = state.m.at(input.index),
                                 .cursor = state.r.at(input.index),
                                 .offset = input.displacement,
                                 .mode = 1});
}

void transfer(BinaryCase &item, const Input input) {
  const auto effective = address(item.expected, input);
  const auto space = input.y ? Space::y : Space::x;
  if (input.load) {
    register_model::write(item.expected, input.reg, input.data);
  } else {
    const auto value = register_model::read(item.expected, input.reg);
    item.after.push_back({.words = {value}, .address = effective, .space = space});
  }
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  item.initial.a = item.initial.b = input.accumulator;
  item.initial.r.at(input.index) = input.cursor;
  item.initial.n.at(input.index) = ignored_n;
  item.initial.m.at(input.index) = input.modifier;
  item.expected = item.initial;
  const auto effective = address(item.initial, input);
  require(effective < memory_words, "Synthetic displacement case stays in RAM");
  std::vector<Word> program{opcode(input)};
  if (input.long_form) { program.push_back(input.displacement); }
  item.before = {
      {.words = std::move(program), .address = start},
      {.words = {input.data}, .address = effective, .space = input.y ? Space::y : Space::x}};
  transfer(item, input);
  item.expected.pc += input.long_form ? 2U : 1U;
  item.progress.estimated_cycles = input.long_form ? 3U : 2U;
  item.name = "Displaced MOVE; word count=" + std::to_string(input.long_form ? 2 : 1) +
              "; reg=" + std::to_string(input.reg) + "; index=" + std::to_string(input.index) +
              "; raw offset=" + std::to_string(input.displacement);
  return item;
}

void directions(Cases &cases, Input input) {
  for (const bool y : {false, true}) {
    for (const bool load : {false, true}) {
      input.y = y;
      input.load = load;
      cases.push_back(single(input));
    }
  }
}

void short_encodings(Cases &cases) {
  for (Word raw = 0; raw < 128; ++raw) {
    const Word expanded = raw < 64 ? raw : word_mask - (127 - raw);
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (unsigned reg = 4; reg < 16; ++reg) {
        const auto variant = raw + index + reg;
        directions(
            cases,
            {.index = index,
             .reg = reg,
             .displacement = expanded,
             .status = modes.at(variant % modes.size()) | (variant % 2 == 0 ? status_byte : 0U),
             .accumulator = values.at(variant % values.size())});
      }
    }
  }
}

void long_encodings(Cases &cases) {
  for (unsigned reg = 0; reg < 64; ++reg) {
    if (!register_model::valid(reg)) { continue; }
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (const Word offset : offsets) {
        const auto variant = reg + index + offset;
        directions(cases,
                   {.index = index,
                    .reg = reg,
                    .displacement = offset,
                    .cursor = address_model::wrap24(location - signed24(offset)),
                    .status = modes.at(variant % modes.size()) | status_byte,
                    .accumulator = values.at(variant % values.size()),
                    .long_form = true});
      }
    }
  }
}

void modifiers(Cases &cases, const bool long_form) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (const Word offset : {0U, 1U, 3U, word_mask - 2, word_mask}) {
      for (const Word modifier : {0U, 3U, 6U, 15U, 0x8001U, 0x800fU, 0xbfffU}) {
        const Word cursor = modifier == 0
                                ? address_model::reverse_carry(
                                      {.original = location, .operand = offset, .subtract = true})
                                : location;
        directions(cases,
                   {.index = index,
                    .reg = index % 2 == 0 ? 14U : 15U,
                    .displacement = offset,
                    .cursor = cursor,
                    .modifier = modifier | ignored_high,
                    .status = status_byte,
                    .accumulator = values.at(index),
                    .long_form = long_form});
      }
    }
  }
}

void precision(Cases &cases, const bool long_form) {
  for (const auto mode : modes) {
    for (const auto value : values) {
      for (unsigned reg = 4; reg < 16; ++reg) {
        directions(cases,
                   {.reg = reg,
                    .displacement = word_mask,
                    .status = mode | status_byte,
                    .accumulator = value,
                    .long_form = long_form});
      }
    }
  }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    directions(cases,
               {.reg = flags % 2 == 0 ? 14U : 15U,
                .status = flags,
                .accumulator = values.at(flags % values.size()),
                .long_form = long_form});
  }
}

void repeated(Cases &cases, const Input input) {
  constexpr Word count = 4;
  for (const Word budget : {1U, 2U, 3U, count + 1}) {
    auto item = single(input);
    item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
    item.expected = item.initial;
    item.after.clear();
    for (Word step = 1; step < budget; ++step) { transfer(item, input); }
    const bool completed = budget == count + 1;
    item.expected.pc = start + (completed ? 2U : 1U);
    item.expected.lc = completed ? item.initial.lc : count + 1 - budget;
    item.interruptible = completed;
    item.budget = item.dispatches = budget;
    item.progress = {.instructions = budget, .estimated_cycles = 5 + (2 * (budget - 1))};
    item.name = "REP short displaced MOVE preserves address source and uses current data";
    cases.push_back(std::move(item));
  }
}

void ending(Cases &cases, const Input input) {
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word saved : {0U, lf, fv, lf | fv}) {
        auto item = single(input);
        auto &state = item.initial;
        const Word end = start + (input.long_form ? 1U : 0U);
        state.sr |= lf;
        state.la = end;
        state.lc = count;
        state.sp = pointer;
        state.sc = 2;
        state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
        state.stack[(pointer + pointer_range - 1) % stack_entries] =
            (std::uint64_t{saved_la} << word_bits) | 7U;
        item.expected = state;
        item.after.clear();
        transfer(item, input);
        item.expected.pc = count == 1 ? end + 1 : start;
        if (count == 1) {
          item.expected.la = saved_la;
          item.expected.lc = 7;
          item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
          item.expected.sc = 0;
          item.expected.sr = (item.expected.sr & ~(lf | fv)) | saved;
        } else {
          --item.expected.lc;
        }
        item.name = "Displaced MOVE DO ending at opcode/extension restores saved flags/frames";
        cases.push_back(std::move(item));
      }
    }
  }
}

void sequences(Cases &cases, const Input input) {
  const auto move = opcode(input);
  std::vector<Word> program{mac_a, mac_b, move};
  if (input.long_form) { program.push_back(input.displacement); }
  program.insert(program.end(), {tfr_x_a, tfr_y_b, adc_x_a, negate_b});
  auto item = single(input);
  item.before.front().words = program;
  item.expected = item.initial;
  item.after.clear();
  item.budget = 0;
  item.progress = {};
  item.name = "Native/displaced MOVE/native register consumers and Carry: every prefix";
  for (std::size_t index = 0; index < program.size(); ++index) {
    const bool displaced = index == 2;
    if (displaced) {
      transfer(item, input);
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, program[index]);
    }
    const Word words = displaced && input.long_form ? 2U : 1U;
    item.expected.pc += words;
    const Word move_cycles = input.long_form ? 3U : 2U;
    item.progress.estimated_cycles += displaced ? move_cycles : 1U;
    ++item.budget;
    item.dispatches = item.budget;
    item.progress.instructions = item.budget;
    cases.push_back(item);
    if (words == 2) { ++index; }
  }
}

void interactions(Cases &cases, const bool long_form) {
  for (const bool y : {false, true}) {
    for (const bool load : {false, true}) {
      for (const unsigned reg : {4U, 14U, 15U}) {
        const Input input{.reg = reg,
                          .displacement = word_mask,
                          .status = status_byte,
                          .accumulator = accumulator_mask,
                          .long_form = long_form,
                          .y = y,
                          .load = load};
        if (!long_form) { repeated(cases, input); }
        ending(cases, input);
        for (const auto mode : modes) {
          auto sequence = input;
          sequence.status |= mode;
          sequences(cases, sequence);
        }
      }
    }
  }
}

void reject(Cases &cases, BinaryCase item, const FaultKind kind, const Word value) {
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {.kind = kind, .address = start, .value = value, .pc = start};
  item.name = "Displaced MOVE explicit invalid/unsupported/fetch boundary";
  cases.push_back(std::move(item));
}

void faults(Cases &cases, const bool long_form) {
  const unsigned limit = long_form ? 64 : 16;
  for (unsigned reg = 0; reg < limit; ++reg) {
    if (register_model::valid(reg)) { continue; }
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) {
        auto item = single({.long_form = long_form, .y = y, .load = load});
        const Word invalid = opcode({.reg = reg, .long_form = long_form, .y = y, .load = load});
        item.before.front().words.front() = invalid;
        reject(cases, std::move(item), FaultKind::invalid_instruction, invalid);
      }
    }
  }
  for (const Word mode :
       {Word{1} << 13, Word{1} << 14, Word{1} << 17, Word{1} << 20, mode_down | mode_up}) {
    auto item = single({.long_form = long_form});
    item.initial.sr = mode;
    if (mode == saturation_mode) {
      item.expected.sr = mode;
      item.name = "SM displaced MOVE preserves its formerly rejected input";
      cases.push_back(std::move(item));
    } else {
      reject(cases, std::move(item), FaultKind::unsupported_mode, mode);
    }
  }
  for (const Word modifier : {0x8000U, 0x8002U, 0xc001U, 0xfffeU}) {
    auto item = single({.long_form = long_form});
    item.initial.m[0] = modifier | ignored_high;
    reject(cases, std::move(item), FaultKind::unsupported_mode, modifier);
  }
  if (!long_form) { return; }
  auto missing = single({.long_form = true});
  missing.before.front().words.pop_back();
  missing.memory_words[0] = start + 1;
  reject(cases, std::move(missing), FaultKind::memory_read, 0);
  cases.back().fault.address = start + 1;
  auto multiword = single({.long_form = true});
  const auto invalid = multiword.before.front().words.front();
  multiword.before.front().words.insert(multiword.before.front().words.begin(),
                                        repeat_base | (2U << 8));
  multiword.budget = 3;
  multiword.dispatches = 1;
  multiword.progress = {.instructions = 1, .estimated_cycles = 5};
  multiword.interruptible = false;
  multiword.fault = {.kind = FaultKind::invalid_instruction,
                     .address = start + 1,
                     .value = invalid,
                     .pc = start + 1};
  cases.push_back(std::move(multiword));
}

void wrapping_extension(Cases &cases) {
  auto item = single({.displacement = word_mask, .long_form = true});
  const auto words = item.before.front().words;
  item.before.front() = {.words = {words.front()}, .address = word_mask};
  item.before.push_back({.words = {words.back()}, .address = 0});
  item.memory_words[0] = static_cast<Word>(address_words);
  item.initial.pc = word_mask;
  item.expected.pc = 1;
  item.name = "Long displaced MOVE extension wraps P:$FFFFFF to P:0";
  cases.push_back(std::move(item));
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3 ||
        (std::string_view(argv[1]) != "short" && std::string_view(argv[1]) != "long")) {
      throw std::runtime_error("Usage: dsp56300_generate_displacement short|long OUTPUT.d56");
    }
    const bool long_form = std::string_view(argv[1]) == "long";
    dsp56300::test::Cases cases;
    if (long_form) {
      dsp56300::test::long_encodings(cases);
    } else {
      dsp56300::test::short_encodings(cases);
    }
    dsp56300::test::modifiers(cases, long_form);
    dsp56300::test::precision(cases, long_form);
    dsp56300::test::interactions(cases, long_form);
    dsp56300::test::faults(cases, long_form);
    if (long_form) { dsp56300::test::wrapping_extension(cases); }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent displaced MOVE encoding/AGU/register/sequence cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
