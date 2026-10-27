// SPDX-License-Identifier: GPL-3.0-only
// Independent ring coordinates and bit-serial carry model; no emulator execution.
#include "address_model.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
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
constexpr Word linear = 0xffff, update_opcode = 0x204000, datum = 0xfedcba;
constexpr Word load_x0 = 0x44c000, load_y0 = 0x4ec000, immediate_move = 0x40f400;
constexpr Word immediate_modifier = 0x05f420, register_high_mask = 0x18;
constexpr Word ignored_high = 0xab0000, fault_high = 0x120000, short_stride = 63;
constexpr unsigned register_high_shift = 17, offset_register_base = 24;
constexpr std::array replacements{15U, 16U, linear, 0U, 0x800fU, 24575U, 1U, 15U, 0xab000fU};
constexpr std::array normal{
    1U, 2U, 3U, 4U, 7U, 15U, 16U, 31U, 255U, 256U, 4095U, 24575U, 32766U, 32767U};
constexpr std::array multiple{0x8001U, 0x8003U, 0x800fU, 0x80ffU, 0x9fffU, 0xbfffU};
constexpr std::array modes{0U, 1U, 2U, 3U, 4U, 5U, 7U};

using address_model::Input;
using address_model::span;
using address_model::updated;
using address_model::wrap24;

void append(Cases &cases, const Input input, const bool memory = false, const bool y = false) {
  BinaryCase item;
  const auto *const location = y ? " Y" : " X";
  item.name = "AGU M=" + std::to_string(input.modifier) + " R=" + std::to_string(input.cursor) +
              " N=" + std::to_string(input.offset) + " mode=" + std::to_string(input.mode) +
              " index=" + std::to_string(input.index) + (memory ? location : " update");
  item.initial = register_model::seed(0);
  item.initial.r.at(input.index) = input.cursor;
  item.initial.n.at(input.index) = input.offset;
  item.initial.m.at(input.index) = input.modifier;
  item.expected = item.initial;
  const auto result = updated(input);
  if (input.mode != 4 && input.mode != 5) { item.expected.r.at(input.index) = result; }
  Word opcode = update_opcode;
  if (memory) {
    opcode = y ? load_y0 : load_x0;
    const auto address = input.mode == 5 || input.mode == 7 ? result : input.cursor;
    require(address < memory_words, "Memory qualification stays within supplied RAM");
    item.before.push_back({.words = {datum}, .address = address, .space = y ? Space::y : Space::x});
    auto &target = y ? item.expected.y : item.expected.x;
    target = (target & ~std::uint64_t{word_mask}) | datum;
    if (input.mode == 5 || input.mode == 7) { ++item.progress.estimated_cycles; }
  }
  opcode |= (input.mode << 11) | (input.index << 8);
  item.before.push_back({.words = {opcode}, .address = start});
  item.expected.pc = start + 1;
  cases.push_back(std::move(item));
}

std::vector<Word> offsets(const Word modifier) {
  if (modifier >= 0x8000) {
    return {0, 1, 2, short_stride, 32768, sign24 - 1, sign24, word_mask - 1, word_mask};
  }
  if (modifier == 0) { return {0, 1, 2, 3, 16, 0x400000, sign24}; }
  const Word length = modifier + 1;
  const Word alignment = span(length);
  return {0,
          1,
          length - 1,
          length,
          wrap24(-static_cast<std::int64_t>(length)),
          word_mask,
          2 * alignment,
          wrap24(-2 * static_cast<std::int64_t>(alignment))};
}

void register_updates(Cases &cases, const Input input) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (const auto offset : mode < 2 ? offsets(input.modifier) : std::vector<Word>{sign24}) {
        append(cases,
               {.modifier = input.modifier | (index % 2 == 0 ? 0U : ignored_high),
                .cursor = input.cursor,
                .offset = offset,
                .mode = mode,
                .index = index});
      }
    }
  }
}

void update_cases(Cases &cases, const Word modifier) {
  const Word length = modifier == 0 || modifier == linear ? 128 : (modifier & 0x7fff) + 1;
  const Word alignment = span(length);
  for (const Word base : {Word{0}, Word{0x40000}, Word{0x1000000} - alignment}) {
    for (const Word position : {Word{0}, length / 2, length - 1}) {
      register_updates(cases, {.modifier = modifier, .cursor = base + position});
    }
  }
}

void memory_cases(Cases &cases) {
  for (const Word modifier : {0U, 1U, 2U, 15U, 16U, 127U, 0x8001U, 0x800fU, linear}) {
    const Word length = modifier == 0 || modifier == linear ? 128 : (modifier & 0x7fff) + 1;
    for (const Word position : {Word{0}, length - 1}) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        for (const auto mode : modes) {
          for (const bool y : {false, true}) {
            append(cases,
                   {.modifier = modifier,
                    .cursor = 128 + position,
                    .offset = 1,
                    .mode = mode,
                    .index = index},
                   true,
                   y);
          }
        }
      }
    }
  }
}

void fault_cases(Cases &cases) {
  for (const Word modifier : {0x8000U, 0x8002U, 0x8004U, 0xc001U, 0xc003U, 0xfffeU}) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (unsigned mode = 0; mode < 4; ++mode) {
        BinaryCase item;
        item.name = "Reserved AGU modifier " + std::to_string(modifier);
        item.initial = register_model::seed(0);
        item.initial.m[index] = modifier | fault_high;
        item.expected = item.initial;
        item.dispatches = 0;
        item.progress = {};
        item.interruptible = false;
        item.fault = {
            .kind = FaultKind::unsupported_mode, .address = start, .value = modifier, .pc = start};
        item.before.push_back(
            {.words = {update_opcode | (mode << 11) | (index << 8)}, .address = start});
        cases.push_back(std::move(item));
      }
    }
  }
}

Word immediate_register(const unsigned reg) {
  return immediate_move | ((reg & 7U) << 16) | ((reg & register_high_mask) << register_high_shift);
}

void modifier_sequences(Cases &cases) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    BinaryCase item;
    item.name = "Guest AGU modifier replacement index " + std::to_string(index);
    item.initial = register_model::seed(0);
    item.expected = item.initial;
    std::vector<Word> program;
    for (const Word modifier : replacements) {
      const Word low = modifier & 0xffff;
      const Word cursor = 0x40000 + (low == linear || low == 0 ? 15 : (low & 0x7fff));
      program.insert(program.end(),
                     {immediate_modifier | index,
                      modifier,
                      immediate_register(16 + index),
                      cursor,
                      immediate_register(offset_register_base + index),
                      1,
                      update_opcode | 0x800 | (index << 8)});
      item.expected.m[index] = modifier;
      item.expected.n[index] = 1;
      item.expected.r[index] =
          updated({.modifier = modifier, .cursor = cursor, .offset = 1, .mode = 1, .index = index});
    }
    item.budget = item.dispatches = static_cast<Word>(4 * replacements.size());
    item.progress = {.instructions = item.budget, .estimated_cycles = program.size()};
    item.expected.pc = start + static_cast<Word>(program.size());
    item.before.push_back({.words = std::move(program), .address = start});
    cases.push_back(std::move(item));
  }
}

void repeat_sequences(Cases &cases) {
  constexpr Word repeat_count = 33;
  constexpr Word repeat_opcode = 0x0621a0;
  for (const Word modifier : {0U, 1U, 2U, 15U, 16U, 24575U, 0x800fU, linear}) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      BinaryCase item;
      item.name = "Repeated AGU ring updates " + std::to_string(modifier);
      item.initial = register_model::seed(0);
      item.initial.r[index] = 0x40000;
      item.initial.n[index] = 1;
      item.initial.m[index] = modifier;
      item.expected = item.initial;
      for (Word iteration = 0; iteration < repeat_count; ++iteration) {
        item.expected.r[index] = updated({.modifier = modifier,
                                          .cursor = item.expected.r[index],
                                          .offset = 1,
                                          .mode = 1,
                                          .index = index});
      }
      item.budget = item.dispatches = repeat_count + 1;
      item.progress = {.instructions = repeat_count + 1, .estimated_cycles = repeat_count + 5};
      item.expected.pc = start + 2;
      item.before.push_back(
          {.words = {repeat_opcode, update_opcode | 0x800 | (index << 8)}, .address = start});
      cases.push_back(std::move(item));
    }
  }
}

void generate(const char *const output) {
  Cases cases;
  update_cases(cases, linear);
  update_cases(cases, 0);
  for (const auto modifier : normal) { update_cases(cases, modifier); }
  for (const auto modifier : multiple) { update_cases(cases, modifier); }
  memory_cases(cases);
  fault_cases(cases);
  modifier_sequences(cases);
  repeat_sequences(cases);
  std::cout << cases.size() << " independently modeled AGU cases\n";
  write_fixture(output, std::move(cases));
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_addressing OUTPUT.d56"); }
    dsp56300::test::generate(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
