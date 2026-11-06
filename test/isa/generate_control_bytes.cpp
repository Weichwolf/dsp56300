// SPDX-License-Identifier: GPL-3.0-only
// Independent byte-logic and sequencing expectations; no core execution/link.
#include "binary_fixture.hpp"
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
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
using Cases = std::vector<BinaryCase>;
using byte_model::Field;
constexpr std::array fields{Field::mr, Field::ccr, Field::com, Field::eom};
constexpr std::array bytes{0U,  255U, 85U,  170U, 1U,   2U,   4U,   8U,   16U,  32U,
                           64U, 128U, 254U, 253U, 251U, 247U, 239U, 223U, 191U, 127U};
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, round_a = 0x200011;
constexpr Word repeat_base = 0x0600a0;
constexpr Word repeat_old_byte = 85;
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, pointer_range = 64, saved_la = 0x70;

BinaryCase single(const byte_model::Input input, const Word old) {
  BinaryCase item;
  item.initial = byte_model::seed(input.field, old);
  item.expected = item.initial;
  byte_model::apply(item.expected, input);
  ++item.expected.pc;
  item.progress.estimated_cycles = 3;
  item.before = {{.words = {byte_model::opcode(input)}, .address = start}};
  item.name = "Control byte logic; field=" + std::to_string(static_cast<unsigned>(input.field)) +
              "; immediate=" + std::to_string(input.immediate) + "; old=" + std::to_string(old);
  return item;
}

void matrix(Cases &cases) {
  for (const auto field : fields) {
    for (const bool inclusive_or : {false, true}) {
      for (Word immediate = 0; immediate <= status_byte; ++immediate) {
        for (const Word old : bytes) {
          cases.push_back(
              single({.field = field, .immediate = immediate, .inclusive_or = inclusive_or}, old));
        }
      }
    }
  }
}

void sequence(Cases &cases, const byte_model::Input input) {
  auto item = single(input, 0);
  item.initial.sr = (input.inclusive_or ? 0U : flag_c) | flag_s | flag_l;
  item.expected = item.initial;
  const std::vector program{mac_a, mac_b, byte_model::opcode(input), adc_x_a, round_a};
  item.before.front().words = program;
  item.name = "Native/control-byte/native Carry and numeric-mode consumers: every prefix";
  item.progress = {};
  item.budget = 0;
  for (std::size_t index = 0; index < program.size(); ++index) {
    if (index == 2) {
      byte_model::apply(item.expected, input);
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, program[index]);
    }
    ++item.expected.pc;
    ++item.budget;
    item.retired = item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += index == 2 ? 3U : 1U;
    cases.push_back(item);
  }
}

void sequences(Cases &cases) {
  for (const auto field : fields) {
    for (const bool inclusive_or : {false, true}) {
      for (const Word immediate : {0U, 1U, 2U, 4U, 8U, 12U, 255U}) {
        if (field == Field::mr && inclusive_or && (immediate & ~Word{15}) != 0) { continue; }
        if (field == Field::mr && inclusive_or && (immediate & 12U) == 12U) { continue; }
        sequence(cases, {.field = field, .immediate = immediate, .inclusive_or = inclusive_or});
      }
    }
  }
}

void repeated(Cases &cases, const byte_model::Input input) {
  Word old_byte = input.inclusive_or ? 0U : status_byte;
  if (input.field == Field::mr) { old_byte = input.inclusive_or ? 1U : 3U; }
  for (const Word encoded : {0U, 4U}) {
    const Word count = encoded == 0 ? 65536U : encoded;
    for (const Word budget : {1U, 2U, count, count + 1}) {
      auto item = single(input, old_byte);
      item.before.front().words.insert(item.before.front().words.begin(),
                                       repeat_base | (encoded << 8));
      item.expected = item.initial;
      // Constant AND/OR is idempotent, including 65536 iterations.
      if (budget > 1) { byte_model::apply(item.expected, input); }
      const bool completed = budget == count + 1;
      item.expected.pc = start + (completed ? 2U : 1U);
      item.expected.lc = completed ? item.initial.lc : count + 1 - budget;
      if (budget == 1) { item.expected.lc = encoded; }
      item.budget = item.retired = budget;
      item.progress = {.instructions = budget, .estimated_cycles = 5 + (3 * (budget - 1))};
      item.interrupt_ready = completed;
      item.name = "REP ANDI/ORI partial prefixes and zero count retain exact byte and LC state";
      cases.push_back(std::move(item));
    }
  }
}

void repetitions(Cases &cases) {
  for (const auto field : fields) {
    for (const bool inclusive_or : {false, true}) {
      // MR repetitions change only IPL, never LF or the scaling selection.
      Word immediate = inclusive_or ? status_byte : 0U;
      if (field == Field::mr) { immediate = inclusive_or ? 3U : 1U; }
      repeated(cases, {.field = field, .immediate = immediate, .inclusive_or = inclusive_or});
    }
  }
}

void ending(Cases &cases, const byte_model::Input input) {
  // MR writes near a DO end are prohibited; CCR byte updates are admitted.
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word saved : {0U, lf, fv, lf | fv}) {
        auto item = single(input, repeat_old_byte);
        auto &state = item.initial;
        state.sr = (state.sr & ~fv) | lf;
        state.la = start;
        state.lc = count;
        state.sp = pointer;
        state.sc = 2;
        state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
        state.stack[(pointer + pointer_range - 1) % stack_entries] =
            (std::uint64_t{saved_la} << word_bits) | 7U;
        item.expected = state;
        byte_model::apply(item.expected, input);
        item.expected.pc = count == 1 ? start + 1 : start;
        if (count == 1) {
          item.expected.la = saved_la;
          item.expected.lc = 7;
          item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
          item.expected.sc = 0;
          item.expected.sr = (item.expected.sr & ~(lf | fv)) | saved;
        } else {
          --item.expected.lc;
        }
        item.name = "CCR ANDI/ORI DO ending keeps changed byte and restores LF/FV/frames";
        cases.push_back(std::move(item));
      }
    }
  }
}

void endings(Cases &cases) {
  for (const bool inclusive_or : {false, true}) {
    for (const Word immediate : {0U, 1U, 128U, 255U, repeat_old_byte}) {
      ending(cases, {.field = Field::ccr, .immediate = immediate, .inclusive_or = inclusive_or});
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: dsp56300_generate_control_bytes OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    dsp56300::test::matrix(cases);
    dsp56300::test::sequences(cases);
    dsp56300::test::repetitions(cases);
    dsp56300::test::endings(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent ANDI/ORI byte and sequence cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
