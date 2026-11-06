// SPDX-License-Identifier: GPL-3.0-only
// Independent bit-coordinate stores and serial-carry/ring-coordinate AGU model.
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
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
constexpr Word saturation_mode = Word{1} << 20;
constexpr Word vsl_base = 0x0ac0c0, address = 384, repeat_base = 0x0600a0;
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, pointer_range = 64;
constexpr Word saved_la = 0x70, sentinel_x = 0xabcdef, sentinel_y = 0x654321;
constexpr Word ignored_high = 0xab0000, negative_offset = 0xfffffd;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array address_modes{0U, 1U, 2U, 3U, 4U, 5U, 7U};
constexpr std::array modifiers{
    0U, 1U, 2U, 7U, 15U, 16U, 127U, 0x8001U, 0x800fU, address_model::linear};
constexpr std::array<std::uint64_t, 12> values{0,
                                               1,
                                               sign24,
                                               word_mask,
                                               std::uint64_t{word_mask} << word_bits,
                                               sign56 - 1,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask,
                                               0x80123456800000,
                                               0x7f654321ffffff,
                                               0xa5123456654321};

struct Input {
  address_model::Input agu{
      .modifier = address_model::linear, .cursor = address, .offset = 3, .mode = 4};
  std::uint64_t value = 0;
  Word status = 0;
  bool source_b = false, insert_one = false;
};

Word opcode(const Input input) {
  return vsl_base | (input.source_b ? 0x010000U : 0U) | (input.insert_one ? 16U : 0U) |
         (input.agu.mode << 11) | (input.agu.index << 8);
}

Word cycles(const Input input) {
  return input.agu.mode >= 5 ? 2U : 1U;
}

struct Transfer {
  Word address = 0, high = 0, low = 0;
};

Transfer model(Registers &state, const Input input) {
  Word effective = input.agu.cursor;
  if (input.agu.mode != 6) {
    auto agu = input.agu;
    agu.cursor = state.r.at(agu.index);
    agu.offset = state.n.at(agu.index);
    agu.modifier = state.m.at(agu.index);
    const auto next = address_model::updated(agu);
    if (agu.mode != 4 && agu.mode != 5) { state.r.at(agu.index) = next; }
    effective = agu.mode == 5 || agu.mode == 7 ? next : agu.cursor;
  }
  const auto value = input.source_b ? state.b : state.a;
  Transfer result{.address = effective, .low = input.insert_one ? 1U : 0U};
  // Walk DSP bit positions, independently of the executor's masked host shifts.
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if (((value / (std::uint64_t{1} << (bit + word_bits))) % 2) != 0) {
      result.high += Word{1} << bit;
    }
    if (bit + 1 < word_bits && ((value / (std::uint64_t{1} << bit)) % 2) != 0) {
      result.low += Word{1} << (bit + 1);
    }
  }
  return result;
}

void stores(BinaryCase &item, const Transfer transfer) {
  require(transfer.address < memory_words, "Synthetic VSL stores stay within supplied RAM");
  item.after.push_back({.words = {transfer.high}, .address = transfer.address, .space = Space::x});
  item.after.push_back({.words = {transfer.low}, .address = transfer.address, .space = Space::y});
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.source_b ? item.initial.b : item.initial.a) = input.value;
  item.initial.r.at(input.agu.index) = input.agu.mode == 6 ? word_mask : input.agu.cursor;
  item.initial.n.at(input.agu.index) = input.agu.offset;
  item.initial.m.at(input.agu.index) = input.agu.modifier;
  item.expected = item.initial;
  const auto transfer = model(item.expected, input);
  std::vector<Word> program{opcode(input)};
  if (input.agu.mode == 6) { program.push_back(input.agu.cursor); }
  item.before = {{.words = program, .address = start},
                 {.words = {sentinel_x}, .address = transfer.address, .space = Space::x},
                 {.words = {sentinel_y}, .address = transfer.address, .space = Space::y}};
  stores(item, transfer);
  item.expected.pc += static_cast<Word>(program.size());
  item.progress.estimated_cycles = cycles(input);
  item.name = "VSL raw X/Y stores; source=" + std::string(input.source_b ? "B" : "A") +
              "; insert=" + std::to_string(static_cast<unsigned>(input.insert_one)) +
              "; EA mode=" + std::to_string(input.agu.mode) +
              "; index=" + std::to_string(input.agu.index);
  return item;
}

void variants(Cases &cases, Input input) {
  for (const bool source_b : {false, true}) {
    for (const bool insert_one : {false, true}) {
      input.source_b = source_b;
      input.insert_one = insert_one;
      cases.push_back(single(input));
    }
  }
}

void raw_matrix(Cases &cases) {
  for (const auto mode : address_modes) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (const auto status : modes) {
        for (const auto value : values) {
          variants(cases,
                   {.agu = {.modifier = address_model::linear,
                            .cursor = address,
                            .offset = 3,
                            .mode = mode,
                            .index = index},
                    .value = value,
                    .status = status | status_byte});
        }
      }
    }
  }
}

void modifier_matrix(Cases &cases) {
  for (const auto modifier : modifiers) {
    const Word length =
        modifier == 0 || modifier == address_model::linear ? 128U : (modifier % 0x8000) + 1;
    for (const auto position : {0U, length / 2, length - 1}) {
      for (const auto mode : address_modes) {
        for (unsigned index = 0; index < agu_registers; ++index) {
          variants(cases,
                   {.agu = {.modifier = modifier | (index % 2 == 0 ? 0U : ignored_high),
                            .cursor = 128 + position,
                            .offset = 1,
                            .mode = mode,
                            .index = index},
                    .value = values[(index + mode) % values.size()],
                    .status = flag_s | flag_c | flag_l});
        }
      }
    }
  }
}

void bit_and_status_cases(Cases &cases) {
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const auto value = std::uint64_t{1} << bit;
    for (const auto raw : {value, accumulator_mask - value}) {
      variants(cases, {.value = raw, .status = modes[bit % modes.size()] | status_byte});
    }
  }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    variants(
        cases,
        {.value = values[flags % values.size()], .status = modes[flags % modes.size()] | flags});
  }
}

void absolute_and_edges(Cases &cases) {
  for (const Word cursor : {0U, address, static_cast<Word>(memory_words - 1)}) {
    for (const auto mode : modes) {
      for (const auto value : values) {
        variants(cases,
                 {.agu = {.modifier = ignored_high | 0x8000U,
                          .cursor = cursor,
                          .offset = sign24,
                          .mode = 6},
                  .value = value,
                  .status = mode | flag_s | flag_c | flag_v | flag_l});
      }
    }
  }
  for (const auto cursor : {0U, static_cast<Word>(memory_words - 1)}) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      variants(
          cases,
          {.agu = {.modifier = address_model::linear, .cursor = cursor, .mode = 3, .index = index},
           .value = accumulator_mask,
           .status = status_byte});
    }
  }
}

void repeated(Cases &cases, const Input input) {
  constexpr Word count = 4;
  for (const Word budget : {1U, 2U, 3U, count + 1}) {
    auto item = single(input);
    item.expected = item.initial;
    item.after.clear();
    const auto iterations = budget - 1;
    for (Word step = 0; step < iterations; ++step) { stores(item, model(item.expected, input)); }
    item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
    const bool completed = iterations == count;
    item.expected.pc = start + (completed ? 2U : 1U);
    item.expected.lc = completed ? item.initial.lc : count - iterations;
    item.interrupt_ready = completed;
    item.budget = item.retired = budget;
    item.progress = {.instructions = budget, .estimated_cycles = 5 + (cycles(input) * iterations)};
    item.name = "REP VSL partial prefixes use current R once and preserve accumulator/CCR";
    cases.push_back(std::move(item));
  }
}

void repetitions(Cases &cases) {
  for (const auto mode : {1U, 2U, 3U, 4U, 5U, 7U}) {
    for (const bool source_b : {false, true}) {
      for (const bool insert_one : {false, true}) {
        repeated(cases,
                 {.agu = {.modifier = address_model::linear,
                          .cursor = address,
                          .offset = negative_offset,
                          .mode = mode,
                          .index = 7},
                  .value = accumulator_mask,
                  .status = status_byte,
                  .source_b = source_b,
                  .insert_one = insert_one});
      }
    }
  }
}

void ending(Cases &cases, const Input input, const Word pointer) {
  for (const Word count : {1U, 2U}) {
    for (const Word saved : {0U, lf, fv, lf | fv}) {
      auto item = single(input);
      auto &state = item.initial;
      const Word end = state.pc + (input.agu.mode == 6 ? 1U : 0U);
      state.sr |= lf;
      state.la = end;
      state.lc = count;
      state.sp = pointer;
      state.sc = 2;
      state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
      state.stack[(pointer + pointer_range - 1) % stack_entries] =
          (std::uint64_t{saved_la} << word_bits) | 7U;
      item.expected = state;
      static_cast<void>(model(item.expected, input));
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
      item.name = "VSL DO end at opcode/extension with saved LF/FV and stack-index wrap";
      cases.push_back(std::move(item));
    }
  }
}

void loops(Cases &cases) {
  for (const bool extended : {false, true}) {
    for (const bool source_b : {false, true}) {
      for (const Word pointer : {0U, 15U, 31U, 63U}) {
        ending(cases,
               {.agu = {.modifier = address_model::linear,
                        .cursor = address,
                        .mode = extended ? 6U : 3U},
                .value = sign56 + word_mask,
                .status = flag_s | flag_c | flag_l,
                .source_b = source_b,
                .insert_one = true},
               pointer);
      }
    }
  }
}

void sequence(Cases &cases, const Input input) {
  constexpr Word multiply = 0x2000d0;
  constexpr Word load_x0 = 0x44e000;
  constexpr Word load_y0 = 0x4ee000;
  constexpr Word adc_x_a = 0x200021;
  constexpr Word mac_b = 0x2000da;
  constexpr Word negate_b = 0x20003e;
  const std::array program{
      multiply, opcode(input), load_x0, load_y0, adc_x_a, mac_b, opcode(input), negate_b};
  auto item = single(input);
  item.before.front().words = {program.begin(), program.end()};
  item.expected = item.initial;
  item.budget = 0;
  item.progress = {};
  item.after.clear();
  item.name = "Native arithmetic/VSL/native raw X/Y loads and Carry consumer: every prefix";
  for (std::size_t index = 0; index < program.size(); ++index) {
    if (index == 1 || index == 6) {
      stores(item, model(item.expected, input));
    } else if (index == 2 || index == 3) {
      const auto &block = item.after[index - 2];
      auto &bank = index == 2 ? item.expected.x : item.expected.y;
      bank = (bank & ~std::uint64_t{word_mask}) | block.words.front();
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, program[index]);
    }
    ++item.expected.pc;
    ++item.budget;
    item.retired = item.budget;
    item.progress = {.instructions = item.budget, .estimated_cycles = item.budget};
    cases.push_back(item);
  }
}

void sequences(Cases &cases) {
  for (const auto status : modes) {
    for (const Word flags : {0U, status_byte}) {
      for (const bool source_b : {false, true}) {
        for (const bool insert_one : {false, true}) {
          sequence(cases,
                   {.value = accumulator_mask,
                    .status = status | flags,
                    .source_b = source_b,
                    .insert_one = insert_one});
        }
      }
    }
  }
}

void reject(Cases &cases, BinaryCase item, const FaultKind kind, const Word value) {
  item.retired = 0;
  item.progress = {};
  item.interrupt_ready = false;
  item.fault = {.kind = kind, .address = start, .value = value, .pc = start};
  cases.push_back(std::move(item));
}

void faults(Cases &cases) {
  for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down}) {
    if (mode == saturation_mode) {
      cases.push_back(single({.status = mode}));
    } else {
      reject(cases, single({.status = mode}), FaultKind::unsupported_mode, mode);
    }
  }
  for (unsigned index = 1; index < agu_registers; ++index) {
    auto item = single({.agu = {.cursor = address, .mode = 6}});
    item.before.front().words.front() |= index << 8;
    const auto instruction = item.before.front().words.front();
    reject(cases, std::move(item), FaultKind::invalid_instruction, instruction);
  }
  for (const Word modifier : {0x8000U, 0x8002U, 0xc001U, 0xfffeU}) {
    auto item = single({.agu = {.cursor = address, .mode = 3}});
    item.initial.m[0] = modifier | ignored_high;
    reject(cases, std::move(item), FaultKind::unsupported_mode, modifier);
  }
  auto missing = single({.agu = {.cursor = address, .mode = 6}});
  missing.memory_words[0] = start + 1;
  missing.before.front().words.pop_back();
  reject(cases, std::move(missing), FaultKind::memory_read, 0);
  cases.back().fault.address = start + 1;
  auto multiword = single({.agu = {.cursor = address, .mode = 6}});
  multiword.before.front().words.insert(multiword.before.front().words.begin(),
                                        repeat_base | (2U << 8));
  multiword.budget = 3;
  multiword.retired = 1;
  multiword.progress = {.instructions = 1, .estimated_cycles = 5};
  multiword.interrupt_ready = false;
  multiword.fault = {.kind = FaultKind::invalid_instruction,
                     .address = start + 1,
                     .value = vsl_base | (6U << 11),
                     .pc = start + 1};
  cases.push_back(std::move(multiword));
}

void wrapping_extension(Cases &cases) {
  auto item = single({.agu = {.modifier = ignored_high, .cursor = address, .mode = 6},
                      .value = accumulator_mask,
                      .status = status_byte,
                      .insert_one = true});
  const auto program = item.before.front().words;
  item.before.front() = {.words = {program.front()}, .address = word_mask};
  item.before.push_back({.words = {program.back()}, .address = 0});
  item.memory_words[0] = static_cast<Word>(address_words);
  item.initial.pc = word_mask;
  item.expected.pc = 1;
  item.name = "VSL absolute extension wraps P:$FFFFFF to P:0";
  cases.push_back(std::move(item));
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_viterbi OUTPUT.d56"); }
    dsp56300::test::Cases cases;
    dsp56300::test::raw_matrix(cases);
    dsp56300::test::modifier_matrix(cases);
    dsp56300::test::bit_and_status_cases(cases);
    dsp56300::test::absolute_and_edges(cases);
    dsp56300::test::repetitions(cases);
    dsp56300::test::loops(cases);
    dsp56300::test::sequences(cases);
    dsp56300::test::faults(cases);
    dsp56300::test::wrapping_extension(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent VSL raw-store and address/sequence cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
