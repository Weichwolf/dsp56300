// SPDX-License-Identifier: GPL-3.0-only
// Independent per-bit shifts and leading-run counts; never executes the core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
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
using Bits = std::array<bool, accumulator_bits>;
constexpr Word normf_base = 0x0c1e20, clb_base = 0x0c1e00;
constexpr int minimum_count = -55, maximum_count = 56;
constexpr Word pointer_range = 64, saved_la = 0x70, native_x1 = 0x123456;
constexpr Word repeat_base = 0x0600a0, lf = Word{1} << 15, fv = Word{1} << 16;
constexpr std::array source_code{2U, 3U, 4U, 6U, 5U, 7U};
constexpr std::array source_registers{12U, 13U, 4U, 5U, 6U, 7U};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array counts{-55, -48, -47, -24, -1, 0, 1, 8, 24, 48, 55, 56};
constexpr std::array<std::uint64_t, 12> values{0,
                                               1,
                                               sign24 - 1,
                                               word_mask,
                                               std::uint64_t{1} << 24,
                                               std::uint64_t{1} << 46,
                                               std::uint64_t{1} << 47,
                                               std::uint64_t{1} << 54,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask,
                                               0x5a123456789abc};
std::size_t left_overflows = 0, full_right_shifts = 0, normalization_witnesses = 0;

struct Input {
  std::uint64_t value = 0;
  Word status = 0;
  int count = 0;
  unsigned selector = 0;
  bool destination_b = false;
};

Bits unpack(std::uint64_t value) {
  Bits result{};
  for (auto &bit : result) {
    bit = value % 2 != 0;
    value /= 2;
  }
  return result;
}

std::uint64_t pack(const Bits &bits) {
  std::uint64_t result = 0;
  for (std::size_t index = bits.size(); index != 0; --index) {
    result = result * 2 + (bits[index - 1] ? 1U : 0U);
  }
  return result;
}

Word source(const Registers &state, const unsigned selector) {
  const std::array words{static_cast<Word>(state.a >> word_bits),
                         static_cast<Word>(state.b >> word_bits),
                         static_cast<Word>(state.x),
                         static_cast<Word>(state.x >> word_bits),
                         static_cast<Word>(state.y),
                         static_cast<Word>(state.y >> word_bits)};
  return words.at(selector) & word_mask;
}

Word opcode(const Input input) {
  return normf_base | (source_code.at(input.selector) << 1) | (input.destination_b ? 1U : 0U);
}

void model(Registers &state, const Input input) {
  const auto count = signed24(source(state, input.selector));
  require(count >= minimum_count && count <= maximum_count,
          "NORMF fixture shift is in the documented domain");
  auto &destination = input.destination_b ? state.b : state.a;
  auto result = unpack(destination);
  bool overflowed = false;
  const bool left = count < 0;
  const auto iterations = left ? -count : count;
  for (std::int64_t step = 0; step < iterations; ++step) {
    Bits next{};
    if (left) {
      for (std::size_t bit = 1; bit < next.size(); ++bit) { next[bit] = result[bit - 1]; }
      overflowed = overflowed || result.back() != next.back();
    } else {
      for (std::size_t bit = 0; bit + 1 < next.size(); ++bit) { next[bit] = result[bit + 1]; }
      next.back() = result.back();
    }
    result = next;
  }
  destination = pack(result);
  sequence_model::flags(state, {.full = signed56(destination), .overflow = overflowed});
  if (overflowed) { ++left_overflows; }
  if (count == maximum_count) { ++full_right_shifts; }
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.destination_b ? item.initial.b : item.initial.a) = input.value;
  // The source may alias the destination's MSP. Seed it last, then read old state.
  register_model::write(item.initial,
                        source_registers.at(input.selector),
                        static_cast<Word>(input.count) & word_mask);
  item.expected = item.initial;
  model(item.expected, input);
  ++item.expected.pc;
  item.before = {{.words = {opcode(input)}, .address = start}};
  item.name = "NORMF per-bit shift; source=" + std::to_string(input.selector) +
              "; shift=" + std::to_string(input.count);
  return item;
}

void all_counts(Cases &cases) {
  for (unsigned selector = 0; selector < source_code.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      for (int count = minimum_count; count <= maximum_count; ++count) {
        for (const auto value : values) {
          cases.push_back(single({.value = value,
                                  .status = (count % 2 == 0 ? status_byte : flag_s | flag_v),
                                  .count = count,
                                  .selector = selector,
                                  .destination_b = destination_b}));
        }
      }
    }
  }
}

void modes_and_flags(Cases &cases) {
  for (unsigned selector = 0; selector < source_code.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      for (const auto mode : modes) {
        for (const int count : counts) {
          for (const auto value : {std::uint64_t{0}, sign56 - 1, sign56, accumulator_mask}) {
            cases.push_back(single({.value = value,
                                    .status = mode | status_byte,
                                    .count = count,
                                    .selector = selector,
                                    .destination_b = destination_b}));
          }
        }
      }
      for (Word flags = 0; flags <= status_byte; ++flags) {
        cases.push_back(single({.value = values[flags % values.size()],
                                .status = flags,
                                .count = counts[flags % counts.size()],
                                .selector = selector,
                                .destination_b = destination_b}));
      }
    }
  }
}

int leading_shift(const std::uint64_t value) {
  if (value == 0) { return 0; }
  const auto bits = unpack(value);
  int leading = 0;
  for (std::size_t bit = bits.size(); bit != 0 && bits[bit - 1] == bits.back(); --bit) {
    ++leading;
  }
  return 9 - leading;
}

void clb_model(Registers &state, const bool source_b, const bool destination_b) {
  const int count = leading_shift(source_b ? state.b : state.a);
  const auto value = static_cast<std::uint64_t>(static_cast<std::int64_t>(count) *
                                                (std::int64_t{1} << word_bits)) &
                     accumulator_mask;
  (destination_b ? state.b : state.a) = value;
  state.sr &= ~(flag_n | flag_z | flag_v);
  if (count == 0) { state.sr |= flag_z; }
  if (count < 0) { state.sr |= flag_n; }
}

void assert_normalized(const std::uint64_t raw) {
  const auto value = signed56(raw);
  constexpr auto lower = std::int64_t{1} << 46;
  constexpr auto upper = std::int64_t{1} << 47;
  require(value == 0 || (value >= lower && value < upper) || (value >= -upper && value < -lower),
          "CLB/NORMF has a zero or a normalized signed fractional result");
  ++normalization_witnesses;
}

void chain(Cases &cases, const Input input, const bool clb_alias) {
  auto item = single(input);
  const bool target_b = input.destination_b;
  const bool count_b = clb_alias ? target_b : !target_b;
  (target_b ? item.initial.b : item.initial.a) = input.value;
  item.expected = item.initial;
  const Word clb = clb_base | (target_b ? 2U : 0U) | (count_b ? 1U : 0U);
  const Input normalize{.selector = count_b ? 1U : 0U, .destination_b = target_b};
  item.before.front().words = {clb, opcode(normalize)};
  item.name = "CLB/NORMF independent leading-run count and bit shifts; every prefix";
  clb_model(item.expected, target_b, count_b);
  ++item.expected.pc;
  cases.push_back(item);
  model(item.expected, normalize);
  ++item.expected.pc;
  item.budget = item.dispatches = 2;
  item.progress = {.instructions = 2, .estimated_cycles = 2};
  if (!clb_alias) { assert_normalized(target_b ? item.expected.b : item.expected.a); }
  cases.push_back(std::move(item));
}

void normalization_chains(Cases &cases) {
  for (const auto mode : modes) {
    for (const bool destination_b : {false, true}) {
      for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
        const auto value = std::uint64_t{1} << bit;
        for (const auto raw : {value, accumulator_mask - value}) {
          for (const bool clb_alias : {false, true}) {
            chain(cases,
                  {.value = raw, .status = mode | status_byte, .destination_b = destination_b},
                  clb_alias);
          }
        }
      }
      for (const auto value : {std::uint64_t{0}, accumulator_mask}) {
        chain(cases,
              {.value = value, .status = mode | flag_c | flag_l, .destination_b = destination_b},
              false);
      }
    }
  }
}

BinaryCase repeated(const Input input, const Word budget) {
  constexpr Word count = 5;
  auto item = single(input);
  // A self-alias starts at A1/B1=1; the first shift changes the next count to zero.
  if (input.selector < 2 && (input.selector == 1) == input.destination_b) {
    (input.destination_b ? item.initial.b : item.initial.a) = (std::uint64_t{1} << word_bits) | 1U;
  }
  item.expected = item.initial;
  const auto iterations = budget - 1;
  for (Word step = 0; step < iterations; ++step) { model(item.expected, input); }
  item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
  const bool completed = iterations == count;
  item.expected.pc = start + (completed ? 2U : 1U);
  item.expected.lc = completed ? item.initial.lc : count - iterations;
  item.interruptible = completed;
  item.budget = item.dispatches = budget;
  item.progress = {.instructions = budget, .estimated_cycles = 5 + iterations};
  item.name = "REP NORMF partial budgets re-read the current source and old Carry";
  return item;
}

void repeats(Cases &cases) {
  constexpr std::array shifts{minimum_count, -1, 0, 1, maximum_count};
  for (const int shift : shifts) {
    for (unsigned selector = 0; selector < source_code.size(); ++selector) {
      for (const bool destination_b : {false, true}) {
        for (const Word budget : {1U, 2U, 4U, 6U}) {
          cases.push_back(repeated({.value = sign56 + 1,
                                    .status = status_byte,
                                    .count = shift,
                                    .selector = selector,
                                    .destination_b = destination_b},
                                   budget));
        }
      }
    }
  }
}

void loop_endings(Cases &cases) {
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word saved : {0U, lf, fv, lf | fv}) {
        const Input input{.value = sign56, .status = status_byte, .count = 56, .selector = 4};
        auto item = single(input);
        auto &initial = item.initial;
        initial.sr |= lf;
        initial.la = start;
        initial.lc = count;
        initial.sp = pointer;
        initial.sc = 2;
        initial.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
        initial.stack[(pointer + pointer_range - 1) % stack_entries] =
            (std::uint64_t{saved_la} << word_bits) | 7U;
        item.expected = initial;
        model(item.expected, input);
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
        item.name = "NORMF at DO end preserves saved LF/FV and wraps stack indices";
        cases.push_back(std::move(item));
      }
    }
  }
}

struct Step {
  Word instruction = 0;
  unsigned selector = 0;
  bool normalization = false, destination_b = false;
};

void native_sequences(Cases &cases) {
  const std::array steps{Step{.instruction = 0x2000d0},
                         Step{.selector = 2, .normalization = true},
                         Step{.instruction = 0x200021},
                         Step{.selector = 4, .normalization = true, .destination_b = true},
                         Step{.instruction = 0x200039},
                         Step{.instruction = 0x200041},
                         Step{.selector = 0, .normalization = true},
                         Step{.instruction = 0x2000d2}};
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      BinaryCase item;
      item.initial = register_model::seed(mode);
      item.initial.sr = mode | flags;
      item.initial.x = (std::uint64_t{native_x1} << word_bits) | 1U;
      item.initial.y = (std::uint64_t{word_mask} << word_bits) | 2U;
      item.expected = item.initial;
      std::vector<Word> program;
      program.reserve(steps.size());
      for (const auto step : steps) {
        program.push_back(step.normalization ? opcode({.selector = step.selector,
                                                       .destination_b = step.destination_b})
                                             : step.instruction);
      }
      item.before = {{.words = program, .address = start}};
      item.budget = 0;
      item.progress = {};
      item.name = "Native arithmetic/NORMF/Carry consumer prefixes and dirty accumulators";
      for (const auto step : steps) {
        if (step.normalization) {
          // The final self-alias consumes A1 supplied by the preceding native TFR.
          model(item.expected, {.selector = step.selector, .destination_b = step.destination_b});
        } else {
          const auto old = item.expected;
          sequence_model::alu(item.expected, old, step.instruction);
        }
        ++item.expected.pc;
        ++item.budget;
        item.dispatches = item.budget;
        item.progress = {.instructions = item.budget, .estimated_cycles = item.budget};
        cases.push_back(item);
      }
    }
  }
}

void fault(Cases &cases, const BinaryCase &original, const FaultKind kind, const Word value) {
  auto item = original;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
  cases.push_back(std::move(item));
}

void faults(Cases &cases) {
  for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down}) {
    auto item = single({.status = mode, .count = 1, .selector = 2});
    if (mode == 0x100000U) {
      item.name = "NORMF BFU shift bypasses arithmetic saturation with SM set";
      cases.push_back(std::move(item));
      continue;
    }
    fault(cases, item, FaultKind::unsupported_mode, mode);
  }
  for (const Word selector : {0U, 1U}) {
    for (const Word target : {0U, 1U}) {
      auto item = single({});
      const Word instruction = normf_base | (selector << 1) | target;
      item.before.front().words = {instruction};
      fault(cases, item, FaultKind::invalid_instruction, instruction);
    }
  }
  for (const int count : {-8388608, -57, -56, 57, 8388607}) {
    for (unsigned selector = 0; selector < source_code.size(); ++selector) {
      for (const bool destination_b : {false, true}) {
        const Input input{.selector = selector, .destination_b = destination_b};
        auto item = single(input);
        register_model::write(
            item.initial, source_registers[selector], static_cast<Word>(count) & word_mask);
        fault(cases, item, FaultKind::unsupported_mode, opcode(input));
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: dsp56300_generate_normalization OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    dsp56300::test::all_counts(cases);
    dsp56300::test::modes_and_flags(cases);
    dsp56300::test::normalization_chains(cases);
    dsp56300::test::repeats(cases);
    dsp56300::test::loop_endings(cases);
    dsp56300::test::native_sequences(cases);
    dsp56300::test::faults(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent NORMF/CLB normalization cases; "
              << dsp56300::test::normalization_witnesses
              << " mathematical normalization witnesses; " << dsp56300::test::left_overflows
              << " overflow model calls; " << dsp56300::test::full_right_shifts
              << " full-width right-shift model calls\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
