// SPDX-License-Identifier: GPL-3.0-only
// Independent signed raw-width products, quotient/remainder rounding and flags.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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
enum class Family : std::uint8_t { immediate, power, mixed, double_product };
enum class Signs : std::uint8_t { ss, su, uu };
constexpr Word immediate_base = 0x0141c0, power_base = 0x0100c0;
constexpr Word mixed_mac = 0x012680, mixed_mpy = 0x012780, double_base = 0x012480;
constexpr Word repeat_base = 0x0600a0, lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word status_changes = flag_n | flag_z | flag_e | flag_u | flag_v;
constexpr unsigned fractional_bits = 47, maximum_power = 23;
constexpr auto maximum56 = static_cast<std::int64_t>(sign56 - 1);
constexpr auto minimum56 = -static_cast<std::int64_t>(sign56);
constexpr std::uint64_t word_unit = std::uint64_t{1} << word_bits;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array values{0U, 1U, sign24 - 1, sign24, sign24 + 1, word_mask - 1, word_mask};
constexpr std::array<std::uint64_t, 8> seeds{
    0, 1, word_unit - 1, word_unit, sign56 - 1, sign56, accumulator_mask - 1, accumulator_mask};
// Inverse of the manual's encoding table; indices here are X0,X1,Y0,Y1.
constexpr std::array<std::array<Word, 4>, 4> pair_code{
    {{0, 10, 13, 4}, {2, 8, 6, 15}, {5, 14, 1, 11}, {12, 7, 3, 9}}};
constexpr std::array immediate_code{0U, 2U, 1U, 3U}, power_code{1U, 3U, 2U, 0U};
constexpr std::array<std::pair<Word, Word>, 8> pair_values{{{0, 0},
                                                            {1, 1},
                                                            {word_mask, sign24},
                                                            {sign24, word_mask},
                                                            {sign24 - 1, sign24 + 1},
                                                            {sign24 + 1, sign24 - 1},
                                                            {word_mask, word_mask},
                                                            {sign24, sign24}}};

struct Input {
  Family family = Family::immediate;
  Signs signs = Signs::ss;
  Word coefficient = 0, left_value = 0, right_value = 0, status = 0;
  unsigned left = 0, right = 2;
  std::uint64_t accumulator = 0;
  bool accumulate = false, rounded = false, subtract = false, destination_b = false;
};

struct Witness {
  std::size_t sum_overflow = 0, round_overflow = 0, half_ties = 0;
  std::size_t negative_discard = 0, unsigned_extension = 0;
};

Witness witnessed;

Word data(const Registers &state, const unsigned selector) {
  const auto value = selector < 2 ? state.x : state.y;
  return static_cast<Word>(value >> (selector % 2 == 0 ? 0U : word_bits)) & word_mask;
}

struct WordWrite {
  unsigned selector = 0;
  Word value = 0;
};

void set_data(Registers &state, const WordWrite input) {
  const auto selector = input.selector;
  const auto value = input.value;
  auto &bank = selector < 2 ? state.x : state.y;
  const auto shift = selector % 2 == 0 ? 0U : word_bits;
  bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
}

Word opcode(const Input input) {
  if (input.family == Family::immediate || input.family == Family::power) {
    const bool power = input.family == Family::power;
    return (power ? power_base | (input.coefficient << 8) : immediate_base) |
           ((power ? power_code[input.left] : immediate_code[input.left]) << 4) |
           (input.destination_b ? 8U : 0U) | (input.subtract ? 4U : 0U) |
           (input.accumulate ? 2U : 0U) | (input.rounded ? 1U : 0U);
  }
  Word base = input.accumulate ? mixed_mac : mixed_mpy;
  if (input.family == Family::double_product) { base = double_base; }
  const Word sign = input.signs == Signs::uu ? Word{1} << 6 : 0U;
  const Word high_sign =
      input.family == Family::double_product && input.signs != Signs::ss ? Word{1} << 8 : 0U;
  return base | sign | high_sign | pair_code[input.left][input.right] |
         (input.destination_b ? Word{1} << 5 : 0U) | (input.subtract ? Word{1} << 4 : 0U);
}

struct FlagInput {
  std::uint64_t raw = 0;
  Word status = 0;
};

Word result_flags(const FlagInput input) {
  const auto raw = input.raw;
  const auto status = input.status;
  const unsigned point =
      fractional_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
  const auto bound = std::int64_t{1} << point;
  const auto signed_result = signed56(raw);
  Word flags = raw == 0 ? flag_z : 0;
  if (signed_result < 0) { flags |= flag_n; }
  if (signed_result < -bound || signed_result >= bound) { flags |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(bound);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { flags |= flag_u; }
  return flags;
}

struct RoundInput {
  std::int64_t value = 0;
  Word status = 0;
};

std::int64_t quantize(const RoundInput input) {
  const auto value = input.value;
  const auto status = input.status;
  const unsigned bits =
      word_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  auto whole = value / unit;
  auto remainder = value % unit;
  if (remainder < 0) {
    --whole;
    remainder += unit;
  }
  if (remainder == unit / 2) { ++witnessed.half_ties; }
  if (remainder > unit / 2 ||
      (remainder == unit / 2 && ((status & mode_rm) != 0 || whole % 2 != 0))) {
    ++whole;
  }
  return whole * unit;
}

std::int64_t term(const Registers &state, const Input input) {
  const auto left_word = data(state, input.left);
  const auto left = input.signs == Signs::uu ? left_word : signed24(left_word);
  std::int64_t right = signed24(input.coefficient);
  if (input.family == Family::power) {
    if (input.coefficient == 0 || input.coefficient > maximum_power) {
      throw std::runtime_error("No positive Q1.23 power coefficient in this model domain");
    }
    right = static_cast<std::int64_t>(word_unit / (std::uint64_t{1} << input.coefficient) / 2);
  } else if (input.family == Family::mixed || input.family == Family::double_product) {
    const auto right_word = data(state, input.right);
    right = input.signs == Signs::ss ? signed24(right_word) : right_word;
  }
  const auto full = 2 * left * right;
  if (input.signs == Signs::uu && full >= (std::int64_t{1} << (word_bits * 2))) {
    ++witnessed.unsigned_extension;
  }
  return input.subtract ? -full : full;
}

void model(Registers &state, const Input input) {
  auto &destination = input.destination_b ? state.b : state.a;
  std::int64_t initial = input.accumulate ? signed56(destination) : 0;
  if (input.family == Family::double_product) {
    initial = signed56(destination) / static_cast<std::int64_t>(word_unit);
    if (signed56(destination) % static_cast<std::int64_t>(word_unit) < 0) {
      --initial;
      ++witnessed.negative_discard;
    }
  }
  auto full = initial + term(state, input);
  bool overflowed = full < minimum56 || full > maximum56;
  if (overflowed) { ++witnessed.sum_overflow; }
  auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  if (input.rounded) {
    const auto wrap_delta = full - signed56(raw);
    full = quantize({.value = full, .status = state.sr});
    const auto isolated_round = full - wrap_delta;
    if (isolated_round < minimum56 || isolated_round > maximum56) { ++witnessed.round_overflow; }
    overflowed = full < minimum56 || full > maximum56;
    raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  }
  const bool saturates = input.family == Family::immediate || input.family == Family::power ||
                         (input.family == Family::double_product && input.signs == Signs::ss);
  if (saturates) {
    const auto limited =
        saturation_model::apply(raw, {.status = state.sr, .rounded = input.rounded});
    raw = limited.value;
    overflowed = overflowed || limited.limited;
  }
  destination = raw;
  state.sr = (state.sr & ~status_changes) | result_flags({.raw = raw, .status = state.sr}) |
             (overflowed ? flag_v | flag_l : 0U);
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.destination_b ? item.initial.b : item.initial.a) = input.accumulator;
  set_data(item.initial, {.selector = input.left, .value = input.left_value});
  if (input.family == Family::mixed || input.family == Family::double_product) {
    set_data(item.initial, {.selector = input.right, .value = input.right_value});
  }
  item.expected = item.initial;
  model(item.expected, input);
  std::vector program{opcode(input)};
  if (input.family == Family::immediate) { program.push_back(input.coefficient); }
  item.expected.pc += static_cast<Word>(program.size());
  item.progress.estimated_cycles = program.size();
  item.before = {{.words = std::move(program), .address = start}};
  item.name = "Extended product family=" + std::to_string(static_cast<unsigned>(input.family)) +
              "; opcode=" + std::to_string(opcode(input));
  return item;
}

void immediate_operands(Cases &cases, Input input) {
  for (std::size_t left = 0; left < values.size(); ++left) {
    input.left_value = values[left];
    for (std::size_t right = 0; right < values.size(); ++right) {
      input.coefficient = values[right];
      input.accumulator = seeds[(left + right + input.status) % seeds.size()];
      cases.push_back(single(input));
    }
  }
}

void immediate_statuses(Cases &cases, Input input) {
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      input.status = mode | flags;
      immediate_operands(cases, input);
    }
  }
}

void power_operands(Cases &cases, Input input) {
  for (unsigned power = 1; power <= maximum_power; ++power) {
    input.coefficient = power;
    for (const auto mode : modes) {
      input.status = mode | (power % 2 == 0 ? status_byte : 0U);
      for (const Word value : {sign24 - 1, sign24 + 1}) {
        input.left_value = value;
        input.accumulator = seeds[(power + input.status) % seeds.size()];
        cases.push_back(single(input));
      }
    }
  }
}

void variants(Cases &cases, Input input) {
  for (const bool accumulate : {false, true}) {
    input.accumulate = accumulate;
    for (const bool rounded : {false, true}) {
      input.rounded = rounded;
      if (input.family == Family::immediate) {
        immediate_statuses(cases, input);
      } else {
        power_operands(cases, input);
      }
    }
  }
}

void immediate_matrix(Cases &cases) {
  for (const bool destination_b : {false, true}) {
    for (unsigned source = 0; source < 4; ++source) {
      for (const bool subtract : {false, true}) {
        variants(cases, {.left = source, .subtract = subtract, .destination_b = destination_b});
        variants(cases,
                 {.family = Family::power,
                  .left = source,
                  .subtract = subtract,
                  .destination_b = destination_b});
      }
    }
  }
}

void rounding_boundaries(Cases &cases, Input input) {
  for (const auto mode : modes) {
    const unsigned bits =
        word_bits + ((mode & mode_down) != 0 ? 1U : 0U) - ((mode & mode_up) != 0 ? 1U : 0U);
    const auto half = std::uint64_t{1} << (bits - 1);
    const std::array boundary{
        std::uint64_t{0}, sign56 - 1, sign56, accumulator_mask, half - 1, half, half + 1, 3 * half};
    for (const Word flags : {0U, status_byte}) {
      input.status = mode | flags;
      for (const auto accumulator : boundary) {
        input.accumulator = accumulator;
        cases.push_back(single(input));
      }
    }
  }
}

void immediate_boundaries(Cases &cases) {
  for (const bool rounded : {false, true}) {
    for (const bool subtract : {false, true}) {
      for (unsigned selector = 0; selector < 4; ++selector) {
        for (const bool destination_b : {false, true}) {
          rounding_boundaries(cases,
                              {.coefficient = 0,
                               .left_value = 1,
                               .left = selector,
                               .accumulate = true,
                               .rounded = rounded,
                               .subtract = subtract,
                               .destination_b = destination_b});
          rounding_boundaries(cases,
                              {.coefficient = 1,
                               .left_value = 1,
                               .left = selector,
                               .accumulate = true,
                               .rounded = rounded,
                               .subtract = subtract,
                               .destination_b = destination_b});
        }
      }
    }
  }
}

void mixed_operands(Cases &cases, Input input) {
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      input.status = mode | flags;
      for (std::size_t index = 0; index < pair_values.size(); ++index) {
        input.left_value = pair_values[index].first;
        input.right_value = pair_values[index].second;
        input.accumulator = seeds[(index + mode) % seeds.size()];
        cases.push_back(single(input));
      }
    }
  }
}

void mixed_variants(Cases &cases, Input input) {
  if (input.family == Family::double_product) {
    input.accumulate = true;
    for (const auto sign : {Signs::ss, Signs::su, Signs::uu}) {
      input.signs = sign;
      mixed_operands(cases, input);
    }
  } else {
    for (const auto sign : {Signs::su, Signs::uu}) {
      input.signs = sign;
      for (const bool accumulate : {false, true}) {
        input.accumulate = accumulate;
        mixed_operands(cases, input);
      }
    }
  }
}

void mixed_matrix(Cases &cases) {
  for (unsigned left = 0; left < 4; ++left) {
    for (unsigned right = 0; right < 4; ++right) {
      for (const bool destination_b : {false, true}) {
        for (const bool subtract : {false, true}) {
          mixed_variants(cases,
                         {.family = Family::mixed,
                          .left = left,
                          .right = right,
                          .subtract = subtract,
                          .destination_b = destination_b});
          mixed_variants(cases,
                         {.family = Family::double_product,
                          .left = left,
                          .right = right,
                          .subtract = subtract,
                          .destination_b = destination_b});
        }
      }
    }
  }
}

void accumulated_boundaries(Cases &cases, Input input) {
  constexpr std::array<std::uint64_t, 4> limits{
      sign56 - 1, sign56, word_unit - 1, accumulator_mask};
  input.left_value = word_mask;
  input.right_value = sign24;
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      input.status = mode | flags;
      for (const auto value : limits) {
        input.accumulator = value;
        cases.push_back(single(input));
      }
    }
  }
}

void mixed_boundaries(Cases &cases, Input input) {
  for (const bool destination_b : {false, true}) {
    input.destination_b = destination_b;
    for (const bool subtract : {false, true}) {
      input.subtract = subtract;
      for (const auto sign : {Signs::su, Signs::uu}) {
        input.signs = sign;
        accumulated_boundaries(cases, input);
      }
    }
  }
}

BinaryCase repeated(const Input input) {
  constexpr Word count = 7;
  auto item = single(input);
  item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
  for (Word iteration = 1; iteration < count; ++iteration) { model(item.expected, input); }
  item.expected.pc = start + 2;
  item.budget = item.retired = count + 1;
  item.progress = {.instructions = count + 1, .estimated_cycles = count + 5};
  item.name += "; REP reuses sources and current accumulator";
  return item;
}

struct Loop {
  Word pointer = 0, count = 1, outer = 0;
};

BinaryCase loop(const Input input, const Loop frame) {
  auto item = single(input);
  constexpr Word saved_la = 0x70;
  constexpr Word saved_lc = 7;
  constexpr Word pointer_range = 64;
  auto &initial = item.initial;
  initial.sr |= lf;
  initial.la = item.expected.pc - 1;
  initial.lc = frame.count;
  initial.sp = frame.pointer;
  initial.sc = 2;
  initial.stack[frame.pointer % stack_entries] =
      (std::uint64_t{start} << word_bits) | frame.outer | flag_v;
  initial.stack[(frame.pointer + pointer_range - 1) % stack_entries] =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  model(item.expected, input);
  if (frame.count > 1) {
    --item.expected.lc;
    item.expected.pc = start;
  } else {
    item.expected.pc = initial.la + 1;
    item.expected.la = saved_la;
    item.expected.lc = saved_lc;
    item.expected.sp = (frame.pointer + pointer_range - 2) % pointer_range;
    item.expected.sc = 0;
    item.expected.sr = (item.expected.sr & ~(lf | fv)) | frame.outer;
  }
  item.name += "; loop-ending word or extension, saved LF/FV";
  return item;
}

void interactions(Cases &cases, const Input input) {
  if (input.family != Family::immediate) { cases.push_back(repeated(input)); }
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word outer : {0U, lf, fv, lf | fv}) {
        cases.push_back(loop(input, {.pointer = pointer, .count = count, .outer = outer}));
      }
    }
  }
}

BinaryCase rejected(const Input input, const FaultKind kind, const Word value) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  item.expected = item.initial;
  std::vector<Word> program{opcode(input)};
  if (input.family == Family::immediate) { program.push_back(input.coefficient); }
  item.before = {{.words = std::move(program), .address = start}};
  item.retired = 0;
  item.progress = {};
  item.interrupt_ready = false;
  item.fault = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
  item.name = "Explicit product undefined-field/unimplemented-mode boundary";
  return item;
}

void faults(Cases &cases, Input input) {
  for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_down | mode_up}) {
    input.status = mode;
    auto item = rejected(input, FaultKind::unsupported_mode, mode);
    if (mode == saturation_model::mode) {
      model(item.expected, input);
      item.expected.pc += static_cast<Word>(item.before.front().words.size());
      item.fault = {};
      item.retired = 1;
      item.progress = {.instructions = 1, .estimated_cycles = item.before.front().words.size()};
      item.interrupt_ready = true;
      item.name = "Qualified SM product/bypass on the former unsupported stimulus";
    }
    cases.push_back(std::move(item));
  }
}

void sequence_prefixes(Cases &cases) {
  constexpr Word add_b_a = 0x200010;
  constexpr Word mac_a = 0x2000d2;
  const std::array steps{
      Input{.coefficient = sign24, .left = 2, .accumulate = true, .rounded = true},
      Input{.family = Family::mixed,
            .signs = Signs::su,
            .left = 3,
            .right = 0,
            .accumulate = true,
            .destination_b = true},
      Input{.family = Family::double_product,
            .signs = Signs::ss,
            .left = 1,
            .right = 2,
            .accumulate = true},
      Input{.family = Family::power,
            .coefficient = 7,
            .left = 0,
            .rounded = true,
            .destination_b = true}};
  std::vector<Word> program{mac_a};
  for (const auto step : steps) {
    program.push_back(opcode(step));
    if (step.family == Family::immediate) { program.push_back(step.coefficient); }
    program.push_back(add_b_a);
  }
  for (const auto mode : modes) {
    for (const auto accumulator : seeds) {
      BinaryCase item;
      item.initial = register_model::seed(mode | flag_c);
      item.initial.a = accumulator;
      item.expected = item.initial;
      item.before = {{.words = program, .address = start}};
      item.name =
          "Native multiply/product helper/native consumers: every prefix and complete flags";
      const auto previous = item.expected;
      sequence_model::alu(item.expected, previous, mac_a);
      ++item.expected.pc;
      cases.push_back(item);
      for (const auto step : steps) {
        model(item.expected, step);
        item.expected.pc += step.family == Family::immediate ? 2U : 1U;
        ++item.budget;
        item.retired = item.budget;
        item.progress = {.instructions = item.budget, .estimated_cycles = item.expected.pc - start};
        cases.push_back(item);
        const auto old = item.expected;
        sequence_model::alu(item.expected, old, add_b_a);
        ++item.expected.pc;
        ++item.budget;
        item.retired = item.budget;
        item.progress = {.instructions = item.budget, .estimated_cycles = item.expected.pc - start};
        cases.push_back(item);
      }
    }
  }
}

void extension_boundaries(Cases &cases) {
  for (unsigned variant = 0; variant < 4; ++variant) {
    for (unsigned source = 0; source < 4; ++source) {
      for (const bool destination_b : {false, true}) {
        for (const bool subtract : {false, true}) {
          const Input input{.coefficient = sign24 + 1,
                            .left_value = word_mask,
                            .left = source,
                            .accumulator = sign56 - 1,
                            .accumulate = (variant & 2U) != 0,
                            .rounded = (variant & 1U) != 0,
                            .subtract = subtract,
                            .destination_b = destination_b};
          auto item = rejected(input, FaultKind::memory_read, 0);
          item.before.front().words.resize(1);
          item.memory_words[0] = start + 1;
          item.fault.address = start + 1;
          cases.push_back(std::move(item));
          auto repeat = rejected(input, FaultKind::invalid_instruction, opcode(input));
          repeat.before.front().words.insert(repeat.before.front().words.begin(),
                                             repeat_base | (2U << 8));
          repeat.budget = 2;
          repeat.retired = 1;
          repeat.progress = {.instructions = 1, .estimated_cycles = 5};
          repeat.fault.address = repeat.fault.pc = start + 1;
          cases.push_back(std::move(repeat));
        }
      }
    }
    auto wrap = single({.coefficient = sign24 + 1,
                        .left_value = word_mask,
                        .accumulator = sign56 - 1,
                        .accumulate = (variant & 2U) != 0,
                        .rounded = (variant & 1U) != 0});
    const auto extension = wrap.before.front().words[1];
    wrap.before.front().words.resize(1);
    wrap.before.front().address = word_mask;
    wrap.before.push_back({.words = {extension}, .address = 0});
    wrap.memory_words[0] = static_cast<Word>(address_words);
    wrap.initial.pc = word_mask;
    wrap.expected.pc = 1;
    wrap.name = "Immediate multiply extension wraps P:$FFFFFF to P:0";
    cases.push_back(std::move(wrap));
  }
}

void ordinary(Cases &cases) {
  immediate_matrix(cases);
  immediate_boundaries(cases);
  sequence_prefixes(cases);
  extension_boundaries(cases);
  for (const auto family : {Family::immediate, Family::power}) {
    for (const bool rounded : {false, true}) {
      const Input input{.family = family,
                        .coefficient = 7,
                        .left_value = sign24 + 1,
                        .accumulator = sign56 - 1,
                        .accumulate = true,
                        .rounded = rounded};
      interactions(cases, input);
      faults(cases, input);
    }
  }
  for (const Word power : {0U, 24U, 31U}) {
    const Input input{.family = Family::power, .coefficient = power};
    cases.push_back(
        rejected(input,
                 power == 0 ? FaultKind::unsupported_mode : FaultKind::invalid_instruction,
                 opcode(input)));
  }
}

void mixed(Cases &cases) {
  mixed_matrix(cases);
  for (unsigned left = 0; left < 4; ++left) {
    for (unsigned right = 0; right < 4; ++right) {
      mixed_boundaries(cases,
                       {.family = Family::mixed, .left = left, .right = right, .accumulate = true});
      mixed_boundaries(
          cases,
          {.family = Family::double_product, .left = left, .right = right, .accumulate = true});
    }
  }
  for (const auto family : {Family::mixed, Family::double_product}) {
    for (const auto sign : {Signs::ss, Signs::su, Signs::uu}) {
      if (family == Family::mixed && sign == Signs::ss) { continue; }
      const Input input{.family = family,
                        .signs = sign,
                        .left_value = word_mask,
                        .right_value = sign24,
                        .accumulator = sign56,
                        .accumulate = true};
      interactions(cases, input);
      faults(cases, input);
    }
  }
  auto reserved = rejected(
      {.family = Family::double_product}, FaultKind::invalid_instruction, double_base | (1U << 6));
  reserved.before.front().words[0] = double_base | (1U << 6);
  cases.push_back(std::move(reserved));
}

void generate(const std::array<const char *, 2> paths) {
  Cases cases;
  ordinary(cases);
  const auto first_count = cases.size();
  write_fixture(paths[0], std::move(cases));
  cases = {};
  mixed(cases);
  const auto second_count = cases.size();
  write_fixture(paths[1], std::move(cases));
  if (witnessed.sum_overflow == 0 || witnessed.round_overflow == 0 || witnessed.half_ties == 0 ||
      witnessed.negative_discard == 0 || witnessed.unsigned_extension == 0) {
    throw std::runtime_error("Product matrix missed a required arithmetic witness");
  }
  std::cout << first_count << " immediate/power and " << second_count << " mixed/DMAC cases\n"
            << witnessed.sum_overflow << " sum overflows; " << witnessed.round_overflow
            << " rounding overflows; " << witnessed.half_ties << " half ties; "
            << witnessed.negative_discard << " negative shifted remainders; "
            << witnessed.unsigned_extension << " extended positive products\n";
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_products IMMEDIATE.d56 MIXED.d56");
    }
    dsp56300::test::generate({argv[1], argv[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
