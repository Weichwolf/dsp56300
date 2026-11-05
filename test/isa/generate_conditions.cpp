// SPDX-License-Identifier: GPL-3.0-only
// Independent truth table and register/stack expectations; no emulator execution.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
constexpr Word flags_end = 128;
constexpr Word all_conditions = 16;
constexpr Word word_shift = 24;
constexpr Word cursor = 0x180;
constexpr Word offset_n = word_mask - 2;
constexpr Word relative_mask = 0x1ff;
constexpr Word short_target = 0x345;
constexpr Word long_target = 0xfedcba;

struct ConditionCase {
  Word flags = 0, selector = 0;
};

bool expected_condition(const ConditionCase input) {
  const bool carry = (input.flags & flag_c) != 0;
  const bool overflow = (input.flags & flag_v) != 0;
  const bool zero = (input.flags & flag_z) != 0;
  const bool negative = (input.flags & flag_n) != 0;
  const bool unnormalized = (input.flags & flag_u) != 0;
  const bool extension = (input.flags & flag_e) != 0;
  const bool limit = (input.flags & flag_l) != 0;
  const bool less = negative != overflow;
  const bool normalized = zero || (!unnormalized && !extension);
  const std::array<bool, all_conditions> table{!carry,
                                               !less,
                                               !zero,
                                               !negative,
                                               !normalized,
                                               !extension,
                                               !limit,
                                               !zero && !less,
                                               carry,
                                               less,
                                               zero,
                                               negative,
                                               normalized,
                                               extension,
                                               limit,
                                               zero || less};
  return table.at(input.selector);
}

Registers seed(const ConditionCase input) {
  auto state = initial_state();
  constexpr std::uint64_t accumulator_a = 0x7eabcdef123456;
  constexpr std::uint64_t accumulator_b = 0x813456abcdef12;
  constexpr std::uint64_t register_x = 0x345678fffff9;
  constexpr std::uint64_t register_y = 0x800002001234;
  state.a = accumulator_a;
  state.b = accumulator_b;
  state.x = register_x;
  state.y = register_y;
  state.sr = input.flags | flag_s | mode_rm;
  for (std::size_t index = 0; index < agu_registers; ++index) {
    state.r[index] = cursor + static_cast<Word>(index);
    state.n[index] = offset_n;
  }
  for (std::size_t index = 0; index < stack_entries; ++index) {
    state.stack[index] = long_mask - index;
  }
  return state;
}

void append(Cases &cases, BinaryCase item, std::vector<Word> program) {
  item.name = "condition " + std::to_string((item.initial.sr & (flags_end - 1))) + ", opcode " +
              std::to_string(program.at(0));
  item.before.push_back({.words = std::move(program), .address = start});
  cases.push_back(std::move(item));
}

void expected_call(BinaryCase &item, const Word resume) {
  item.expected.sp = 1;
  item.expected.sc = 1;
  item.expected.stack[1] = (std::uint64_t{resume} << word_shift) | item.initial.sr;
}

struct JumpCase {
  ConditionCase condition;
  unsigned mode = 3;
  bool call = false, short_address = false;
};

void jump(Cases &cases, const JumpCase input) {
  BinaryCase item;
  item.initial = seed(input.condition);
  item.initial.r[2] = cursor;
  item.expected = item.initial;
  Word target = short_target;
  Word words = 1;
  Word opcode = 0;
  if (input.short_address) {
    constexpr Word jump_prefix = 0x0e0000;
    constexpr Word call_prefix = 0x0f0000;
    opcode = (input.call ? call_prefix : jump_prefix) | (input.condition.selector << 12) | target;
  } else {
    constexpr Word jump_prefix = 0x0ac0a0;
    constexpr Word call_prefix = 0x0bc0a0;
    const Word index = input.mode == 6 ? 0U : 2U;
    opcode = (input.call ? call_prefix : jump_prefix) | (input.mode << 11) | (index << 8) |
             input.condition.selector;
    target = cursor;
    auto &updated = item.expected.r[2];
    switch (input.mode) {
      case 0: updated = cursor + 3; break;
      case 1: updated = cursor - 3; break;
      case 2: updated = cursor - 1; break;
      case 3: updated = cursor + 1; break;
      case 4: break;
      case 5: target = cursor - 3; break;
      case 6:
        target = long_target;
        words = 2;
        break;
      case 7:
        target = cursor - 1;
        updated = target;
        break;
      default: throw std::logic_error("Unknown jump addressing mode");
    }
  }
  item.expected.pc = start + words;
  if (expected_condition(input.condition)) {
    if (input.call) { expected_call(item, item.expected.pc); }
    item.expected.pc = target;
  }
  item.progress.estimated_cycles = 4;
  std::vector<Word> program{opcode};
  if (words == 2) { program.push_back(long_target); }
  append(cases, std::move(item), std::move(program));
}

enum class RelativeForm : std::uint8_t { small, large, reg };

struct BranchCase {
  ConditionCase condition;
  RelativeForm form = RelativeForm::small;
  std::int32_t offset = -1;
  bool call = false, unconditional = false;
};

void branch(Cases &cases, const BranchCase input) {
  BinaryCase item;
  item.initial = seed(input.condition);
  const auto raw_offset = static_cast<Word>(input.offset) & word_mask;
  item.initial.r[2] = raw_offset;
  item.expected = item.initial;
  Word opcode = 0;
  Word words = 1;
  item.progress.estimated_cycles = 4;
  switch (input.form) {
    case RelativeForm::small: {
      constexpr Word prefix = 0x050000;
      constexpr Word no_call = 0x400;
      constexpr Word unconditional = 0x800;
      const auto packed = static_cast<Word>(input.offset) & relative_mask;
      opcode = prefix | (input.call ? 0U : no_call) |
               (input.unconditional ? unconditional : input.condition.selector << 12) |
               ((packed / 32) << 6) | (packed % 32);
      break;
    }
    case RelativeForm::large:
    case RelativeForm::reg: {
      constexpr Word large_prefix = 0x0d1000;
      constexpr Word register_prefix = 0x0d1a00;
      constexpr Word no_call = 0x40;
      constexpr Word unconditional = 0x80;
      opcode = (input.form == RelativeForm::large ? large_prefix : register_prefix) |
               (input.call ? 0U : no_call) |
               (input.unconditional ? unconditional : input.condition.selector);
      if (input.form == RelativeForm::large) {
        words = 2;
        // Rev.5 appendix A: BRA=4, BSR=5, Bcc long=5, BScc=4.
        if ((input.unconditional && input.call) || (!input.unconditional && !input.call)) {
          item.progress.estimated_cycles = 5;
        }
      }
      break;
    }
    default: throw std::logic_error("Unknown model branch form");
  }
  item.expected.pc = start + words;
  if (input.unconditional || expected_condition(input.condition)) {
    if (input.call) { expected_call(item, item.expected.pc); }
    item.expected.pc =
        static_cast<Word>(static_cast<std::int64_t>(start) + input.offset) & word_mask;
  }
  std::vector<Word> program{opcode};
  if (words == 2) { program.push_back(raw_offset); }
  append(cases, std::move(item), std::move(program));
}

void transfers(Cases &cases, const ConditionCase input) {
  constexpr Word data_prefix = 0x020000;
  constexpr Word both_prefix = 0x030100;
  constexpr Word address_prefix = 0x020a00;
  constexpr Word data_fields = 0x48;
  constexpr Word address_target = 5;
  constexpr Word address_only_target = 3;
  for (const Word prefix : {data_prefix, both_prefix, address_prefix}) {
    BinaryCase item;
    item.initial = seed(input);
    item.expected = item.initial;
    item.expected.pc = start + 1;
    Word opcode = prefix | (input.selector << 12);
    if (prefix == data_prefix) {
      // Full internal B-to-A transfer, without bus limiting or flag changes.
      if (expected_condition(input)) { item.expected.a = item.initial.b; }
    } else if (prefix == both_prefix) {
      opcode |= data_fields | address_target;
      if (expected_condition(input)) {
        const auto value = signed24(static_cast<Word>(item.initial.x) & word_mask);
        item.expected.b =
            static_cast<std::uint64_t>(value * (std::int64_t{1} << word_shift)) & accumulator_mask;
        item.expected.r[address_target] = item.initial.r[1];
      }
    } else {
      opcode |= address_only_target;
      if (expected_condition(input)) { item.expected.r[address_only_target] = item.initial.r[2]; }
    }
    append(cases, std::move(item), {opcode});
  }
}

void conditional_arithmetic(Cases &cases, const ConditionCase input) {
  constexpr Word clr_a = 0x13;
  constexpr Word prefix = 0x202000;
  constexpr Word update = 0x1000;
  for (const bool update_flags : {false, true}) {
    BinaryCase item;
    item.initial = seed(input);
    item.expected = item.initial;
    item.expected.pc = start + 1;
    if (expected_condition(input)) {
      item.expected.a = 0;
      if (update_flags) {
        item.expected.sr =
            (item.initial.sr & ~(flag_e | flag_n | flag_z | flag_u | flag_v)) | flag_z | flag_u;
      }
    }
    const auto opcode = prefix | (update_flags ? update : 0U) | (input.selector << 8) | clr_a;
    append(cases, std::move(item), {opcode});
  }
}

void conditions(Cases &cases) {
  constexpr unsigned indirect_mode = 3;
  for (Word flags = 0; flags < flags_end; ++flags) {
    for (Word selector = 0; selector < all_conditions; ++selector) {
      const ConditionCase condition{.flags = flags, .selector = selector};
      for (const bool call : {false, true}) {
        jump(cases, {.condition = condition, .mode = indirect_mode, .call = call});
        jump(cases, {.condition = condition, .call = call, .short_address = true});
        for (const auto form : {RelativeForm::small, RelativeForm::large, RelativeForm::reg}) {
          branch(cases, {.condition = condition, .form = form, .call = call});
        }
      }
      transfers(cases, condition);
      conditional_arithmetic(cases, condition);
    }
  }
}

void boundaries(Cases &cases) {
  constexpr std::array<ConditionCase, 2> conditions{
      {{.flags = flag_z, .selector = 10}, {.flags = 0, .selector = 10}}};
  constexpr std::array<unsigned, 7> modes{0, 1, 2, 4, 5, 6, 7};
  for (const auto condition : conditions) {
    for (const auto mode : modes) {
      for (const bool call : {false, true}) {
        jump(cases, {.condition = condition, .mode = mode, .call = call});
      }
    }
    for (const auto form : {RelativeForm::small, RelativeForm::large, RelativeForm::reg}) {
      for (const std::int32_t offset : {-256, -1, 0, 1, 255}) {
        for (const bool call : {false, true}) {
          branch(cases, {.condition = condition, .form = form, .offset = offset, .call = call});
          branch(cases,
                 {.condition = condition,
                  .form = form,
                  .offset = offset,
                  .call = call,
                  .unconditional = true});
        }
      }
    }
  }
  constexpr ConditionCase equal{.flags = flag_z, .selector = 10};
  constexpr std::array<std::int32_t, 3> large_offsets{-0x800000, -257, 0x7fffff};
  for (const auto form : {RelativeForm::large, RelativeForm::reg}) {
    for (const auto offset : large_offsets) {
      for (const bool call : {false, true}) {
        branch(cases, {.condition = equal, .form = form, .offset = offset, .call = call});
      }
    }
  }
}

struct TransferCase {
  Word source = 0;
  bool destination_b = false, taken = false;
};

void transfer_source(Cases &cases, const TransferCase input) {
  constexpr Word prefix = 0x02a000; // EQ, internal data transfer.
  constexpr std::array<Word, 4> input_words{0xfffff9, 0x001234, 0x345678, 0x800002};
  BinaryCase item;
  item.initial = seed({.flags = input.taken ? flag_z : 0U});
  item.expected = item.initial;
  item.expected.pc = start + 1;
  if (input.taken) {
    auto value = input.destination_b ? item.initial.a : item.initial.b;
    if (input.source != 0) {
      const auto full =
          signed24(input_words.at(input.source - 4)) * (std::int64_t{1} << word_shift);
      value = static_cast<std::uint64_t>(full) & accumulator_mask;
    }
    (input.destination_b ? item.expected.b : item.expected.a) = value;
  }
  append(cases, std::move(item), {prefix | (input.source << 4) | (input.destination_b ? 8U : 0U)});
}

void transfer_sources(Cases &cases) {
  for (const Word source : {0U, 4U, 5U, 6U, 7U}) {
    for (const bool destination_b : {false, true}) {
      for (const bool taken : {false, true}) {
        transfer_source(cases, {.source = source, .destination_b = destination_b, .taken = taken});
      }
    }
  }
}

struct ImmediateCase {
  std::uint64_t accumulator = 0;
  Word value = 0, kind = 0, mode = 0;
  bool long_value = false, destination_b = false, carry = false;
};

Word arithmetic_flags(const ImmediateCase input, const std::int64_t full) {
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto value = signed56(raw);
  const unsigned position =
      47U + (input.mode == mode_down ? 1U : 0U) - (input.mode == mode_up ? 1U : 0U);
  const auto boundary = std::int64_t{1} << position;
  Word flags = value == 0 ? flag_z : 0;
  if (value < 0) { flags |= flag_n; }
  if (value < -boundary || value >= boundary) { flags |= flag_e; }
  const auto divider = static_cast<std::uint64_t>(boundary);
  if ((raw / divider) % 2 == (raw / (divider / 2)) % 2) { flags |= flag_u; }
  constexpr auto minimum = -static_cast<std::int64_t>(sign56);
  constexpr auto maximum = static_cast<std::int64_t>(sign56 - 1);
  if (full < minimum || full > maximum) { flags |= flag_v | flag_l; }
  return flags;
}

void immediate_arithmetic(BinaryCase &item, const ImmediateCase input) {
  const auto operand = signed24(input.value) * (std::int64_t{1} << word_shift);
  const auto unsigned_operand = static_cast<std::uint64_t>(operand) & accumulator_mask;
  const bool subtract = input.kind != 0;
  const auto full = signed56(input.accumulator) + (subtract ? -operand : operand);
  const bool carry = subtract ? input.accumulator < unsigned_operand
                              : input.accumulator + unsigned_operand > accumulator_mask;
  if (input.kind != 5) {
    (input.destination_b ? item.expected.b : item.expected.a) =
        static_cast<std::uint64_t>(full) & accumulator_mask;
  }
  item.expected.sr = (item.initial.sr & ~(flag_e | flag_u | flag_n | flag_z | flag_v | flag_c)) |
                     arithmetic_flags(input, full) | (carry ? flag_c : 0);
}

void immediate_logical(BinaryCase &item, const ImmediateCase input) {
  const auto word = static_cast<Word>((input.accumulator >> word_shift) & word_mask);
  Word result = 0;
  switch (input.kind) {
    case 2: result = word | input.value; break;
    case 3: result = word ^ input.value; break;
    case 6: result = word & input.value; break;
    default: throw std::logic_error("Unknown immediate logical model operation");
  }
  constexpr auto main_mask = std::uint64_t{word_mask} << word_shift;
  (input.destination_b ? item.expected.b : item.expected.a) =
      (input.accumulator & ~main_mask) | (std::uint64_t{result} << word_shift);
  item.expected.sr = (item.initial.sr & ~(flag_n | flag_z | flag_v)) | (result == 0 ? flag_z : 0) |
                     (result >= sign24 ? flag_n : 0);
}

void immediate(Cases &cases, const ImmediateCase input) {
  BinaryCase item;
  item.initial = seed({});
  item.initial.sr = mode_rm | input.mode | flag_e | flag_u | flag_z | flag_n | flag_v | flag_s |
                    (input.carry ? flag_c | flag_l : 0);
  (input.destination_b ? item.initial.b : item.initial.a) = input.accumulator;
  item.expected = item.initial;
  item.expected.pc = start + (input.long_value ? 2U : 1U);
  if (input.kind == 0 || input.kind == 4 || input.kind == 5) {
    immediate_arithmetic(item, input);
  } else {
    immediate_logical(item, input);
  }
  constexpr Word short_prefix = 0x014080;
  constexpr Word long_prefix = 0x0140c0;
  const Word fields = (input.destination_b ? 8U : 0U) | input.kind;
  const Word opcode =
      input.long_value ? long_prefix | fields : short_prefix | (input.value << 8) | fields;
  std::vector<Word> program{opcode};
  if (input.long_value) { program.push_back(input.value); }
  item.progress.estimated_cycles = input.long_value ? 2 : 1;
  append(cases, std::move(item), std::move(program));
}

void immediate_variants(Cases &cases, ImmediateCase input) {
  constexpr std::array<Word, 5> short_values{0, 1, 31, 32, 63};
  constexpr std::array<Word, 6> long_values{0, 1, sign24 - 1, sign24, word_mask, 0x555555};
  constexpr std::array<std::uint64_t, 9> accumulators{0,
                                                      1,
                                                      sign56 - 1,
                                                      sign56,
                                                      accumulator_mask,
                                                      0x123456abcdef12,
                                                      0xab654321123456,
                                                      std::uint64_t{1} << 47,
                                                      long_mask};
  const auto values =
      input.long_value ? std::span<const Word>{long_values} : std::span<const Word>{short_values};
  for (const auto accumulator : accumulators) {
    for (const auto value : values) {
      for (const bool destination_b : {false, true}) {
        for (const bool carry : {false, true}) {
          input.accumulator = accumulator;
          input.value = value;
          input.destination_b = destination_b;
          input.carry = carry;
          immediate(cases, input);
        }
      }
    }
  }
}

void immediates(Cases &cases) {
  for (const Word kind : {0U, 2U, 3U, 4U, 5U, 6U}) {
    for (const Word mode : {Word{0}, mode_down, mode_up}) {
      for (const bool long_value : {false, true}) {
        immediate_variants(cases, {.kind = kind, .mode = mode, .long_value = long_value});
      }
    }
  }
}

void call_returns(Cases &cases) {
  constexpr Word conditional_call = 0x0bf0a2;
  constexpr Word relative_call = 0x0d1080;
  constexpr Word return_subroutine = 0x0c;
  for (const Word opcode : {conditional_call, relative_call}) {
    BinaryCase item;
    item.initial = seed({});
    item.expected = item.initial;
    item.expected.pc = start + 2;
    expected_call(item, item.expected.pc);
    item.expected.sp = 0;
    item.expected.sc = 0;
    item.budget = 2;
    item.dispatches = 2;
    item.progress = {.instructions = 2, .estimated_cycles = opcode == conditional_call ? 7U : 8U};
    const Word target = opcode == conditional_call ? start + 4 : 4;
    append(cases, std::move(item), {opcode, target, 0, 0, return_subroutine});
  }
}

void conditional_repeats(Cases &cases) {
  constexpr Word repeat_two = 0x0602a0;
  constexpr Word conditional_asr = 0x202022;
  constexpr Word update = 0x1000;
  constexpr Word old_lc = 0x17;
  for (const bool update_flags : {false, true}) {
    BinaryCase item;
    item.initial = seed({});
    item.initial.a = 1;
    item.initial.lc = old_lc;
    item.expected = item.initial;
    item.expected.a = 0;
    item.expected.pc = start + 2;
    if (update_flags) { item.expected.sr |= flag_c | flag_z | flag_u; }
    item.budget = 3;
    item.dispatches = 3;
    item.progress = {.instructions = 3, .estimated_cycles = 7};
    const auto opcode = conditional_asr | (update_flags ? update : 0U);
    append(cases, std::move(item), {repeat_two, opcode});
  }
  BinaryCase item;
  item.initial = seed({.flags = flag_c});
  item.initial.b = 0;
  item.initial.lc = old_lc;
  item.expected = item.initial;
  item.expected.b = std::uint64_t{3} << word_shift;
  item.expected.pc = start + 2;
  item.expected.sr = (item.initial.sr & ~flag_c) | flag_u;
  item.budget = 4;
  item.dispatches = 4;
  item.progress = {.instructions = 4, .estimated_cycles = 8};
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_one_b = 0x014188;
  append(cases, std::move(item), {repeat_three, add_one_b});
}

void rejected_control(Cases &cases) {
  constexpr Word repeat_two = 0x0602a0;
  constexpr Word conditional_jump = 0x0e2000 | start;
  BinaryCase repeated;
  repeated.initial = seed({.flags = flag_z}); // NE is false, but REP still prohibits a jump.
  repeated.budget = 2;
  repeated.dispatches = 1;
  repeated.progress = {.instructions = 1, .estimated_cycles = 5};
  repeated.interruptible = false;
  repeated.fault = {.kind = FaultKind::invalid_instruction,
                    .space = Space::program,
                    .address = start + 1,
                    .value = conditional_jump,
                    .pc = start + 1};
  append(cases, std::move(repeated), {repeat_two, conditional_jump});
  constexpr Word immediate_ea_jump = 0x0af4a2;
  BinaryCase address;
  address.initial = seed({.flags = flag_z});
  address.dispatches = 0;
  address.progress = {};
  address.interruptible = false;
  address.fault = {.kind = FaultKind::invalid_instruction,
                   .space = Space::program,
                   .address = start,
                   .value = immediate_ea_jump,
                   .pc = start};
  append(cases, std::move(address), {immediate_ea_jump, long_target});
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_conditions OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::conditions(cases);
    dsp56300::test::boundaries(cases);
    dsp56300::test::transfer_sources(cases);
    dsp56300::test::immediates(cases);
    dsp56300::test::call_returns(cases);
    dsp56300::test::conditional_repeats(cases);
    dsp56300::test::rejected_control(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent condition cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
