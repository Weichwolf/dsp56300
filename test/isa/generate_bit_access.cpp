// SPDX-License-Identifier: GPL-3.0-only
// Independent bit, raw-register and control-flow expectations; no DSP execution.
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
using namespace register_model;

struct FixtureSets {
  Cases access, branches;
};

constexpr Word action_family = 0x0a0000, change_family = 0x0b0000;
constexpr Word register_field = 0xc040, effective_field = 0x4000;
constexpr Word jump_family = 0x0a0000, relative_family = 0x0c8000;
constexpr Word relative_register = 0x4080, jump_register = 0xc000;
constexpr Word call_field = 0x10000, set_field = 0x20;
constexpr Word absolute_target = 0xfedcba, relative_target = 0xffff00;
constexpr Word short_location = 12;
constexpr Word low_io_begin = 0xffff80;
constexpr std::array actions{BitAction::clear, BitAction::set, BitAction::change, BitAction::test};
constexpr std::array scales{Word{0}, mode_down, mode_up};

Word action_word(const BitAction action) {
  switch (action) {
    case BitAction::clear: return action_family;
    case BitAction::set: return action_family | set_field;
    case BitAction::change: return change_family;
    case BitAction::test: return change_family | set_field;
    default: throw std::logic_error("Invalid model bit action");
  }
}

Word changed(const Word value, const BitAction action, const unsigned bit) {
  const auto unit = Word{1} << bit;
  const bool previous = (value / unit) % 2 != 0;
  if ((action == BitAction::clear && previous) || (action == BitAction::change && previous)) {
    return value - unit;
  }
  if ((action == BitAction::set && !previous) || (action == BitAction::change && !previous)) {
    return value + unit;
  }
  return value;
}

Registers initial(const Word mode, const bool zero) {
  auto state = seed(mode);
  if (zero) {
    state.sr = mode;
    state.a = 0;
    state.b = 0;
    state.x = 0;
    state.y = 0;
    state.n.fill(0);
    state.stack.fill(0);
  }
  return state;
}

void append(Cases &cases, BinaryCase item, std::vector<Word> program) {
  item.name = "independent bit access opcode " + std::to_string(program.at(0));
  item.before.push_back({.words = std::move(program), .address = start});
  cases.push_back(std::move(item));
}

struct RegisterAccess {
  Registers state{};
  unsigned reg = 4, bit = 0;
  BitAction action = BitAction::test;
};

void register_access(Cases &cases, const RegisterAccess &input) {
  BinaryCase item;
  item.initial = input.state;
  item.expected = item.initial;
  const bool stack = input.reg == reg_ssh && input.action != BitAction::test;
  const auto old =
      stack
          ? static_cast<Word>(item.expected.stack.at(item.expected.sp % stack_entries) >> word_bits)
          : read(item.expected, input.reg);
  if (input.action != BitAction::test) {
    const auto value = changed(old, input.action, input.bit);
    if (stack) {
      auto &frame = item.expected.stack.at(item.expected.sp % stack_entries);
      frame = (std::uint64_t{value} << word_bits) | (frame % (std::uint64_t{1} << word_bits));
    } else {
      write(item.expected, input.reg, value);
    }
  }
  if (input.reg != reg_sr || input.action == BitAction::test) {
    item.expected.sr = (item.expected.sr & ~flag_c) | (((old >> input.bit) & 1) != 0 ? flag_c : 0);
  }
  item.expected.pc = start + 1;
  item.progress.estimated_cycles = 2;
  append(cases,
         std::move(item),
         {action_word(input.action) | register_field | (input.reg << 8) | input.bit});
}

void register_variants(Cases &cases, const Registers &state, const unsigned reg) {
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    for (const auto action : actions) {
      register_access(cases, {.state = state, .reg = reg, .bit = bit, .action = action});
    }
  }
}

void register_cases(Cases &cases) {
  for (const auto mode : scales) {
    for (const bool zero : {false, true}) {
      const auto state = initial(mode, zero);
      for (unsigned reg = 4; reg < 64; ++reg) {
        if (valid(reg)) { register_variants(cases, state, reg); }
      }
    }
  }
}

struct Flow {
  bool relative = false, call = false, set = false;
};

constexpr std::array<Flow, 8> flows{{{},
                                     {.set = true},
                                     {.call = true},
                                     {.call = true, .set = true},
                                     {.relative = true},
                                     {.relative = true, .set = true},
                                     {.relative = true, .call = true},
                                     {.relative = true, .call = true, .set = true}}};

struct FlowResult {
  Flow flow{};
  Word source = 0, extension = 0;
  unsigned bit = 0;
};

void flow_result(BinaryCase &item, const FlowResult input) {
  const auto flow = input.flow;
  item.expected.pc = start + 2;
  item.progress.estimated_cycles = flow.relative ? 5 : 4;
  if (((input.source >> input.bit) & 1) != (flow.set ? 1U : 0U)) { return; }
  if (flow.call) {
    item.expected.sp = (item.expected.sp + 1) % 64;
    ++item.expected.sc;
    item.expected.stack.at(item.expected.sp % stack_entries) =
        (std::uint64_t{start + 2} << word_bits) | item.expected.sr;
  }
  item.expected.pc = input.extension;
  if (flow.relative) {
    item.expected.pc =
        static_cast<Word>(static_cast<std::int64_t>(start) + signed24(input.extension)) & word_mask;
  }
}

Word flow_word(const Flow flow) {
  return (flow.relative ? relative_family : jump_family) | (flow.call ? call_field : 0U) |
         (flow.set ? set_field : 0U);
}

void register_flow(Cases &cases,
                   const RegisterAccess &input,
                   const Flow flow,
                   const Word extension) {
  BinaryCase item;
  item.initial = input.state;
  item.expected = item.initial;
  const auto value = read(item.expected, input.reg);
  flow_result(item, {.flow = flow, .source = value, .extension = extension, .bit = input.bit});
  const auto field = flow.relative ? relative_register : jump_register;
  append(
      cases, std::move(item), {flow_word(flow) | field | (input.reg << 8) | input.bit, extension});
}

void flow_register_variants(Cases &cases, const Registers &state, const unsigned reg) {
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    for (const auto flow : flows) {
      register_flow(cases,
                    {.state = state, .reg = reg, .bit = bit},
                    flow,
                    flow.relative ? relative_target : absolute_target);
    }
  }
}

void register_flow_cases(Cases &cases) {
  for (const auto mode : scales) {
    for (const bool zero : {false, true}) {
      const auto state = initial(mode, zero);
      for (unsigned reg = 4; reg < 64; ++reg) {
        if (valid(reg)) { flow_register_variants(cases, state, reg); }
      }
    }
  }
}

struct MemorySource {
  Word mode = 4, value = 0;
  Space space = Space::x;
  bool short_address = false;
};

Word memory_location(BinaryCase &item, const MemorySource source) {
  item.initial.r[7] = cursor;
  item.expected.r[7] = cursor;
  if (source.short_address) { return short_location; }
  auto &reg = item.expected.r[7];
  switch (source.mode) {
    case 0: reg -= 3; return cursor;
    case 1: reg += 3; return cursor;
    case 2: --reg; return cursor;
    case 3: ++reg; return cursor;
    case 4: return cursor;
    case 5: return cursor + 3;
    case 6: return cursor;
    case 7: --reg; return cursor - 1;
    default: throw std::logic_error("Invalid model bit addressing");
  }
}

Word memory_field(const MemorySource source) {
  if (source.short_address) { return short_location << 8; }
  const Word index = source.mode == 6 ? 0U : 7U;
  return effective_field | (source.mode << 11) | (index << 8);
}

void memory_access(Cases &cases, const MemorySource source, const RegisterAccess &input) {
  BinaryCase item;
  item.initial = seed(0);
  item.expected = item.initial;
  const auto location = memory_location(item, source);
  item.expected.sr =
      (item.expected.sr & ~flag_c) | (((source.value >> input.bit) & 1) != 0 ? flag_c : 0);
  item.expected.pc = start + 1;
  item.progress.estimated_cycles = 2;
  item.before.push_back({.words = {source.value}, .address = location, .space = source.space});
  if (input.action != BitAction::test) {
    item.after.push_back({.words = {changed(source.value, input.action, input.bit)},
                          .address = location,
                          .space = source.space});
  }
  const Word opcode = action_word(input.action) | memory_field(source) |
                      (source.space == Space::y ? 0x40U : 0U) | input.bit;
  std::vector<Word> program{opcode};
  if (!source.short_address && (source.mode == 5 || source.mode == 6 || source.mode == 7)) {
    ++item.progress.estimated_cycles;
  }
  if (!source.short_address && source.mode == 6) {
    program.push_back(cursor);
    ++item.expected.pc;
  }
  append(cases, std::move(item), std::move(program));
}

void memory_flow(Cases &cases, const MemorySource source, const unsigned bit, const Flow flow) {
  BinaryCase item;
  item.initial = seed(0);
  item.expected = item.initial;
  const auto location = memory_location(item, source);
  const Word extension = flow.relative ? relative_target : absolute_target;
  flow_result(item, {.flow = flow, .source = source.value, .extension = extension, .bit = bit});
  Word field = memory_field(source);
  if (flow.relative) {
    field ^= source.short_address ? 0x80U : effective_field;
  } else {
    field |= 0x80U;
  }
  const Word opcode = flow_word(flow) | field | (source.space == Space::y ? 0x40U : 0U) | bit;
  item.before.push_back({.words = {source.value}, .address = location, .space = source.space});
  if (!source.short_address && (source.mode == 5 || source.mode == 7)) {
    ++item.progress.estimated_cycles;
  }
  append(cases, std::move(item), {opcode, extension});
}

void memory_variants(FixtureSets &cases, const MemorySource source) {
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    for (const auto action : actions) {
      memory_access(cases.access, source, {.bit = bit, .action = action});
    }
  }
  if (!source.short_address && source.mode == 6) { return; }
  for (const unsigned bit : {0U, 7U, word_bits - 1}) {
    for (const auto flow : flows) { memory_flow(cases.branches, source, bit, flow); }
  }
}

void memory_cases(FixtureSets &cases) {
  for (const auto space : {Space::x, Space::y}) {
    for (const Word value : {Word{0}, word_mask}) {
      memory_variants(cases, {.value = value, .space = space, .short_address = true});
      for (Word mode = 0; mode < 8; ++mode) {
        memory_variants(cases, {.mode = mode, .value = value, .space = space});
      }
    }
  }
}

void wrap_cases(Cases &cases) {
  constexpr std::array offsets{Word{0}, Word{1}, sign24 - 1, sign24, word_mask, word_mask - start};
  for (const auto extension : offsets) {
    for (const auto flow : flows) {
      for (const bool zero : {false, true}) {
        register_flow(cases, {.state = initial(0, zero), .bit = 1}, flow, extension);
      }
    }
  }
}

struct FailedAccess {
  std::vector<Word> program;
  Fault fault{};
  Word pointer = cursor, stack_count = 2;
  bool opcode_value = true;
};

void failed_access(Cases &cases, FailedAccess input) {
  BinaryCase item;
  item.initial = seed(0);
  item.initial.r[0] = input.pointer;
  item.initial.sc = input.stack_count;
  item.initial.sp = input.stack_count;
  item.expected = item.initial;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = input.fault;
  item.fault.pc = start;
  if (input.opcode_value) {
    item.fault.kind = FaultKind::invalid_instruction;
    item.fault.space = Space::program;
    item.fault.address = start;
    item.fault.value = input.program.at(0);
  }
  append(cases, std::move(item), std::move(input.program));
}

void failed_cases(FixtureSets &cases) {
  constexpr Word bad_bit_register = 0x0ac458;
  constexpr Word reserved_register = 0x0ac040;
  constexpr Word immediate_source = 0x0a7400;
  constexpr Word branch_long_source = 0x0cb000;
  // This shared opcode is a long displaced MOVE with reserved register zero.
  constexpr Word reserved_displacement_register = 0x0a7080;
  constexpr Word out_of_span = 0x0a6000;
  constexpr Word test_ssh = 0x0bfc60;
  constexpr Word call_clear_x0 = 0x0bc400;
  failed_access(cases.access, {.program = {bad_bit_register}});
  failed_access(cases.access, {.program = {immediate_source, cursor}});
  failed_access(cases.access,
                {.program = {reserved_register},
                 .fault = {.kind = FaultKind::invalid_instruction, .address = start},
                 .opcode_value = false});
  failed_access(
      cases.access,
      {.program = {out_of_span},
       .fault = {.kind = FaultKind::memory_read, .space = Space::x, .address = memory_words},
       .pointer = memory_words,
       .opcode_value = false});
  failed_access(cases.access,
                {.program = {test_ssh},
                 .fault = {.kind = FaultKind::stack, .address = start},
                 .stack_count = 0,
                 .opcode_value = false});
  failed_access(cases.branches, {.program = {branch_long_source, cursor}});
  failed_access(cases.branches, {.program = {reserved_displacement_register, cursor}});
  failed_access(cases.branches,
                {.program = {call_clear_x0, cursor},
                 .fault = {.kind = FaultKind::stack, .address = start, .value = 15},
                 .stack_count = 15,
                 .opcode_value = false});
  constexpr Word low_port_set = 0x010060;
  failed_access(cases.access,
                {.program = {low_port_set},
                 .fault = {.kind = FaultKind::io_bit, .space = Space::y, .address = low_io_begin},
                 .opcode_value = false});
}

void repeated_cases(Cases &cases) {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word change_x0 = 0x0bc440;
  BinaryCase item;
  item.initial = seed(0);
  item.expected = item.initial;
  item.expected.x ^= 1;
  item.expected.sr &= ~flag_c;
  item.expected.pc = start + 2;
  item.budget = 4;
  item.dispatches = 4;
  item.progress = {.instructions = 4, .estimated_cycles = 11};
  append(cases, std::move(item), {repeat_three, change_x0});
  for (const auto action : {BitAction::clear, BitAction::set, BitAction::change}) {
    auto state = seed(0);
    state.sp = 0;
    state.sc = 0;
    register_access(cases,
                    {.state = state, .reg = reg_ssh, .bit = word_bits - 1, .action = action});
  }
}

void loop_case(Cases &cases) {
  constexpr Word loop_cycles = 17;
  constexpr Word do_three = 0x060380;
  constexpr Word set_x0 = 0x0ac461;
  constexpr Word change_x0 = 0x0bc440;
  BinaryCase item;
  item.initial = seed(0);
  item.expected = item.initial;
  item.expected.x = (item.expected.x | 2) ^ 1;
  item.expected.sr &= ~flag_c;
  item.expected.pc = start + 4;
  item.expected.stack[3] = (std::uint64_t{item.initial.la} << word_bits) | item.initial.lc;
  item.expected.stack[4] = (std::uint64_t{start + 2} << word_bits) | item.initial.sr;
  item.budget = 7;
  item.dispatches = 7;
  item.progress = {.instructions = 7, .estimated_cycles = loop_cycles};
  append(cases, std::move(item), {do_three, start + 3, set_x0, change_x0});
}

void rejected_repeat(Cases &cases) {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word jump_bit = 0x0ac400;
  BinaryCase item;
  item.initial = seed(0);
  item.expected = item.initial;
  item.budget = 2;
  item.dispatches = 1;
  item.progress = {.instructions = 1, .estimated_cycles = 5};
  item.interruptible = false;
  item.fault = {.kind = FaultKind::invalid_instruction,
                .address = start + 1,
                .value = jump_bit,
                .pc = start + 1};
  append(cases, std::move(item), {repeat_three, jump_bit, cursor});
}

void write_cases(const char *const path, Cases cases) {
  const auto count = cases.size();
  write_fixture(path, std::move(cases));
  std::cout << count << " independent bit cases written to " << path << '\n';
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_bit_access ACCESS.d56 BRANCHES.d56");
    }
    dsp56300::test::FixtureSets cases;
    dsp56300::test::register_cases(cases.access);
    dsp56300::test::register_flow_cases(cases.branches);
    dsp56300::test::memory_cases(cases);
    dsp56300::test::wrap_cases(cases.branches);
    dsp56300::test::failed_cases(cases);
    dsp56300::test::repeated_cases(cases.access);
    dsp56300::test::loop_case(cases.access);
    dsp56300::test::rejected_repeat(cases.branches);
    dsp56300::test::write_cases(argv[1], std::move(cases.access));
    dsp56300::test::write_cases(argv[2], std::move(cases.branches));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
