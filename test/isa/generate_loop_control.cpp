// SPDX-License-Identifier: GPL-3.0-only
// Independent raw stack/count/condition models; no DSP core linkage or execution.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
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
enum class Kind : std::uint8_t { absolute, relative, repeat, forever_absolute, forever_relative };
enum class Operand : std::uint8_t { immediate, reg, short_memory, indirect };
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, sc_mode = Word{1} << 13;
constexpr Word status_mask = lf | fv, sp_range = 64;
constexpr Word do_immediate = 0x060080, dor_bit = 0x10, rep_immediate = 0x0600a0;
constexpr Word do_register = 0x06c000, rep_register = 0x06c020;
constexpr Word do_short = 0x060000, rep_short = 0x060020;
constexpr Word do_indirect = 0x064000, rep_indirect = 0x064020;
constexpr Word do_forever = 0x000203, dor_forever = 0x000202;
constexpr Word brk_base = 0x000210, enddo = 0x00008c, mac_a = 0x2000d2;
constexpr Word short_address = 0x3f, data_address = 0x180, last = start + 4;
constexpr Word repeat_zero = 65536, maximum_budget = 20;
constexpr Word saved_la = 0x80, saved_lc = 0x123456;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array pointers{0U, 14U, 30U, 62U};

struct Input {
  Kind kind = Kind::absolute;
  Operand operand = Operand::immediate;
  Word source = 3, mode = 0, flags = flag_c, pointer = 0, short_location = short_address;
  unsigned address_mode = 4, index = 0;
  Space space = Space::x;
};

bool forever(const Kind kind) {
  return kind == Kind::forever_absolute || kind == Kind::forever_relative;
}

bool relative(const Kind kind) {
  return kind == Kind::relative || kind == Kind::forever_relative;
}

Word opcode(const Input input) {
  if (forever(input.kind)) { return relative(input.kind) ? dor_forever : do_forever; }
  const bool rep = input.kind == Kind::repeat;
  const Word displacement = relative(input.kind) ? dor_bit : 0U;
  const Word space = input.space == Space::y ? Word{1} << 6 : 0U;
  switch (input.operand) {
    case Operand::immediate:
      return (rep ? rep_immediate : do_immediate) | displacement | ((input.source % 256) << 8) |
             (input.source / 256);
    case Operand::reg:
      return (rep ? rep_register : do_register) | displacement | (input.source << 8);
    case Operand::short_memory:
      return (rep ? rep_short : do_short) | displacement | (input.short_location << 8) | space;
    case Operand::indirect:
      return (rep ? rep_indirect : do_indirect) | displacement | (input.address_mode << 11) |
             (input.index << 8) | space;
    default: throw std::runtime_error("Unknown loop count operand");
  }
}

void stack_push(Registers &state, const std::uint64_t frame) {
  state.sp = (state.sp + 1) % sp_range;
  ++state.sc;
  state.stack.at(state.sp % stack_entries) = frame;
}

void stack_restore(Registers &state) {
  const auto flags = static_cast<Word>(state.stack.at(state.sp % stack_entries)) & status_mask;
  state.sp = (state.sp + sp_range - 1) % sp_range;
  const auto outer = state.stack.at(state.sp % stack_entries);
  state.sp = (state.sp + sp_range - 1) % sp_range;
  state.sc -= 2;
  state.la = static_cast<Word>(outer >> word_bits);
  state.lc = static_cast<Word>(outer) & word_mask;
  state.sr = (state.sr & ~status_mask) | flags;
}

Word address(Registers &state, const Input input) {
  if (input.operand != Operand::indirect) { return input.short_location; }
  const auto old = state.r.at(input.index);
  const auto increment = signed24(state.n.at(input.index));
  std::int64_t next = old;
  switch (input.address_mode) {
    case 0: next -= increment; break;
    case 1: next += increment; break;
    case 2: --next; break;
    case 3: ++next; break;
    case 4: return old;
    case 5: next += increment; break;
    case 7: --next; break;
    default: throw std::runtime_error("No extended count address is encoded");
  }
  constexpr auto span = static_cast<std::int64_t>(address_words);
  const auto wrapped = static_cast<Word>((next + span) % span);
  if (input.address_mode != 5) { state.r.at(input.index) = wrapped; }
  return input.address_mode == 5 || input.address_mode == 7 ? wrapped : old;
}

Word count(Registers &state, const Input input) {
  if (input.operand == Operand::immediate) { return input.source; }
  if (input.operand == Operand::reg) { return register_model::read(state, input.source); }
  static_cast<void>(address(state, input));
  return input.source;
}

BinaryCase entry(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.mode | input.flags);
  item.initial.sr = input.mode | input.flags;
  item.initial.sp = input.pointer;
  item.initial.sc = 2;
  item.initial.lc = 7;
  item.initial.r.at(input.index) = data_address;
  item.initial.n.at(input.index) = 3;
  item.expected = item.initial;
  const bool rep = input.kind == Kind::repeat;
  const Word words = rep ? 1U : 2U;
  std::vector<Word> program{opcode(input)};
  if (!rep) { program.push_back(relative(input.kind) ? last - start : last); }
  program.insert(program.end(), {mac_a, 0, 0});
  item.before = {{.words = std::move(program), .address = start}};
  if (input.operand == Operand::short_memory || input.operand == Operand::indirect) {
    auto state = item.initial;
    const auto location = address(state, input);
    item.before.push_back({.words = {input.source}, .address = location, .space = input.space});
  }
  auto &state = item.expected;
  if (!rep) { stack_push(state, (std::uint64_t{state.la} << word_bits) | state.lc); }
  const auto loaded = forever(input.kind) ? state.lc : count(state, input);
  if (!rep && loaded == 0) {
    throw std::runtime_error("Zero-DO scratch-stack needs qualification");
  }
  if (rep) {
    state.lc = loaded;
    item.interruptible = false;
  } else {
    state.lc = loaded;
    stack_push(state, (std::uint64_t{start + words} << word_bits) | state.sr);
    state.la = last;
    state.sr = (state.sr & ~fv) | lf | (forever(input.kind) ? fv : 0U);
  }
  state.pc = start + words;
  item.progress.estimated_cycles = forever(input.kind) ? 4U : 5U;
  if (input.operand == Operand::indirect && (input.address_mode == 5 || input.address_mode == 7)) {
    ++item.progress.estimated_cycles;
  }
  item.name = "Loop entry kind=" + std::to_string(static_cast<unsigned>(input.kind)) +
              "; operand=" + std::to_string(static_cast<unsigned>(input.operand)) +
              "; selector=" + std::to_string(input.source);
  return item;
}

void entry_statuses(Cases &cases, Input input) {
  for (const auto mode : modes) {
    input.mode = mode;
    for (const Word flags : {flag_c, status_byte | status_mask}) {
      input.flags = flags;
      for (const auto pointer : pointers) {
        input.pointer = pointer;
        cases.push_back(entry(input));
      }
    }
  }
}

void register_entries(Cases &cases) {
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (Word source = 0; source < 64; ++source) {
      if (!register_model::valid(source) ||
          (source == register_model::reg_ssh && kind != Kind::repeat)) {
        continue;
      }
      entry_statuses(cases, {.kind = kind, .operand = Operand::reg, .source = source});
    }
  }
}

void memory_entries(Cases &cases) {
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (const auto space : {Space::x, Space::y}) {
      entry_statuses(cases, {.kind = kind, .operand = Operand::short_memory, .space = space});
      for (Word location = 0; location < 64; ++location) {
        cases.push_back(entry({.kind = kind,
                               .operand = Operand::short_memory,
                               .short_location = location,
                               .space = space}));
      }
      for (unsigned index = 0; index < agu_registers; ++index) {
        for (const unsigned mode : {0U, 1U, 2U, 3U, 4U, 5U, 7U}) {
          cases.push_back(entry({.kind = kind,
                                 .operand = Operand::indirect,
                                 .address_mode = mode,
                                 .index = index,
                                 .space = space}));
        }
      }
    }
  }
}

void repeated(Cases &cases, const Input input) {
  auto item = entry(input);
  const auto loaded = item.expected.lc;
  for (const Word executed : {0U, 1U, 2U, 3U, 7U}) {
    const auto iterations = executed < loaded ? executed : loaded;
    auto result = item;
    for (Word index = 0; index < iterations; ++index) {
      const auto previous = result.expected;
      sequence_model::alu(result.expected, previous, mac_a);
    }
    const bool finished = iterations == loaded;
    result.expected.lc = finished ? item.initial.lc : loaded - iterations;
    result.expected.pc = start + (finished ? 2U : 1U);
    result.interruptible = finished;
    result.budget = result.dispatches = 1 + iterations;
    result.progress = {.instructions = result.budget, .estimated_cycles = 5 + iterations};
    cases.push_back(std::move(result));
  }
}

void repeat_sequences(Cases &cases) {
  for (const auto operand :
       {Operand::immediate, Operand::reg, Operand::short_memory, Operand::indirect}) {
    for (const Word source : {1U, 2U, 3U, 7U}) {
      Input input{.kind = Kind::repeat, .operand = operand, .source = source};
      if (operand == Operand::reg) { input.source = 4; }
      if (operand == Operand::reg) {
        auto item = entry(input);
        item.name += "; count source entry only";
        cases.push_back(std::move(item));
      } else {
        repeated(cases, input);
      }
    }
  }
  for (const auto operand : {Operand::immediate, Operand::short_memory, Operand::indirect}) {
    auto item = entry({.kind = Kind::repeat, .operand = operand, .source = 0});
    for (Word index = 0; index < repeat_zero; ++index) {
      const auto previous = item.expected;
      sequence_model::alu(item.expected, previous, mac_a);
    }
    item.expected.lc = item.initial.lc;
    item.expected.pc = start + 2;
    item.budget = item.dispatches = repeat_zero + 1;
    item.progress = {.instructions = item.budget, .estimated_cycles = repeat_zero + 5};
    item.interruptible = true;
    cases.push_back(std::move(item));
  }
}

void zero_repeat_counter_reads(Cases &cases) {
  constexpr Word read_lc_x0 = 0x0444bf;
  for (const auto operand :
       {Operand::immediate, Operand::reg, Operand::short_memory, Operand::indirect}) {
    for (const Word budget : {1U, 2U, 3U, 7U, repeat_zero, repeat_zero + 1}) {
      auto item = entry(
          {.kind = Kind::repeat, .operand = operand, .source = operand == Operand::reg ? 4U : 0U});
      item.before.front().words[1] = read_lc_x0;
      item.initial.x &= ~std::uint64_t{word_mask};
      item.expected = item.initial;
      const auto iterations = budget - 1;
      if (iterations != 0) {
        const Word observed_lc = iterations == 1 ? 0U : repeat_zero - iterations + 1;
        item.expected.x |= observed_lc;
      }
      const bool completed = iterations == repeat_zero;
      item.expected.lc = iterations == 0 ? 0U : repeat_zero - iterations;
      if (completed) { item.expected.lc = item.initial.lc; }
      item.expected.pc = start + (completed ? 2U : 1U);
      item.interruptible = completed;
      item.budget = item.dispatches = budget;
      item.progress = {.instructions = budget, .estimated_cycles = budget + 4};
      item.name = "REP zero: LC initially zero, first/subsequent body reads, 65536 iterations, "
                  "captured count";
      cases.push_back(std::move(item));
    }
  }
}

struct Execution {
  Word initial_lc = 0, budget = 1;
};

BinaryCase forever_run(const Input input, const Execution execution) {
  const auto initial_lc = execution.initial_lc;
  const auto budget = execution.budget;
  auto item = entry(input);
  item.initial.lc = initial_lc;
  item.expected = item.initial;
  auto &state = item.expected;
  stack_push(state, (std::uint64_t{state.la} << word_bits) | state.lc);
  stack_push(state, (std::uint64_t{start + 2} << word_bits) | state.sr);
  state.sr |= status_mask;
  state.la = last;
  state.pc = start + 2;
  for (Word retired = 1; retired < budget; ++retired) {
    const auto position = (retired - 1) % 3;
    if (position == 0) {
      const auto old = state;
      sequence_model::alu(state, old, mac_a);
    }
    ++state.pc;
    if (position == 2) {
      state.lc = (state.lc + word_mask) % address_words;
      state.pc = start + 2;
    }
  }
  item.budget = item.dispatches = budget;
  item.progress = {.instructions = budget, .estimated_cycles = budget + 3};
  item.name = "FOREVER finite host budget, zero/one/wrapping LC, native ending";
  return item;
}

void forever_sequences(Cases &cases, Input input) {
  for (const auto mode : modes) {
    input.mode = mode;
    for (const auto pointer : pointers) {
      input.pointer = pointer;
      for (const Word lc : {0U, 1U, 2U, word_mask}) {
        for (Word budget = 1; budget <= maximum_budget; ++budget) {
          cases.push_back(forever_run(input, {.initial_lc = lc, .budget = budget}));
        }
      }
    }
  }
}

struct Condition {
  Word flags = 0;
  unsigned selector = 0;
};

bool selected(const Condition input) {
  const auto flags = input.flags;
  const auto selector = input.selector;
  const bool c = (flags & flag_c) != 0;
  const bool v = (flags & flag_v) != 0;
  const bool z = (flags & flag_z) != 0;
  const bool n = (flags & flag_n) != 0;
  const bool u = (flags & flag_u) != 0;
  const bool e = (flags & flag_e) != 0;
  const bool l = (flags & flag_l) != 0;
  const bool less = n != v;
  const bool normalized = z || (!u && !e);
  const std::array truth{!c,
                         !less,
                         !z,
                         !n,
                         !normalized,
                         !e,
                         !l,
                         !z && !less,
                         c,
                         less,
                         z,
                         n,
                         normalized,
                         e,
                         l,
                         z || less};
  return truth.at(selector);
}

struct Exit {
  Word current_flags = lf, outer_flags = 0, status = 0, pointer = 0;
  unsigned selector = 0;
  bool unconditional = false;
};

BinaryCase exit_loop(const Exit input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status | input.current_flags);
  auto &initial = item.initial;
  initial.sr = input.status | input.current_flags;
  initial.sp = input.pointer;
  initial.sc = 4;
  initial.la = last;
  initial.lc = 2;
  initial.stack.at(input.pointer % stack_entries) =
      (std::uint64_t{start - 7} << word_bits) | input.outer_flags | (status_byte ^ input.status);
  initial.stack.at((input.pointer + sp_range - 1) % stack_entries) =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  const bool taken =
      input.unconditional || selected({.flags = input.status, .selector = input.selector});
  item.expected.pc = start + 1;
  if (taken) {
    if (!input.unconditional) { item.expected.pc = last + 1; }
    stack_restore(item.expected);
  }
  item.before = {
      {.words = {input.unconditional ? enddo : brk_base | input.selector}, .address = start}};
  item.progress.estimated_cycles = input.unconditional ? 1U : 5U;
  item.name = "BRK truth table/ENDDO; restore LF/FV only, preserve CCR and PC distinction";
  return item;
}

void exit_statuses(Cases &cases, Exit input) {
  for (Word status = 0; status < 128; ++status) {
    input.status = status | flag_s | mode_rm;
    for (const Word outer : {0U, lf, fv, status_mask}) {
      input.outer_flags = outer;
      for (const Word pointer : {0U, 63U}) {
        input.pointer = pointer;
        cases.push_back(exit_loop(input));
      }
    }
  }
}

void exits(Cases &cases) {
  for (const Word current : {lf, status_mask}) {
    for (unsigned selector = 0; selector < 16; ++selector) {
      exit_statuses(cases, {.current_flags = current, .selector = selector});
    }
    exit_statuses(cases, {.current_flags = current, .unconditional = true});
  }
}

struct Nesting {
  Input input;
  bool outer_forever = false, inner_forever = false;
  Word budget = 1;
};

void nested_begin(Registers &state, const Input input) {
  const bool infinite = forever(input.kind);
  const auto old_count = state.lc;
  stack_push(state, (std::uint64_t{state.la} << word_bits) | state.lc);
  stack_push(state, (std::uint64_t{state.pc + 2} << word_bits) | state.sr);
  state.sr = (state.sr & ~fv) | lf | (infinite ? fv : 0U);
  state.la = input.source;
  state.lc = infinite ? old_count : 2U;
  state.pc += 2;
}

Word nested_step(Registers &state, const Nesting input, const unsigned phase) {
  if (phase == 0) {
    const auto previous = state;
    sequence_model::alu(state, previous, mac_a);
    ++state.pc;
    return 1;
  }
  if (phase == 1) {
    const auto kind = input.inner_forever ? Kind::forever_relative : Kind::relative;
    nested_begin(state, {.kind = kind, .source = start + 9});
    return input.inner_forever ? 4U : 5U;
  }
  if (phase == 3) {
    state.pc = start + 10;
    stack_restore(state);
    return 5;
  }
  ++state.pc;
  if (phase == 6) {
    if (!input.outer_forever && state.lc == 1) {
      stack_restore(state);
    } else {
      state.lc = (state.lc + word_mask) % address_words;
      state.pc = start + 2;
    }
  }
  return 1;
}

BinaryCase nested(const Nesting input) {
  const auto &base = input.input;
  const Kind outer_kind = input.outer_forever ? Kind::forever_absolute : Kind::absolute;
  const Kind inner_kind = input.inner_forever ? Kind::forever_relative : Kind::relative;
  BinaryCase item;
  item.initial = register_model::seed(base.mode);
  item.initial.sr = base.mode | flag_s;
  item.initial.sp = base.pointer;
  item.expected = item.initial;
  std::vector<Word> program{opcode({.kind = outer_kind, .source = 2}),
                            start + 12,
                            mac_a,
                            opcode({.kind = inner_kind, .source = 2}),
                            6,
                            0,
                            brk_base,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0};
  item.before = {{.words = std::move(program), .address = start}};
  auto &state = item.expected;
  nested_begin(state, {.kind = outer_kind, .source = start + 12});
  Word cycles = input.outer_forever ? 4U : 5U;
  for (Word retired = 1; retired < input.budget; ++retired) {
    cycles += nested_step(state, input, (retired - 1) % 7);
  }
  item.budget = item.dispatches = input.budget;
  item.progress = {.instructions = input.budget, .estimated_cycles = cycles};
  item.name = "Nested finite/FOREVER DO/DOR with BRK: all execution prefixes and outer LF/FV";
  return item;
}

void nested_prefixes(Cases &cases, Nesting input) {
  const Word limit = input.outer_forever ? maximum_budget : 15U;
  for (const auto mode : modes) {
    input.input.mode = mode;
    for (const auto pointer : pointers) {
      input.input.pointer = pointer;
      for (Word budget = 1; budget <= limit; ++budget) {
        input.budget = budget;
        cases.push_back(nested(input));
      }
    }
  }
}

BinaryCase rejected(const Word instruction, const Fault fault) {
  BinaryCase item;
  item.initial = register_model::seed(0);
  item.expected = item.initial;
  item.before = {{.words = {instruction, last}, .address = start}};
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = fault;
  item.name = "Explicit unsupported count/mode, stack or memory boundary";
  return item;
}

void faults(Cases &cases) {
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    const Word reserved = opcode({.kind = kind, .operand = Operand::reg, .source = 0});
    cases.push_back(rejected(reserved,
                             {.kind = FaultKind::invalid_instruction,
                              .space = Space::program,
                              .address = start,
                              .pc = start}));
    for (unsigned index = 0; index < agu_registers; ++index) {
      const Word invalid =
          opcode({.kind = kind, .operand = Operand::indirect, .address_mode = 6, .index = index});
      cases.push_back(rejected(invalid,
                               {.kind = FaultKind::invalid_instruction,
                                .space = Space::program,
                                .address = start,
                                .value = invalid,
                                .pc = start}));
    }
  }
  for (const auto kind : {Kind::absolute, Kind::relative}) {
    const Word invalid =
        opcode({.kind = kind, .operand = Operand::reg, .source = register_model::reg_ssh});
    cases.push_back(rejected(invalid,
                             {.kind = FaultKind::invalid_instruction,
                              .space = Space::program,
                              .address = start,
                              .value = invalid,
                              .pc = start}));
  }
  for (const auto kind :
       {Kind::absolute, Kind::relative, Kind::forever_absolute, Kind::forever_relative}) {
    auto item = rejected(opcode({.kind = kind}),
                         {.kind = FaultKind::unsupported_mode,
                          .space = Space::program,
                          .address = start,
                          .value = sc_mode,
                          .pc = start});
    item.initial.sr = sc_mode;
    cases.push_back(std::move(item));
  }
  auto active = rejected(0,
                         {.kind = FaultKind::unsupported_mode,
                          .space = Space::program,
                          .address = start,
                          .value = sc_mode | lf,
                          .pc = start});
  active.initial.sr = sc_mode | lf;
  active.initial.la = start;
  active.initial.lc = 3;
  cases.push_back(std::move(active));
}

void stack_faults(Cases &cases) {
  for (const auto kind :
       {Kind::absolute, Kind::relative, Kind::forever_absolute, Kind::forever_relative}) {
    for (const Word depth : {14U, 15U}) {
      auto item = rejected(opcode({.kind = kind}),
                           {.kind = FaultKind::stack,
                            .space = Space::program,
                            .address = start,
                            .value = depth == 14 ? 3U : 2U,
                            .pc = start});
      item.initial.sc = depth;
      cases.push_back(std::move(item));
    }
  }
  for (const Word instruction : {enddo, brk_base}) {
    for (const Word depth : {0U, 1U}) {
      auto item = rejected(instruction,
                           {.kind = FaultKind::stack,
                            .space = Space::program,
                            .address = start,
                            .value = depth == 0 ? 2U : 1U,
                            .pc = start});
      item.initial.sr = 0;
      item.initial.sc = depth;
      cases.push_back(std::move(item));
    }
  }
}

void extension_faults(Cases &cases) {
  for (const auto kind :
       {Kind::absolute, Kind::relative, Kind::forever_absolute, Kind::forever_relative}) {
    auto item = rejected(opcode({.kind = kind}),
                         {.kind = FaultKind::memory_read,
                          .space = Space::program,
                          .address = start + 1,
                          .pc = start});
    item.memory_words[0] = start + 1;
    item.before.front().words.resize(1);
    cases.push_back(std::move(item));
  }
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    auto item = rejected(
        opcode({.kind = kind, .operand = Operand::indirect}),
        {.kind = FaultKind::memory_read, .space = Space::x, .address = memory_words, .pc = start});
    item.initial.r[0] = memory_words;
    cases.push_back(std::move(item));
  }
}

void target_cases(Cases &cases) {
  constexpr Word far_target = 0x345678;
  for (const auto kind :
       {Kind::absolute, Kind::relative, Kind::forever_absolute, Kind::forever_relative}) {
    for (const Word target : {0U, word_mask, far_target}) {
      auto item = entry({.kind = kind});
      const auto displacement = (target + address_words - start) % address_words;
      item.before.front().words[1] = relative(kind) ? static_cast<Word>(displacement) : target;
      item.expected.la = target;
      item.name += "; full 24-bit end address and signed/wrapping displacement";
      cases.push_back(std::move(item));
    }
  }
}

} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_loop_control OUTPUT.d56"); }
    dsp56300::test::Cases cases;
    dsp56300::test::register_entries(cases);
    dsp56300::test::memory_entries(cases);
    dsp56300::test::repeat_sequences(cases);
    dsp56300::test::zero_repeat_counter_reads(cases);
    for (const auto kind :
         {dsp56300::test::Kind::forever_absolute, dsp56300::test::Kind::forever_relative}) {
      dsp56300::test::forever_sequences(cases, {.kind = kind});
    }
    dsp56300::test::exits(cases);
    for (const bool outer : {false, true}) {
      for (const bool inner : {false, true}) {
        dsp56300::test::nested_prefixes(
            cases, {.input = {}, .outer_forever = outer, .inner_forever = inner});
      }
    }
    dsp56300::test::faults(cases);
    dsp56300::test::stack_faults(cases);
    dsp56300::test::extension_faults(cases);
    dsp56300::test::target_cases(cases);
    const auto size = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << size << " independent loop-control cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
