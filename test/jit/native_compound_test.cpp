// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "architecture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/resident.hpp"
#include "loop_frames.hpp"
#include "native/direct.hpp"
#include "native_long_load_fixture.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"
#include "support/isa.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word wrap = 0x8000, lf = 1U << 15, fv = 1U << 16;
constexpr Word long_base = 0x404000, single_base = 0x404000, temporary_register = 23;
constexpr Word source_address = 64;
constexpr unsigned clobbered_register = 21;
constexpr std::size_t ram_words = 128;
constexpr unsigned selector_high_shift = 17, register_high_mask = 24, long_bits = 48;
constexpr std::uint64_t long_mask = 0xffffffffffff;
constexpr Word coefficient_cursor = 47;
constexpr Word y_bus = 0x080000;
constexpr std::size_t prefix_size = 8;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array numeric_modes{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

enum class Tail : std::uint8_t {
  reg,
  short_,
  absolute,
  relative_short,
  relative_long,
  relative_reg
};

struct Shape {
  Word mode = 0, count = 3, condition = 0, alu = 0;
  unsigned selector = 2, address_mode = 3;
  Tail tail = Tail::reg;
  bool loop = false, conditional = true, left = false, self_clobber = false;
};

struct Memory {
  std::array<Word, ram_words * 2> words{};
  std::array<std::size_t, 2> limits{ram_words, ram_words};
  bool alias = false;

  std::span<Word> bank(const bool y) {
    return std::span(words).subspan(y && !alias ? ram_words : 0, limits[y ? 1 : 0]);
  }

  void initialize(const Word target) {
    constexpr Word high = 0xab000000;
    constexpr Word multiplier = 7919;
    for (Word index = 0; index < words.size(); ++index) {
      words[index] = high | (multiplier * index);
    }
    bank(false)[source_address] = target;
  }

  detail::JitRuntime runtime() {
    return {.memory = {{{.data = bank(false).data(), .words = limits[0]},
                        {.data = bank(true).data(), .words = limits[1]}}}};
  }
};

Word long_opcode(const Shape shape, const unsigned index, const bool load) {
  return long_base | ((shape.selector & 3U) << 16) |
         ((shape.selector & 4U) << selector_high_shift) | (shape.address_mode << 11) |
         (index << 8) | (load ? 0x8000U : 0U) | shape.alu;
}

Word single_opcode(const unsigned reg, const unsigned index, const unsigned mode) {
  return single_base | ((reg & 7U) << 16) | ((reg & register_high_mask) << selector_high_shift) |
         0x8000U | (mode << 11) | (index << 8);
}

Word relative_opcode(const Shape shape, const Word displacement) {
  using isa::Opcode;
  if (shape.tail == Tail::relative_short) {
    constexpr Word upper = 0x1e0;
    constexpr Word lower = 0x1f;
    const auto base = shape.conditional ? Opcode::bcc_relative_short : Opcode::bra_relative_short;
    return static_cast<Word>(base) | ((displacement & upper) << 1) | (displacement & lower) |
           (shape.conditional ? shape.condition << 12 : 0U);
  }
  auto base = shape.conditional ? Opcode::bcc_relative_long : Opcode::bra_relative_long;
  if (shape.tail == Tail::relative_reg) {
    base = shape.conditional ? Opcode::bcc_relative_register : Opcode::bra_relative_register;
  }
  return static_cast<Word>(base) | (shape.tail == Tail::relative_reg ? 6U << 8 : 0U) |
         (shape.conditional ? shape.condition : 0U);
}

detail::Instruction terminal(const Shape shape, const Word pc, const Word target) {
  using isa::Opcode;
  Word opcode = 0;
  Word extension = 0;
  if (shape.tail == Tail::reg || shape.tail == Tail::absolute) {
    opcode = static_cast<Word>(shape.conditional ? Opcode::jcc_effective : Opcode::jmp_effective);
    opcode |= shape.tail == Tail::reg ? (4U << 11) | (7U << 8) : 6U << 11;
    if (shape.conditional) { opcode |= shape.condition; }
    extension = target;
  } else if (shape.tail == Tail::short_) {
    const auto base = shape.conditional ? Opcode::jcc_absolute_short : Opcode::jmp_absolute_short;
    opcode = static_cast<Word>(base) | target | (shape.conditional ? shape.condition << 12 : 0U);
  } else {
    extension = (target - pc) & word_mask;
    opcode = relative_opcode(shape, extension);
  }
  auto instruction = detail::decode(opcode);
  instruction.extension = extension;
  return instruction;
}

std::vector<detail::BlockInstruction> operations(const Shape shape, const Word target) {
  const shift_model::Control shift{.count = shape.count,
                                   .left = shape.left,
                                   .source_b = false,
                                   .destination_b = true,
                                   .saturate = false};
  const std::array words{long_opcode(shape, 0, true),
                         long_opcode(shape, 4, true),
                         shift_model::immediate(shift),
                         single_opcode(temporary_register, 5, 4),
                         long_opcode(shape, 0, false),
                         single_opcode(6, 4, 3) | y_bus,
                         long_opcode(shape, 4, false),
                         single_opcode(clobbered_register, shape.self_clobber ? 5U : 0U, 4) |
                             y_bus};
  std::vector<detail::BlockInstruction> result;
  Word pc = start;
  for (std::size_t index = 0; index < words.size(); ++index) {
    const auto word = words[index];
    const auto instruction = detail::decode(word);
    const bool long_move = index == 0 || index == 1 || index == 4 || index == 6;
    const auto cost = long_move && shape.address_mode == 7 ? 2U : 1U;
    require(instruction.words == 1 && instruction.cycles == cost,
            "Independent compound RAM forms have declared widths and base costs");
    result.push_back({.instruction = instruction, .pc = pc++, .mode = shape.mode});
  }
  result.push_back({.instruction = terminal(shape, pc, target), .pc = pc, .mode = shape.mode});
  const auto tail_words =
      shape.tail == Tail::absolute || shape.tail == Tail::relative_long ? 2U : 1U;
  const bool relative = shape.tail == Tail::relative_short || shape.tail == Tail::relative_long ||
                        shape.tail == Tail::relative_reg;
  auto tail_cost = shape.conditional || relative || shape.tail == Tail::absolute ? 4U : 3U;
  if (shape.tail == Tail::relative_long && shape.conditional) { ++tail_cost; }
  require(result.back().instruction.words == tail_words &&
              result.back().instruction.cycles == tail_cost,
          "Independent jump forms retain their declared widths and base costs");
  const auto last = pc + result.back().instruction.words - 1;
  for (auto &item : result) { item.loop_address = shape.loop ? last : detail::inactive_loop; }
  return result;
}

Registers initial(const Shape shape, const Word target) {
  auto state = register_model::seed(shape.mode);
  state.sr = shape.mode | (shape.loop ? lf : 0U);
  state.m.fill(word_mask);
  state.m[0] = wrap | 15U;
  state.m[4] = wrap | 7U;
  state.r[0] = 15;
  state.r[5] = source_address;
  state.r[4] = coefficient_cursor;
  state.r[6] = (target - (start + 8)) & word_mask;
  state.n[0] = 3;
  state.n[4] = word_mask;
  state.pc = start;
  state.lc = 3;
  state.stack[state.sp % stack_entries] = std::uint64_t{start} << word_bits;
  const auto ops = operations(shape, target);
  if (shape.loop) { state.la = ops.back().loop_address; }
  return state;
}

Word address(Registers &state, const unsigned index, const unsigned mode) {
  const auto before = state.r[index];
  const auto next = address_model::updated(
      {.modifier = state.m[index], .cursor = before, .offset = state.n[index], .mode = mode});
  if (mode != 4) { state.r[index] = next; }
  return mode == 7 ? next : before;
}

std::uint64_t long_value(Registers &state, const unsigned selector) {
  if (selector < 2) { return (selector == 0 ? state.a : state.b) & long_mask; }
  if (selector < 4) { return selector == 2 ? state.x : state.y; }
  if (selector >= 6) {
    const auto high = register_model::read(state, selector == 6 ? 14U : 15U);
    const auto low = register_model::read(state, selector == 6 ? 15U : 14U);
    return (std::uint64_t{high} << word_bits) | low;
  }
  auto value = signed56(selector == 4 ? state.a : state.b);
  if ((state.sr & mode_up) != 0) { value *= 2; }
  if ((state.sr & mode_down) != 0) { value = value >= 0 ? value / 2 : -((-value + 1) / 2); }
  constexpr unsigned growth_bit = 45;
  const auto raw = static_cast<std::uint64_t>(value);
  if (((raw >> growth_bit) & 1U) != ((raw >> (growth_bit + 1)) & 1U)) { state.sr |= flag_s; }
  constexpr auto bound = std::int64_t{1} << (long_bits - 1);
  if (value < -bound || value >= bound) { state.sr |= flag_l; }
  return static_cast<std::uint64_t>(std::clamp(value, -bound, bound - 1)) & long_mask;
}

void move_long(
    Registers &state, Memory &memory, const Shape shape, const unsigned index, const bool load) {
  const auto old = state;
  const auto at = address(state, index, shape.address_mode);
  if (load) {
    const auto value = (std::uint64_t{memory.bank(false)[at] & word_mask} << word_bits) |
                       (memory.bank(true)[at] & word_mask);
    long_load::expected_load(state, {.selector = shape.selector, .mode = 4, .value = value});
  } else {
    const auto value = long_value(state, shape.selector);
    memory.bank(false)[at] = static_cast<Word>(value >> word_bits);
    memory.bank(true)[at] = static_cast<Word>(value) & word_mask;
    ++state.pc;
  }
  long_load::expected_alu(state, old, shape.alu);
}

void step(Registers &state, Memory &memory, const Shape shape, const std::size_t index) {
  if (index == 0 || index == 1 || index == 4 || index == 6) {
    move_long(state, memory, shape, index == 0 || index == 4 ? 0U : 4U, index < 2);
    return;
  }
  if (index == 2) {
    const auto old = state;
    shift_model::apply(state,
                       old,
                       {.count = shape.count,
                        .left = shape.left,
                        .source_b = false,
                        .destination_b = true,
                        .saturate = false});
  } else if (index == 3) {
    register_model::write(state, temporary_register, memory.bank(false)[state.r[5]] & word_mask);
  } else if (index == 5) {
    const auto at = address(state, 4, 3);
    register_model::write(state, 6, memory.bank(true)[at] & word_mask);
  } else if (index == 7) {
    const auto cursor = state.r[shape.self_clobber ? 5U : 0U];
    register_model::write(state, clobbered_register, memory.bank(true)[cursor] & word_mask);
  }
  ++state.pc;
}

bool admitted(const Registers &state, const Memory &memory, const Shape shape) {
  if (shape.loop && !loop_frames::admitted(state)) { return false; }
  return state.m[0] % 0x10000 == (wrap | 15U) && state.m[4] % 0x10000 == (wrap | 7U) &&
         (state.r[0] / 16 + 1) * std::uint64_t{16} <=
             std::min(memory.limits[0], memory.limits[1]) &&
         (state.r[4] / 8 + 1) * std::uint64_t{8} <= std::min(memory.limits[0], memory.limits[1]) &&
         state.r[5] <
             (shape.self_clobber ? std::min(memory.limits[0], memory.limits[1]) : memory.limits[0]);
}

struct Grant {
  Word target = 0, budget = 0;
  bool deadline = false;
};

void report_registers(const Registers &actual, const Registers &expected) {
  std::cerr << std::hex << "a=" << actual.a << '/' << expected.a << " b=" << actual.b << '/'
            << expected.b << " x=" << actual.x << '/' << expected.x << " y=" << actual.y << '/'
            << expected.y << " sr=" << actual.sr << '/' << expected.sr << " pc=" << actual.pc << '/'
            << expected.pc << '\n';
  for (std::size_t index = 0; index < agu_registers; ++index) {
    if (actual.r[index] != expected.r[index]) {
      std::cerr << "r" << index << '=' << actual.r[index] << '/' << expected.r[index] << '\n';
    }
  }
}

void direct_case(const native::Direct &code,
                 Memory &memory,
                 Registers expected,
                 const Shape shape,
                 const Grant grant) {
  const auto target = grant.target;
  const auto budget = grant.budget;
  const auto beginning = expected;
  memory.initialize(target);
  auto expected_memory = memory;
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = cpu.cycles = maximum - 2;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  const auto ops = operations(shape, target);
  Word retired = 0;
  std::uint64_t cycles = 0;
  auto boundary = detail::NativeBoundary::none;
  while (budget - retired >= ops.size() && admitted(expected, memory, shape)) {
    for (std::size_t index = 0; index < prefix_size; ++index) {
      step(expected, expected_memory, shape, index);
    }
    const auto &last = ops.back();
    expected.pc = last.pc + last.instruction.words;
    if (!shape.conditional ||
        selection_model::test_condition({.status = expected.sr, .selector = shape.condition})) {
      expected.pc = shape.tail == Tail::reg ? expected.r[7] : target;
    }
    for (const auto &item : ops) { cycles += item.instruction.cycles; }
    retired += static_cast<Word>(ops.size());
    if (shape.loop && expected.pc == expected.la + 1) {
      if (expected.lc == 1 && (expected.sr & fv) == 0) {
        loop_frames::complete(expected);
        boundary = detail::NativeBoundary::context;
      } else {
        expected.lc = (expected.lc - 1) & word_mask;
        expected.pc = start;
      }
    }
    if (!shape.loop || boundary != detail::NativeBoundary::none || expected.pc != start) { break; }
  }
  detail::NativeContext context{.runtime = {}, .budget = budget};
  const auto result = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  const bool correct =
      result == retired && cpu.reg == expected && memory.words == expected_memory.words &&
      cpu.instructions == maximum - 2 + retired && cpu.cycles == maximum - 2 + cycles &&
      cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0) &&
      context.failure.kind == FaultKind::none && context.boundary == boundary;
  if (!correct) {
    std::cerr << "shape mode=" << shape.mode << " selector=" << shape.selector
              << " address=" << shape.address_mode << " condition=" << shape.condition
              << " tail=" << static_cast<unsigned>(shape.tail) << " loop=" << shape.loop
              << " target=" << target << " budget=" << budget << " initial-sr=" << beginning.sr
              << " initial-lc=" << beginning.lc << " retired=" << result << '/' << retired
              << " cycles=" << cpu.cycles << '/' << maximum - 2 + cycles
              << " boundary=" << static_cast<unsigned>(context.boundary) << '/'
              << static_cast<unsigned>(boundary) << '\n';
    report_registers(cpu.reg, expected);
  }
  require(correct,
          "Compound native RAM/control bytes preserve independent complete state and retirement");
}

void matrix(const Shape shape, const bool alias) {
  constexpr std::array targets{start, start + 9, start + 10, start + 16};
  for (const auto target : targets) {
    Memory memory{.alias = alias};
    const auto state = initial(shape, target);
    const auto ops = operations(shape, target);
    detail::a64::Emitter emitter;
    const auto bytes = emitter.generate_guarded(ops, memory.runtime(), state.m);
    if (!bytes) {
      throw std::runtime_error("A guarded compound RAM prefix and native jump must be admitted");
    }
    const native::Direct code(bytes.value());
    for (Word flags = 0; flags <= status_byte; ++flags) {
      auto changed = state;
      changed.sr |= flags;
      direct_case(code,
                  memory,
                  changed,
                  shape,
                  {.target = target, .budget = static_cast<Word>(ops.size())});
    }
    for (const Word lc : {0U, 1U, 2U, 3U, word_mask}) {
      for (Word budget = 0; budget <= ops.size() * 2; ++budget) {
        auto changed = state;
        changed.lc = lc;
        direct_case(code, memory, changed, shape, {.target = target, .budget = budget});
      }
    }
    for (const Word cursor : {0U, 15U, 16U, 112U, 127U, 128U, word_mask}) {
      auto changed = state;
      changed.r[0] = cursor;
      direct_case(code,
                  memory,
                  changed,
                  shape,
                  {.target = target, .budget = static_cast<Word>(ops.size())});
    }
  }
}

void admission() {
  Memory memory;
  constexpr Shape shape{};
  const auto state = initial(shape, start);
  auto ops = operations(shape, start);
  const auto prefix = std::span(ops).first(prefix_size);
  const auto runtime = memory.runtime();
  require(!detail::resident_plan(
               prefix, runtime.memory, state.m, detail::RegionAddressing::masked_rings)
                  .valid &&
              !detail::resident_plan(
                   prefix, runtime.memory, state.m, detail::RegionAddressing::compound)
                   .valid &&
              detail::resident_plan(
                  prefix, runtime.memory, state.m, detail::RegionAddressing::single_pass)
                  .valid,
          "Compound RAM admission requires an explicit backend capability");
  for (const unsigned reg : {16U, 20U, 31U}) {
    ops[3].instruction = detail::decode(single_opcode(reg, 5, 4));
    require(!detail::resident_plan(
                 prefix, runtime.memory, state.m, detail::RegionAddressing::single_pass)
                 .valid,
            "Loaded memory cursors and N registers cannot invalidate an entry proof");
  }
  ops = operations(shape, start);
  for (const std::size_t bank : {0U, 1U}) {
    auto denied = runtime;
    denied.memory[bank].aliases_program = true;
    detail::a64::Emitter emitter;
    require(!emitter.generate_guarded(ops, denied, state.m),
            "Either Harvard program alias retains precise store invalidation");
  }
  auto invalid = shape;
  invalid.count = accumulator_bits;
  detail::a64::Emitter emitter;
  require(!emitter.generate_guarded(operations(invalid, start), runtime, state.m),
          "Undefined immediate shifts retain their ordinary fault boundary");
}

void public_case(TestCore &core,
                 Memory &memory,
                 const Shape shape,
                 const std::span<const detail::BlockInstruction> ops,
                 const Grant grant) {
  memory.initialize(grant.target);
  auto expected_memory = memory;
  const auto state = initial(shape, grant.target);
  auto expected = state;
  std::uint64_t cycles = 0;
  Word retired = 0;
  while (retired < ops.size() && (grant.deadline ? cycles : retired) < grant.budget) {
    const auto &item = ops[retired];
    if (retired < prefix_size) {
      step(expected, expected_memory, shape, retired);
    } else {
      expected.pc = item.pc + item.instruction.words;
      if (!shape.conditional ||
          selection_model::test_condition({.status = expected.sr, .selector = shape.condition})) {
        expected.pc = grant.target;
      }
    }
    cycles += item.instruction.cycles;
    ++retired;
  }
  core.set_registers(state);
  const auto before = core.progress();
  const auto run = grant.deadline ? core.run_until(before.estimated_cycles + grant.budget)
                                  : core.run(grant.budget);
  require(run.retired == retired && run.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.words == expected_memory.words &&
              run.progress == Progress{.instructions = before.instructions + retired,
                                       .estimated_cycles = before.estimated_cycles + cycles},
          "Every public instruction/grant.deadline prefix preserves RAM, registers and complete "
          "cycles");
}

void install_program(TestCore &core, const std::span<const detail::BlockInstruction> ops) {
  std::vector<Word> program;
  program.reserve(ops.size() + 1);
  for (const auto &item : ops) {
    program.push_back(item.instruction.opcode);
    if (item.instruction.words == 2) { program.push_back(item.instruction.extension); }
  }
  core.load(Space::program, start, program);
}

void public_prefixes(const Shape shape) {
  constexpr Word target = start + 16;
  Memory memory;
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                nullptr,
                {.backend = Backend::jit});
  const auto ops = operations(shape, target);
  install_program(core, ops);
  std::uint64_t total = 0;
  for (const auto &item : ops) { total += item.instruction.cycles; }
  for (const bool deadline : {false, true}) {
    const auto end = deadline ? total : ops.size();
    for (std::uint64_t grant = 0; grant <= end; ++grant) {
      public_case(core,
                  memory,
                  shape,
                  ops,
                  {.target = target, .budget = static_cast<Word>(grant), .deadline = deadline});
    }
  }
}

void reentry_fault() {
  constexpr Shape shape{.loop = true, .conditional = false};
  Memory memory;
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                nullptr,
                {.backend = Backend::jit});
  const auto ops = operations(shape, start);
  install_program(core, ops);
  memory.initialize(start);
  auto expected_memory = memory;
  const auto state = initial(shape, start);
  auto expected = state;
  for (std::size_t index = 0; index < prefix_size; ++index) {
    step(expected, expected_memory, shape, index);
  }
  expected.pc = start;
  const auto invalid = expected.r[5];
  require(invalid >= ram_words && invalid < detail::io_begin,
          "The independent late clobber must leave ordinary RAM");
  std::uint64_t cycles = 0;
  for (const auto &item : ops) { cycles += item.instruction.cycles; }
  constexpr Word before_fault = 3;
  for (Word index = 0; index < before_fault; ++index) {
    step(expected, expected_memory, shape, index);
    cycles += ops[index].instruction.cycles;
  }
  expected.pc = start + before_fault + 1;
  core.set_registers(state);
  const auto run = core.run(static_cast<Word>(ops.size() * 2));
  require(run.retired == ops.size() + before_fault && core.registers() == expected &&
              memory.words == expected_memory.words &&
              run.fault == Fault{.kind = FaultKind::memory_read,
                                 .space = Space::x,
                                 .address = invalid,
                                 .pc = start + before_fault} &&
              run.progress ==
                  Progress{.instructions = ops.size() + before_fault, .estimated_cycles = cycles},
          "Reentry rejects the changed cursor and exposes the exact ordinary RAM fault prefix");
}

void grant_boundaries() {
  constexpr Shape shape{};
  constexpr Word target = start + 16;
  for (const std::size_t bank : {0U, 1U}) {
    Memory memory;
    memory.limits[bank] = ram_words - 1;
    const auto state = initial(shape, target);
    const auto ops = operations(shape, target);
    detail::a64::Emitter emitter;
    const auto bytes = emitter.generate_guarded(ops, memory.runtime(), state.m);
    if (!bytes) { throw std::runtime_error("A shortened Harvard grant retains guarded admission"); }
    const native::Direct code(bytes.value());
    for (const Word cursor : {96U, 112U, 126U, 127U}) {
      auto changed = state;
      changed.r[0] = cursor;
      direct_case(code,
                  memory,
                  changed,
                  shape,
                  {.target = target, .budget = static_cast<Word>(ops.size())});
    }
    for (const Word modifier : {0U, 0x800fU, 0xab800fU, 0xffffU}) {
      auto changed = state;
      changed.m[0] = modifier;
      direct_case(code,
                  memory,
                  changed,
                  shape,
                  {.target = target, .budget = static_cast<Word>(ops.size())});
    }
  }
}

void mode_cases(const Word mode) {
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const unsigned address_mode : {1U, 2U, 3U, 4U, 7U}) {
      matrix({.mode = mode, .selector = selector, .address_mode = address_mode}, false);
    }
  }
  for (Word condition = 0; condition < 16; ++condition) {
    for (const auto tail : {Tail::reg,
                            Tail::short_,
                            Tail::absolute,
                            Tail::relative_short,
                            Tail::relative_long,
                            Tail::relative_reg}) {
      for (const bool loop : {false, true}) {
        matrix({.mode = mode, .condition = condition, .tail = tail, .loop = loop}, true);
      }
    }
  }
  for (const Word count : {0U, 1U, 23U, 24U, 47U, 48U, 55U}) {
    for (const bool left : {false, true}) {
      matrix({.mode = mode, .count = count, .loop = true, .conditional = false, .left = left},
             false);
    }
  }
  for (const Word alu : {0x10U, 0x11U, 0x13U, 0x36U, 0x43U, 0xd2U}) {
    matrix({.mode = mode, .alu = alu}, true);
  }
  matrix({.mode = mode, .loop = true, .self_clobber = true}, true);
  public_prefixes({.mode = mode, .self_clobber = true});
  for (unsigned selector = 0; selector < 8; ++selector) {
    public_prefixes({.mode = mode, .selector = selector});
  }
  for (const auto tail : {Tail::reg,
                          Tail::short_,
                          Tail::absolute,
                          Tail::relative_short,
                          Tail::relative_long,
                          Tail::relative_reg}) {
    matrix({.mode = mode, .tail = tail, .loop = true, .conditional = false}, true);
    public_prefixes({.mode = mode, .tail = tail});
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    admission();
    reentry_fault();
    grant_boundaries();
    for (const auto mode : numeric_modes) { mode_cases(mode); }
    std::cout << checks << " independent compound RAM/control checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
