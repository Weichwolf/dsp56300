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
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "support/isa.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
constexpr Word entry = 16, count = 3, lf = 1U << 15, fv = 1U << 16;
constexpr std::size_t words = 128;
constexpr unsigned mixed_cursor = 2, coefficient_cursor = 4;
constexpr unsigned target_shift = 18, y_accumulator_shift = 19, x_accumulator_shift = 17;
constexpr Word host_bits = 0xab000000U, multiplier = 7919;
constexpr Word linear = 0xffff, ring = 0x8007, mac = 0xd2;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

struct Shape {
  unsigned target = 0, secondary = 0, address_mode = 3;
  bool y = false, load = true, accumulator_b = false;
  Word modifier = linear, alu = 0;
};

struct Ram {
  std::array<Word, words * 2> data{};
  bool alias = false;

  std::span<Word> bank(const bool y) {
    return std::span(data).subspan(y && !alias ? words : 0, words);
  }

  void reset(const Word seed) {
    for (std::size_t index = 0; index < data.size(); ++index) {
      data[index] = host_bits | ((static_cast<Word>(index) * multiplier + seed) & word_mask);
    }
  }

  detail::JitRuntime runtime() {
    return {.memory = {{{.data = bank(false).data(), .words = words},
                        {.data = bank(true).data(), .words = words}}}};
  }
};

unsigned first_register(const Shape shape) {
  if (shape.target >= 2) { return 12 + shape.target; }
  return (shape.y ? 6U : 4U) + shape.target;
}

unsigned second_register(const Shape shape) {
  return (shape.y ? 4U : 6U) + shape.secondary;
}

Word mixed(const Shape shape) {
  auto result = static_cast<Word>(isa::Opcode::move_register_y);
  result |= shape.target << (shape.y ? 16U : target_shift);
  result |= shape.secondary << (shape.y ? target_shift : 16U);
  result |= static_cast<Word>(shape.accumulator_b)
            << (shape.y ? y_accumulator_shift : x_accumulator_shift);
  return result | (shape.y ? 1U << 14 : 0U) | (shape.load ? 1U << 15 : 0U) |
         (shape.address_mode << 11) | (mixed_cursor << 8) | shape.alu;
}

std::array<Word, count> program(const Shape shape) {
  const auto coefficient = static_cast<Word>(isa::Opcode::move_y_effective) | (1U << 15) |
                           (2U << 16) | (3U << 11) | (coefficient_cursor << 8);
  const auto output = static_cast<Word>(isa::Opcode::move_x_effective) | (3U << 11);
  return {mixed(shape), coefficient, output};
}

auto operations(const Shape shape, const Word mode) {
  std::array<detail::BlockInstruction, count> result{};
  const auto source = program(shape);
  for (unsigned index = 0; index < count; ++index) {
    result[index] = {.instruction = detail::decode(source[index]),
                     .pc = entry + index,
                     .mode = mode,
                     .loop_address = entry + count - 1};
    require(result[index].instruction.words == 1, "Authored mixed region has one-word operations");
  }
  return result;
}

Registers initial(const Shape shape, const Word mode, const Word flags = 0) {
  auto state = register_model::seed(mode);
  state.sr = mode | flags | lf;
  state.pc = entry;
  state.la = entry + count - 1;
  state.lc = 4;
  state.m.fill(word_mask);
  state.m[mixed_cursor] = shape.modifier;
  state.r[0] = 32;
  state.r[mixed_cursor] = 32;
  state.r[coefficient_cursor] = 64;
  state.n[mixed_cursor] = 3;
  state.stack[state.sp % stack_entries] = std::uint64_t{entry} << word_bits;
  return state;
}

Word address(Registers &state, const unsigned cursor, const unsigned mode) {
  const auto prior = state.r[cursor];
  state.r[cursor] = address_model::updated(
      {.modifier = state.m[cursor], .cursor = prior, .offset = state.n[cursor], .mode = mode});
  return mode == 7 ? state.r[cursor] : prior;
}

void step(Registers &state, Ram &ram, const Shape shape) {
  const auto before = state;
  const auto index = state.pc - entry;
  if (index == 0) {
    const auto at = address(state, mixed_cursor, shape.address_mode);
    const auto secondary = register_model::read(state, shape.accumulator_b ? 15U : 14U);
    if (shape.load) {
      register_model::write(state, first_register(shape), ram.bank(shape.y)[at] & word_mask);
    } else {
      ram.bank(shape.y)[at] = register_model::read(state, first_register(shape));
    }
    register_model::write(state, second_register(shape), secondary);
    sequence_model::alu(state, before, shape.alu);
  } else if (index == 1) {
    const auto at = address(state, coefficient_cursor, 3);
    register_model::write(state, 6, ram.bank(true)[at] & word_mask);
  } else {
    const auto at = address(state, 0, 3);
    ram.bank(false)[at] = register_model::read(state, 4);
  }
  ++state.pc;
  if (index != count - 1) { return; }
  if (state.lc == 1 && (state.sr & fv) == 0) {
    loop_frames::complete(state);
  } else {
    state.lc = (state.lc - 1) & word_mask;
    state.pc = entry;
  }
}

struct Grant {
  Word flags = 0, budget = count * 4;
};

void check(
    const native::Direct &code, Ram &ram, const Shape shape, const Word mode, const Grant grant) {
  ram.reset(grant.flags);
  auto expected_ram = ram;
  auto expected = initial(shape, mode, grant.flags);
  auto raw = expected;
  raw.a <<= detail::padding_bits;
  raw.b <<= detail::padding_bits;
  const Word retired = std::min(grant.budget / count, 4U) * count;
  std::uint64_t cycles = 0;
  for (Word index = 0; index < retired; ++index) {
    cycles += expected.pc == entry && shape.address_mode == 7 ? 2U : 1U;
    step(expected, expected_ram, shape);
  }
  detail::Cpu cpu{.reg = raw, .instructions = maximum - 2, .cycles = maximum - 7};
  detail::NativeContext context{.runtime = {}, .budget = grant.budget};
  require(code.enter(cpu, context) == retired, "Mixed guarded bytes retire complete proven bodies");
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(cpu.reg == expected && ram.data == expected_ram.data &&
              cpu.instructions == maximum - 2 + retired && cpu.cycles == maximum - 7 + cycles,
          "Mixed RAM preserves old accumulator/ALU inputs, aliases, AGU, flags and DO frames");
}

void matrix(const Shape shape, const Word mode, const bool alias) {
  Ram ram{.alias = alias};
  const auto state = initial(shape, mode);
  const auto runtime = ram.runtime();
  const auto ops = operations(shape, mode);
  require(!detail::resident_plan(ops, runtime.memory, state.m).valid,
          "Mixed RAM admission requires the explicit ARM compound capability");
  detail::a64::Emitter emitter;
  const auto bytes = emitter.generate_guarded(ops, runtime, state.m);
  if (!bytes) {
    throw std::runtime_error("Mixed memory/accumulator transfers require a guarded native region");
  }
  const native::Direct code(*bytes);
  for (Word flags = 0; flags <= status_byte; ++flags) {
    check(code, ram, shape, mode, {.flags = flags});
  }
  for (Word budget = 0; budget < count * 4; ++budget) {
    check(code, ram, shape, mode, {.flags = status_byte, .budget = budget});
  }
}

void prefixes(const Shape shape) {
  Ram ram;
  std::array<Word, words> p{};
  const auto source = program(shape);
  std::ranges::copy(source, p.begin() + entry);
  TestCore core(Buffers{.program = p, .x = ram.bank(false), .y = ram.bank(true)});
  constexpr Word total = count * 4;
  const Word cycles = total + (shape.address_mode == 7 ? 4U : 0U);
  for (const bool timed : {false, true}) {
    for (Word limit = 0; limit <= (timed ? cycles : total); ++limit) {
      ram.reset(limit);
      auto expected_ram = ram;
      auto expected = initial(shape, 0, status_byte);
      core.set_registers(expected);
      Word retired = 0;
      Word consumed = 0;
      while (retired < total && (timed ? consumed : retired) < limit) {
        consumed += expected.pc == entry && shape.address_mode == 7 ? 2U : 1U;
        step(expected, expected_ram, shape);
        ++retired;
      }
      const auto before = core.progress();
      const auto result = timed ? core.run_until(before.estimated_cycles + limit) : core.run(limit);
      require(result.fault == Fault{} && result.retired == retired &&
                  core.registers() == expected && ram.data == expected_ram.data &&
                  result.progress.instructions - before.instructions == retired &&
                  result.progress.estimated_cycles - before.estimated_cycles == consumed,
              "Every mixed native public budget/deadline prefix preserves exact ISA state");
    }
  }
}

void rejected_addresses(const Shape shape) {
  Ram ram;
  const auto runtime = ram.runtime();
  const auto ops = operations(shape, 0);
  const auto seed = initial(shape, 0, status_byte);
  detail::a64::Emitter emitter;
  const auto bytes = emitter.generate_guarded(ops, runtime, seed.m);
  if (!bytes) { throw std::runtime_error("Compile before testing mixed entry rejection"); }
  const native::Direct code(*bytes);
  for (const bool bad_modifier : {false, true}) {
    auto state = seed;
    state.r[0] = state.r[coefficient_cursor] = 0;
    if (bad_modifier) {
      state.m[mixed_cursor] = ring;
    } else {
      state.r[mixed_cursor] = words;
    }
    state.a <<= detail::padding_bits;
    state.b <<= detail::padding_bits;
    const detail::Cpu before{.reg = state};
    auto cpu = before;
    const auto data = ram.data;
    detail::NativeContext context{.runtime = {}, .budget = count};
    require(code.enter(cpu, context) == 0 && cpu.reg == before.reg && cpu.instructions == 0 &&
                cpu.cycles == 0 && ram.data == data,
            "A rejected mixed cursor or modifier has no speculative RAM or register effects");
  }
  auto aliased = runtime;
  aliased.memory[shape.y ? 1U : 0U].aliases_program = true;
  if (!shape.load) {
    detail::a64::Emitter denied;
    require(!denied.generate_guarded(ops, aliased, seed.m),
            "Mixed stores cannot bypass Harvard program-alias invalidation");
  }
}

void reentry_fault(const Shape shape) {
  Ram ram;
  ram.reset(0);
  std::array<Word, words> p{};
  const auto source = program(shape);
  std::ranges::copy(source, p.begin() + entry);
  TestCore core(Buffers{.program = p, .x = ram.bank(false), .y = ram.bank(true)});
  auto expected = initial(shape, 0);
  expected.r[mixed_cursor] = words - 1;
  core.set_registers(expected);
  auto expected_ram = ram;
  for (unsigned index = 0; index < count; ++index) { step(expected, expected_ram, shape); }
  const auto at = address(expected, mixed_cursor, shape.address_mode);
  static_cast<void>(register_model::read(expected, shape.accumulator_b ? 15U : 14U));
  const auto value = shape.load ? 0U : register_model::read(expected, first_register(shape));
  expected.pc = entry + 1;
  const auto result = core.run(count * 4);
  require(result.retired == count && core.registers() == expected &&
              ram.data == expected_ram.data &&
              result.progress == Progress{.instructions = count, .estimated_cycles = count} &&
              result.fault ==
                  Fault{.kind = shape.load ? FaultKind::memory_read : FaultKind::memory_write,
                        .space = shape.y ? Space::y : Space::x,
                        .address = at,
                        .value = value,
                        .pc = entry},
          "Mixed reentry preserves the complete ordinary fault prefix and preceding valid body");
}

void selectors(const Shape input, const Word mode) {
  for (const bool accumulator_b : {false, true}) {
    for (unsigned target = 0; target < 4; ++target) {
      for (unsigned secondary = 0; secondary < 2; ++secondary) {
        auto shape = input;
        shape.accumulator_b = accumulator_b;
        shape.target = target;
        shape.secondary = secondary;
        matrix(shape, mode, false);
        matrix(shape, mode, true);
      }
    }
  }
}

void addressing() {
  for (const auto address_mode : {1U, 2U, 3U, 4U, 7U}) {
    for (const auto modifier : {linear, ring}) {
      if (modifier != linear && (address_mode == 2 || address_mode == 7)) { continue; }
      for (const auto mode : modes) {
        matrix({.address_mode = address_mode, .modifier = modifier, .alu = mac}, mode, false);
      }
    }
    prefixes({.address_mode = address_mode, .alu = mac});
  }
}
}

int main() try {
  for (const auto mode : modes) {
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) { selectors({.y = y, .load = load}, mode); }
    }
  }
  addressing();
  for (const bool y : {false, true}) {
    for (const bool load : {false, true}) {
      rejected_addresses({.y = y, .load = load});
      reentry_fault({.y = y, .load = load});
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
