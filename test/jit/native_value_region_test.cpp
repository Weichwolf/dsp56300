// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word loop_flag = Word{1} << 15, forever_flag = Word{1} << 16;
constexpr Word loop_bits = loop_flag | forever_flag, pointer_mask = 63;
constexpr Word input_base = 32, output_base = 80, previous_end = 0x350, previous_count = 19;
constexpr unsigned register_bits = 24;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word postincrements = (3U << 11) | (3U << 20);
constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
constexpr Word store_registers = (2U << 18) | (3U << 16);
constexpr Word store_addresses = (1U << 8) | (1U << 13);
constexpr Word product = static_cast<Word>(isa::Opcode::mpy_register_a) | (5U << 4);
constexpr Word rounding = static_cast<Word>(isa::Opcode::rnd_a);
constexpr std::array body{pair | postincrements | loads,
                          product,
                          rounding,
                          pair | postincrements | store_registers | store_addresses};
constexpr std::array<Word, 9> samples{
    0, 1, sign24 - 1, sign24, word_mask, 0x400000, 0xc00000, 0x123456, 0xabfedcba};
constexpr std::size_t data_words = 128;
using Data = std::array<Word, data_words>;

enum class Shape : std::uint8_t { product, negative, wide_shift, word_shift, clear, add, subtract };

std::array<Word, 4> instructions(const Shape shape) {
  auto result = body;
  switch (shape) {
    case Shape::product: break;
    case Shape::negative: result[1] |= 4U; break;
    case Shape::wide_shift:
      result[0] = static_cast<Word>(isa::Opcode::tfr_b_a);
      result[1] = static_cast<Word>(isa::Opcode::asr_a);
      break;
    case Shape::word_shift:
      result[1] = static_cast<Word>(isa::Opcode::tfr_x0_a);
      result[2] = static_cast<Word>(isa::Opcode::asr_a);
      break;
    case Shape::clear:
      result[1] = static_cast<Word>(isa::Opcode::clr_a);
      result[2] = static_cast<Word>(isa::Opcode::asr_a);
      break;
    case Shape::add:
      result[1] = static_cast<Word>(isa::Opcode::tfr_x0_a);
      result[2] = static_cast<Word>(isa::Opcode::add_y0_a);
      break;
    case Shape::subtract:
      result[1] = static_cast<Word>(isa::Opcode::tfr_x0_a);
      result[2] = static_cast<Word>(isa::Opcode::sub_y0_a);
      break;
    default: throw std::logic_error("Unknown directed map shape");
  }
  return result;
}

struct Point {
  Word count = 1, prefix = 0, mode = 0, pointer = 0, output = output_base, saved = 0,
       modifier = word_mask, stack_count = 2;
  std::uint64_t accumulator_b = register_model::seed_accumulator_b;
  bool shared = false;
};

struct Model {
  Registers state;
  Data x;
  Data y;
  Fault failure;
  std::array<Word, 4> code = body;
  Word retired = 0;
};

void advance(Registers &state, const unsigned index) {
  state.r[index] = address_model::updated({.modifier = state.m[index],
                                           .cursor = state.r[index],
                                           .offset = state.n[index],
                                           .mode = 3,
                                           .index = index});
}

bool step(Model &model, const bool shared) {
  auto &state = model.state;
  auto &x = model.x;
  auto &y = shared ? model.x : model.y;
  const auto index = state.pc - start;
  if (index == 0 && model.code[0] == body[0]) {
    state.x = (state.x & ~std::uint64_t{word_mask}) | (x[state.r[0]] & word_mask);
    state.y = (state.y & ~std::uint64_t{word_mask}) | (y[state.r[4]] & word_mask);
    advance(state, 0);
    advance(state, 4);
  } else if (index < 3) {
    const auto old = state;
    sequence_model::alu(state, old, model.code[index]);
  } else {
    const auto a = register_model::bus_word(state, state.a);
    const auto b = register_model::bus_word(state, state.b);
    x[state.r[1]] = a;
    y[state.r[5]] = b;
    advance(state, 1);
    advance(state, 5);
  }
  ++state.pc;
  if (index != body.size() - 1) { return true; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
    return true;
  }
  const auto pop = [&]() -> std::uint64_t {
    if (state.sc == 0) {
      model.failure = {
          .kind = FaultKind::stack, .address = state.la, .value = state.sp, .pc = state.la};
      return 0;
    }
    const auto frame = state.stack[state.sp % stack_entries];
    state.sp = (state.sp + pointer_mask) & pointer_mask;
    --state.sc;
    return frame;
  };
  const auto status = static_cast<Word>(pop());
  const auto outer = pop();
  state.sr = (state.sr & ~loop_bits) | (status & loop_bits);
  state.la = static_cast<Word>(outer >> word_bits);
  state.lc = static_cast<Word>(outer) & word_mask;
  return model.failure.kind == FaultKind::none;
}

class Machine {
public:
  explicit Machine(const bool shared, const Shape shape = Shape::product)
      : code_(instructions(shape)),
        core_(Memory::separate(
            {.program = program_, .x = x_, .y = shared ? std::span(x_) : std::span(y_)})) {
    std::ranges::copy(code_, program_.begin() + start);
    for (std::size_t index = 0; index < x_.size(); ++index) {
      x_[index] = samples[index % samples.size()];
      y_[index] = samples[(3 * index + 2) % samples.size()];
    }
    seed_x_ = x_;
    seed_y_ = y_;
  }

  void check(const Point point) {
    if (core_.state().failure.kind != FaultKind::none) { core_.reset(); }
    x_ = seed_x_;
    y_ = seed_y_;
    auto state = register_model::seed(point.mode);
    state.sr |= loop_flag;
    state.b = point.accumulator_b;
    state.la = start + static_cast<Word>(body.size()) - 1;
    state.lc = point.count;
    state.sp = point.pointer;
    state.sc = point.stack_count;
    state.m.fill(point.modifier);
    state.r[0] = state.r[4] = input_base;
    state.r[1] = state.r[5] = point.output;
    state.stack[state.sp % stack_entries] =
        (std::uint64_t{start} << register_bits) | ((state.sr & ~loop_bits) | point.saved);
    state.stack[(state.sp + pointer_mask) % stack_entries] =
        (std::uint64_t{previous_end} << register_bits) | previous_count;
    Model model{.state = state, .x = seed_x_, .y = seed_y_, .failure = {}, .code = code_};
    for (Word index = 0; index < point.prefix; ++index) {
      if (!step(model, point.shared)) { break; }
      ++model.retired;
    }
    core_.set_registers(state);
    const auto before = core_.state().progress;
    const auto result = core_.run(point.prefix);
    const auto expected_progress =
        Progress{.instructions = before.instructions + model.retired,
                 .estimated_cycles = before.estimated_cycles + model.retired};
    const auto actual = core_.registers();
    const bool matched = result.retired == model.retired && result.fault == model.failure &&
                         result.state == ExecutionState::running && !result.yielded &&
                         actual == model.state && result.progress == expected_progress &&
                         x_ == model.x && y_ == model.y;
    if (!matched) {
      std::cerr << "count=" << point.count << " prefix=" << point.prefix << " mode=" << point.mode
                << " SP=" << point.pointer << " output=" << point.output
                << " shared=" << point.shared << " registers=" << (actual == model.state)
                << " memory=" << (x_ == model.x && y_ == model.y)
                << " actual PC/SR/A/B=" << actual.pc << '/' << actual.sr << '/' << actual.a << '/'
                << actual.b << " expected=" << model.state.pc << '/' << model.state.sr << '/'
                << model.state.a << '/' << model.state.b << '\n';
    }
    require(matched,
            "Cold/warm whole and partial maps preserve independent full state, aliases, sticky "
            "flags and all RAM");
  }

private:
  std::array<Word, memory_words> program_{};
  Data x_{}, y_{}, seed_x_{}, seed_y_{};
  const std::array<Word, 4> code_;
  Core core_;
};

void prefixes(Machine &machine, Point point) {
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    for (Word prefix = 0; prefix <= point.count * body.size(); ++prefix) {
      point.prefix = prefix;
      machine.check(point);
    }
  }
}

void counts(Machine &machine, Point point) {
  for (const Word count : {1U, 2U, 3U, 4U, 5U, 8U, 9U}) {
    point.count = count;
    prefixes(machine, point);
  }
}

void placements(Machine &machine, Point point) {
  for (const Word pointer : {0U, 15U, 16U, pointer_mask}) {
    point.pointer = pointer;
    for (const Word output : {input_base, input_base + 1, output_base}) {
      point.output = output;
      counts(machine, point);
    }
  }
}

void shapes(const bool shared) {
  for (const auto shape : {Shape::negative,
                           Shape::wide_shift,
                           Shape::word_shift,
                           Shape::clear,
                           Shape::add,
                           Shape::subtract}) {
    Machine machine(shared, shape);
    for (const Word mode : {0U, mode_rm}) {
      for (const auto accumulator : {std::uint64_t{0}, sign56 - 1, sign56, accumulator_mask}) {
        prefixes(machine,
                 {.count = 4, .mode = mode, .accumulator_b = accumulator, .shared = shared});
      }
    }
  }
}

void tail_guard() {
  constexpr std::size_t extent = 64;
  constexpr Word count = 4;
  std::array<Word, memory_words> program{};
  Data x;
  Data y;
  constexpr Word x_guard = 0x13579b;
  constexpr Word y_guard = 0x2468ac;
  x.fill(x_guard);
  y.fill(y_guard);
  for (Word index = 0; index < count; ++index) {
    x[input_base + index] = 1;
    y[input_base + index] = sign24 / 2;
  }
  std::ranges::copy(body, program.begin() + start);
  auto initial = initial_state();
  initial.b = register_model::seed_accumulator_b;
  initial.sr = loop_flag | flag_c;
  initial.la = start + static_cast<Word>(body.size()) - 1;
  initial.lc = count;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  initial.r[0] = initial.r[4] = input_base;
  initial.r[1] = initial.r[5] = extent - 3;
  Core core(Memory::separate(
      {.program = program, .x = std::span(x).first(extent), .y = std::span(y).first(extent)}));
  core.set_registers(initial);
  const Word budget = count * body.size();
  const auto result = core.run(budget);
  require(result.retired == budget - 1 &&
              result.progress ==
                  Progress{.instructions = budget - 1, .estimated_cycles = budget - 1} &&
              result.fault.kind == FaultKind::memory_write && result.fault.space == Space::x &&
              result.fault.address == extent && result.fault.pc == initial.la &&
              x[extent] == x_guard && y[extent] == y_guard,
          "A packet crossing its RAM extent retains the precise first failed write and untouched "
          "host guards");
  for (std::size_t index = extent - 3; index < extent; ++index) {
    require(x[index] == 0 && y[index] == sign24,
            "Completed writes before the RAM fault remain committed");
  }
}

void root_spills() {
  constexpr Word count = 8;
  constexpr Word base = 16;
  constexpr Word separation = 16;
  constexpr std::array code{body[0],
                            body[0] | (1U << 8) | (1U << 13),
                            body[0] | (2U << 8) | (2U << 13),
                            body[0] | (3U << 8) | (3U << 13),
                            product,
                            rounding,
                            pair | postincrements | store_registers};
  std::array<Word, memory_words> program{};
  Data x{};
  Data y{};
  for (Word index = 0; index < count; ++index) {
    x[base + (3 * separation) + index] = index + 1;
    y[base + (3 * separation) + index] = sign24 / 2;
  }
  auto expected_x = x;
  auto expected_y = y;
  auto initial = initial_state();
  initial.b = register_model::seed_accumulator_b;
  initial.sr = loop_flag | flag_c;
  initial.la = start + static_cast<Word>(code.size()) - 1;
  initial.lc = count;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  initial.stack[1] = (std::uint64_t{previous_end} << word_bits) | previous_count;
  for (unsigned index = 0; index < 4; ++index) {
    initial.r[index] = initial.r[index + 4] = base + (index * separation);
  }
  auto expected = initial;
  for (Word index = 0; index < count; ++index) {
    expected.x = x[base + (3 * separation) + index];
    expected.y = y[base + (3 * separation) + index];
    auto old = expected;
    sequence_model::alu(expected, old, product);
    old = expected;
    sequence_model::alu(expected, old, rounding);
    expected_x[base + 1 + (2 * index)] = register_model::bus_word(expected, expected.a);
    expected_y[base + 1 + (2 * index)] = register_model::bus_word(expected, expected.b);
  }
  for (unsigned index = 0; index < agu_registers; ++index) {
    expected.r[index] += count * ((index == 0 || index == 4) ? 2U : 1U);
  }
  expected.pc = start + static_cast<Word>(code.size());
  expected.la = previous_end;
  expected.lc = previous_count;
  expected.sp = expected.sc = 0;
  expected.sr &= ~loop_bits;
  std::ranges::copy(code, program.begin() + start);
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  core.set_registers(initial);
  const Word budget = count * code.size();
  const auto result = core.run(budget);
  require(result.retired == budget && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = budget, .estimated_cycles = budget} &&
              core.registers() == expected && x == expected_x && y == expected_y,
          "Eight independent address roots retain native spills and second-packet induction "
          "without a host-register cap");
}

void cases() {
  tail_guard();
  root_spills();
  for (const bool shared : {false, true}) {
    shapes(shared);
    Machine machine(shared);
    for (const Word mode : {0U, mode_rm, mode_down, mode_up}) {
      placements(machine, {.mode = mode, .shared = shared});
    }
    for (const Word modifier : {3U, 18U, 0x8007U}) {
      prefixes(machine, {.count = 9, .modifier = modifier, .shared = shared});
    }
    prefixes(machine, {.count = 1, .modifier = 0, .shared = shared});
    for (const Word stack_count : {0U, 1U}) {
      prefixes(machine, {.count = 1, .stack_count = stack_count, .shared = shared});
    }
    for (const Word saved : {loop_flag, forever_flag, loop_bits}) {
      prefixes(machine, {.count = 4, .saved = saved, .shared = shared});
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::cases();
    std::cout << dsp56300::test::checks << " independent native region/prefix checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
