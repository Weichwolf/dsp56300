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
#include <optional>
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
  bool shared = false, mirrored = false;
  std::uint8_t input_known = 1, output_known = 1;
  bool deadline = false;
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

class Machine final : public Io {
public:
  explicit Machine(const bool shared,
                   const Shape shape = Shape::product,
                   const bool mapped = false,
                   const Word origin = 0)
      : code_(instructions(shape)),
        windows_{MemoryWindow{.address = origin,
                              .words = std::span(x_).subspan(origin),
                              .known = std::span(x_known_).subspan(origin),
                              .writable = true},
                 MemoryWindow{.address = origin,
                              .words = (shared ? std::span(x_) : std::span(y_)).subspan(origin),
                              .known = std::span(y_known_).subspan(origin),
                              .writable = true}},
        mapped_(mapped),
        core_(mapped ? Memory::external()
                     : Memory::separate({.program = program_,
                                         .x = x_,
                                         .y = shared ? std::span(x_) : std::span(y_)}),
              mapped ? this : nullptr) {
    std::ranges::copy(code_, program_.begin() + start);
    for (std::size_t index = 0; index < x_.size(); ++index) {
      x_[index] = samples[index % samples.size()];
      y_[index] = samples[(3 * index + 2) % samples.size()];
    }
    seed_x_ = x_;
    seed_y_ = y_;
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(&windows_[space == Space::x ? 0 : 1], 1);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program_.size() ? std::optional{program_[address]} : std::nullopt;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    const auto word = inspect_program(address);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  void check(const Point point) {
    if (core_.state().failure.kind != FaultKind::none) { core_.reset(); }
    x_ = seed_x_;
    y_ = seed_y_;
    x_known_.fill(point.input_known);
    y_known_.fill(point.input_known);
    for (Word index = 0; index < point.count && point.output + index < data_words; ++index) {
      x_known_[point.output + index] = y_known_[point.output + index] = point.output_known;
    }
    auto expected_x_known = x_known_;
    auto expected_y_known = y_known_;
    const auto state = registers(point);
    Model model{.state = state, .x = seed_x_, .y = seed_y_, .failure = {}, .code = code_};
    for (Word index = 0; index < point.prefix; ++index) {
      if (model.state.pc == start + body.size() - 1) {
        expected_x_known[model.state.r[1]] = expected_y_known[model.state.r[5]] = 1;
      }
      if (!step(model, point.shared)) { break; }
      ++model.retired;
    }
    core_.set_registers(state);
    const auto before = core_.state().progress;
    const auto result = point.deadline ? core_.run_until(before.estimated_cycles + point.prefix)
                                       : core_.run(point.prefix);
    const auto expected_progress =
        Progress{.instructions = before.instructions + model.retired,
                 .estimated_cycles = before.estimated_cycles + model.retired};
    const auto actual = core_.registers();
    const bool matched =
        result.retired == model.retired && result.fault == model.failure &&
        result.state == ExecutionState::running && !result.yielded && actual == model.state &&
        result.progress == expected_progress && x_ == model.x && y_ == model.y &&
        (!mapped_ || (x_known_ == expected_x_known && y_known_ == expected_y_known));
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

  void refused_access(const Word hole, const bool store, const bool alias = false) {
    constexpr Word count = 33;
    x_ = seed_x_;
    y_ = seed_y_;
    x_known_.fill(1);
    y_known_.fill(1);
    if (store) {
      windows_[0].writable = alias;
      windows_[0].aliases_program = alias;
    } else {
      x_known_[input_base + hole] = 0;
    }
    const auto state = registers({.count = count});
    Model expected{.state = state, .x = x_, .y = y_, .failure = {}, .code = code_};
    const Word retired =
        store ? static_cast<Word>(body.size() - 1) : hole * static_cast<Word>(body.size());
    for (Word index = 0; index < retired; ++index) {
      require(step(expected, false), "The independent prefix completes before its refused access");
    }
    core_.set_registers(state);
    const auto result = core_.run(count * body.size());
    require(result.retired == retired &&
                result.progress == Progress{.instructions = retired, .estimated_cycles = retired} &&
                result.fault.kind == (store ? FaultKind::memory_write : FaultKind::memory_read) &&
                result.fault.space == Space::x && result.fault.pc == (store ? state.la : start) &&
                result.fault.address == (store ? output_base : input_base + hole) &&
                x_ == expected.x && y_ == expected.y,
            "Unknown reads, ROM stores and executable-alias stores retain their precise prefix");
  }

  void metadata_alias() {
    constexpr Word count = 32;
    constexpr Word canonical_word = 0x01010101;
    x_ = seed_x_;
    y_.fill(canonical_word);
    y_known_.fill(1);
    windows_[0].known = {reinterpret_cast<std::uint8_t *>(y_.data() + output_base), data_words};
    const auto state = registers({.count = count});
    Model expected{.state = state, .x = x_, .y = y_, .failure = {}, .code = code_};
    auto knowledge = std::as_writable_bytes(std::span(expected.y))
                         .subspan(output_base * sizeof(Word), data_words);
    Word retired = 0;
    while (retired < count * body.size()) {
      if (expected.state.pc == start && knowledge[expected.state.r[0]] == std::byte{0}) { break; }
      const auto store = expected.state.pc == state.la;
      const auto output = expected.state.r[1];
      require(step(expected, false), "The independent alias prefix has valid loop frames");
      // Metadata marks are beyond the Y output prefix preceding this read fault.
      if (store) { knowledge[output] = std::byte{1}; }
      ++retired;
    }
    require(retired < count * body.size(), "An actual Y store makes a future X input unknown");
    core_.set_registers(state);
    const auto result = core_.run(count * body.size());
    require(result.retired == retired &&
                result.progress == Progress{.instructions = retired, .estimated_cycles = retired} &&
                result.fault.kind == FaultKind::memory_read && result.fault.space == Space::x &&
                result.fault.address == expected.state.r[0] && result.fault.pc == start &&
                x_ == expected.x && y_ == expected.y,
            "Word stores aliasing knowledge metadata preserve ordered unknown-read faults");
  }

private:
  static Registers registers(const Point point) {
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
    return state;
  }

  std::array<Word, memory_words> program_{};
  Data x_{}, y_{}, seed_x_{}, seed_y_{};
  std::array<std::uint8_t, data_words> x_known_{}, y_known_{};
  const std::array<Word, 4> code_;
  std::array<MemoryWindow, 2> windows_;
  bool mapped_;
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

void mapped_cases() {
  constexpr Word packet_tail = 33;
  for (const bool shared : {false, true}) {
    for (const auto shape : {Shape::product,
                             Shape::negative,
                             Shape::word_shift,
                             Shape::wide_shift,
                             Shape::clear,
                             Shape::add,
                             Shape::subtract}) {
      Machine machine(shared, shape, true);
      for (const Word mode : {0U, mode_rm, mode_down, mode_up}) {
        placements(machine, {.mode = mode, .shared = shared});
        for (const Word count : {32U, packet_tail}) {
          prefixes(machine, {.count = count, .mode = mode, .shared = shared});
        }
      }
    }
  }
  Machine based(false, Shape::product, true, 16);
  prefixes(based, {.count = packet_tail});
  constexpr std::uint8_t noncanonical = 0xff;
  for (const auto known : std::array<std::uint8_t, 4>{0, 1, 2, noncanonical}) {
    Machine tracked(false, Shape::product, true);
    prefixes(tracked, {.count = packet_tail, .input_known = noncanonical, .output_known = known});
  }
  for (const Word hole : {0U, 3U, 32U}) {
    Machine guarded(false, Shape::product, true);
    guarded.refused_access(hole, false);
  }
  for (const bool alias : {false, true}) {
    Machine guarded(false, Shape::product, true);
    guarded.refused_access(0, true, alias);
  }
  Machine alias(false, Shape::product, true);
  alias.metadata_alias();
  for (const bool mapped : {false, true}) {
    Machine timed(false, Shape::product, mapped);
    prefixes(timed, {.count = packet_tail, .deadline = true});
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

void tail_guard(const bool load_fault) {
  constexpr std::size_t extent = 64;
  constexpr Word count = 4;
  std::array<Word, memory_words> program{};
  Data x;
  Data y;
  constexpr Word x_guard = 0x13579b;
  constexpr Word y_guard = 0x2468ac;
  x.fill(x_guard);
  y.fill(y_guard);
  const Word source = load_fault ? extent - 3 : input_base;
  const Word target = load_fault ? 16U : extent - 3;
  const Word available = count - static_cast<Word>(load_fault);
  for (Word index = 0; index < available; ++index) {
    x[source + index] = 1;
    y[source + index] = sign24 / 2;
  }
  std::ranges::copy(body, program.begin() + start);
  auto initial = initial_state();
  initial.b = register_model::seed_accumulator_b;
  initial.sr = loop_flag | flag_c;
  initial.la = start + static_cast<Word>(body.size()) - 1;
  initial.lc = count;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  initial.r[0] = initial.r[4] = source;
  initial.r[1] = initial.r[5] = target;
  Core core(Memory::separate(
      {.program = program, .x = std::span(x).first(extent), .y = std::span(y).first(extent)}));
  core.set_registers(initial);
  const Word budget = count * body.size();
  const auto result = core.run(budget);
  const Word retired = budget - (load_fault ? static_cast<Word>(body.size()) : 1U);
  const auto kind = load_fault ? FaultKind::memory_read : FaultKind::memory_write;
  const auto pc = load_fault ? start : initial.la;
  require(result.retired == retired &&
              result.progress == Progress{.instructions = retired, .estimated_cycles = retired} &&
              result.fault.kind == kind && result.fault.space == Space::x &&
              result.fault.address == extent && result.fault.pc == pc && x[extent] == x_guard &&
              y[extent] == y_guard,
          "A packet crossing its RAM extent retains the precise first failed access and untouched "
          "host guards");
  for (std::size_t index = target; index < target + 3; ++index) {
    require(x[index] == 0 && y[index] == sign24,
            "Completed writes before the RAM fault remain committed");
  }
}

void root_spills(const bool repeated) {
  constexpr Word count = 8;
  constexpr Word base = 16;
  constexpr Word separation = 16;
  const std::array code{body[0],
                        body[0] | (1U << 8) | (1U << 13),
                        body[0] | (2U << 8) | (2U << 13),
                        body[0] | (3U << 8) | (3U << 13),
                        repeated ? body[0] | (3U << 8) | (3U << 13)
                                 : static_cast<Word>(isa::Opcode::nop),
                        product,
                        rounding,
                        pair | postincrements | store_registers};
  std::array<Word, memory_words> program{};
  Data x{};
  Data y{};
  const Word stride = repeated ? 2U : 1U;
  for (Word index = 0; index < count * stride; ++index) {
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
    const auto input = base + (3 * separation) + (index * stride) + stride - 1;
    expected.x = x[input];
    expected.y = y[input];
    auto old = expected;
    sequence_model::alu(expected, old, product);
    old = expected;
    sequence_model::alu(expected, old, rounding);
    expected_x[base + 1 + (2 * index)] = register_model::bus_word(expected, expected.a);
    expected_y[base + 1 + (2 * index)] = register_model::bus_word(expected, expected.b);
  }
  for (unsigned index = 0; index < agu_registers; ++index) {
    const auto read_advances = (index == 3 || index == 7) ? stride : 1U;
    expected.r[index] += count * ((index == 0 || index == 4) ? 2U : read_advances);
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

void nested_finish(Registers &state, const Word last) {
  if (last != state.la || state.pc != state.la + 1) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = static_cast<Word>(state.stack[state.sp % stack_entries] >> word_bits);
    return;
  }
  const auto saved = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  const auto outer = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  state.sc -= 2;
  state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
  state.la = static_cast<Word>(outer >> word_bits);
  state.lc = static_cast<Word>(outer) & word_mask;
}

using NestedData = std::array<Word, 4096>;

enum class NestedShape : unsigned char { reset, independent, feedback, strided };
enum class ChildVariant : unsigned char { ordinary, negative, absolute };

struct NestedModel {
  Registers state;
  NestedData x;
  NestedData y;
  NestedShape shape = NestedShape::reset;
  bool mirrored = false;
  Word arithmetic = product;
};

Word nested_step(NestedModel &model, const Word child) {
  auto &state = model.state;
  const auto offset = state.pc - start;
  Word words = 1;
  Word cycles = 1;
  if (offset == 0 || offset == 2) {
    constexpr std::array inputs{0U, 4U};
    constexpr std::array unused{6U, 7U};
    const auto index = offset / 2;
    state.r[model.shape != NestedShape::reset ? unused[index] : inputs[index]] = input_base;
    words = cycles = 2;
  } else if (offset == 4) {
    const auto push = [&](const Word high, const Word low) {
      state.sp = (state.sp + 1) & pointer_mask;
      ++state.sc;
      state.stack[state.sp % stack_entries] = (std::uint64_t{high} << word_bits) | low;
    };
    push(state.la, state.lc);
    push(state.pc + 2, state.sr);
    state.la = start + 9;
    state.lc = child;
    words = 2;
    cycles = 5;
  } else if (offset == 6) {
    state.x = (state.x & ~std::uint64_t{word_mask}) | (model.x[state.r[0]] & word_mask);
    state.y = (state.y & ~std::uint64_t{word_mask}) | (model.y[state.r[4]] & word_mask);
    state.r[0] += state.n[0];
    state.r[4] += state.n[4];
  } else if (offset == 7 || offset == 8) {
    const auto old = state;
    sequence_model::alu(state, old, offset == 7 ? model.arithmetic : rounding);
  } else if (offset == 9) {
    model.x[state.r[1]] = register_model::bus_word(state, state.a);
    model.y[state.r[5]] = register_model::bus_word(state, model.mirrored ? state.a : state.b);
    state.r[1] += model.shape == NestedShape::strided ? state.n[1] : 1U;
    state.r[5] += model.shape == NestedShape::strided ? state.n[5] : 1U;
  } else {
    constexpr std::array<unsigned, 4> post{0, 4, 1, 5};
    const auto index = post.at(offset - 10);
    state.r[index] += state.n[index];
  }
  const auto last = state.pc + words - 1;
  state.pc += words;
  nested_finish(state, last);
  return cycles;
}

Word child_product(const ChildVariant variant) {
  switch (variant) {
    case ChildVariant::ordinary: return product;
    case ChildVariant::negative: return product | 4U;
    case ChildVariant::absolute: return static_cast<Word>(isa::Opcode::abs_a);
    default: throw std::logic_error("Unknown child operation");
  }
}

Word set_child_product(Core &core, const ChildVariant variant, const unsigned warmth) {
  if (variant == ChildVariant::ordinary) { return product; }
  const auto opcode = warmth == 0 ? product : child_product(variant);
  const std::array changed{opcode};
  core.load(Space::program, start + 7, changed);
  return opcode;
}

void nested_case(const Word child,
                 const Point point,
                 const NestedShape shape = NestedShape::reset,
                 const ChildVariant variant = ChildVariant::ordinary) {
  constexpr Word immediate = static_cast<Word>(isa::Opcode::move_long) | (6U << 11) | (4U << 8) |
                             (Word{1} << 14) | (Word{1} << 15);
  constexpr Word low_count_mask = 255;
  const Word setup_x = shape != NestedShape::reset ? 22U : 16U;
  const Word setup_y = shape != NestedShape::reset ? 23U : 20U;
  constexpr unsigned high_mask = 24;
  constexpr unsigned high_shift = 17;
  const auto address_word = [](const Word reg) {
    return immediate | ((reg & 7U) << 16) | ((reg & high_mask) << high_shift);
  };
  const Word update = static_cast<Word>(isa::Opcode::move_address_update) | (1U << 11);
  constexpr Word post_offsets = (1U << 11) | (1U << 20);
  Word store = shape == NestedShape::strided ? (body[3] & ~postincrements) | post_offsets : body[3];
  if (point.mirrored) { store &= ~(Word{1} << 16); }
  const std::array code{address_word(setup_x),
                        input_base,
                        address_word(setup_y),
                        input_base,
                        static_cast<Word>(isa::Opcode::do_immediate) |
                            ((child & low_count_mask) << 8) | (child >> 8),
                        start + 9,
                        (body[0] & ~postincrements) | (1U << 11) | (1U << 20),
                        product,
                        rounding,
                        store,
                        update,
                        update | (4U << 8),
                        update | (1U << 8),
                        update | (5U << 8)};
  std::array<Word, memory_words> program{};
  std::ranges::copy(code, program.begin() + start);
  NestedData x{};
  NestedData y{};
  NestedData seed_x{};
  NestedData seed_y{};
  for (std::size_t index = 0; index < x.size(); ++index) {
    seed_x[index] = samples[index % samples.size()];
    seed_y[index] = samples[(3 * index + 2) % samples.size()];
  }
  auto initial = register_model::seed(point.mode);
  initial.sr |= loop_flag;
  initial.pc = start;
  initial.la = start + static_cast<Word>(code.size()) - 1;
  initial.lc = point.count;
  initial.sp = point.pointer;
  initial.sc = 2;
  initial.r[1] = initial.r[5] = point.output;
  initial.n.fill(1);
  initial.n[4] = shape == NestedShape::feedback ? 1U : 2U;
  if (shape == NestedShape::strided) {
    for (const unsigned index : {0U, 1U, 4U, 5U}) { initial.n[index] = 2; }
  }
  if (shape != NestedShape::reset) { initial.r[0] = initial.r[4] = input_base; }
  if (shape == NestedShape::feedback) { initial.r[1] = initial.r[5] = input_base + 1; }
  initial.stack[point.pointer % stack_entries] = std::uint64_t{start} << word_bits;
  initial.stack[(point.pointer + pointer_mask) % stack_entries] =
      (std::uint64_t{previous_end} << word_bits) | previous_count;
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  const Word budget = point.count * ((4 * child) + 7);
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    const auto arithmetic = set_child_product(core, variant, warmth);
    for (Word prefix = 0; prefix <= budget; ++prefix) {
      const auto parent_size = (4 * child) + 7;
      const auto position = prefix % parent_size;
      // Exhaust small nests; retain boundary and interior cuts for large nests.
      if (child > 8 && position > 6 && position < parent_size - 6 && position != parent_size / 2) {
        continue;
      }
      NestedModel expected{.state = initial,
                           .x = seed_x,
                           .y = seed_y,
                           .shape = shape,
                           .mirrored = point.mirrored,
                           .arithmetic = arithmetic};
      Word cycles = 0;
      for (Word index = 0; index < prefix; ++index) { cycles += nested_step(expected, child); }
      x = seed_x;
      y = seed_y;
      core.set_registers(initial);
      const auto before = core.state().progress;
      const auto result = core.run(prefix);
      const auto actual = core.registers();
      if (actual != expected.state) {
        std::cerr << "child=" << child << " parents=" << point.count << " prefix=" << prefix
                  << " R0=" << actual.r[0] << '/' << expected.state.r[0] << " R4=" << actual.r[4]
                  << '/' << expected.state.r[4] << '\n';
      }
      require(result.retired == prefix && result.fault.kind == FaultKind::none &&
                  actual == expected.state && x == expected.x && y == expected.y &&
                  result.progress == Progress{.instructions = before.instructions + prefix,
                                              .estimated_cycles = before.estimated_cycles + cycles},
              "Every nested-loop prefix retains complete state, reset-root induction, post "
              "updates and both active/inactive stack frames");
    }
  }
}

void nested_cases() {
  for (const Word child : {1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U, 512U}) {
    for (const Word parents : {2U, 3U, 4U, 5U}) {
      for (const Word pointer : {0U, 14U, pointer_mask}) {
        for (const Word mode : {0U, mode_rm}) {
          const Word output = (parents & 1U) != 0 ? 1280U : output_base;
          nested_case(child,
                      {.count = parents, .mode = mode, .pointer = pointer, .output = output});
        }
      }
    }
  }
}

void mixed_positions(Machine &machine, const bool shared) {
  constexpr std::array<std::uint64_t, 2> accumulators{std::uint64_t{1} << 21,
                                                      std::uint64_t{1} << 45};
  for (const auto accumulator : accumulators) {
    for (const Word count : {1U, 4U, 5U}) {
      prefixes(machine, {.count = count, .accumulator_b = accumulator, .shared = shared});
    }
  }
}

void mirrored_nested_cases() {
  constexpr Word advancing_output_base = 1536;
  for (const auto shape : {NestedShape::reset, NestedShape::independent, NestedShape::strided}) {
    for (const Word child : {1U, 2U, 4U, 8U, 64U, 128U, 512U}) {
      if (child > 128 && shape != NestedShape::reset) { continue; }
      for (const Word parents : {2U, 3U, 4U, 5U}) {
        const Word output = child > 128 && (parents & 1U) != 0 ? 1280U : advancing_output_base;
        for (const Word mode : {0U, mode_rm, flag_s, mode_rm | flag_s}) {
          nested_case(child,
                      {.count = parents,
                       .mode = mode,
                       .pointer = pointer_mask,
                       .output = output,
                       .mirrored = true},
                      shape);
        }
      }
    }
  }
}

void cases() {
  mapped_cases();
  constexpr Word advancing_output_base = 1536;
  for (const auto variant : {ChildVariant::negative, ChildVariant::absolute}) {
    nested_case(4, {.count = 8, .output = advancing_output_base}, NestedShape::reset, variant);
  }
  nested_cases();
  mirrored_nested_cases();
  for (const auto shape : {NestedShape::independent, NestedShape::feedback, NestedShape::strided}) {
    for (const Word child : {4U, 8U, 16U, 64U, 128U}) {
      for (const Word parents : {2U, 3U, 4U, 5U}) {
        for (const Word pointer : {0U, pointer_mask}) {
          for (const Word mode : {0U, mode_rm}) {
            nested_case(child,
                        {.count = parents,
                         .mode = mode,
                         .pointer = pointer,
                         .output = advancing_output_base},
                        shape);
          }
        }
      }
    }
  }
  tail_guard(false);
  tail_guard(true);
  root_spills(false);
  root_spills(true);
  for (const bool shared : {false, true}) {
    shapes(shared);
    Machine machine(shared);
    mixed_positions(machine, shared);
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
