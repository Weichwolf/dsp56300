// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "move_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "reject_io.hpp"
#include "selection_model.hpp"
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
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
template <isa::Opcode value> constexpr Word op = static_cast<Word>(value);
constexpr Word first = 16, base = 0x4000, turns = 3, loop_flag = 1U << 15;
constexpr std::size_t words = 64, form_count = 128;
constexpr Word linear = 0xffff, wrapped = 0x8007, previous_la = 300, linear_cursor = 24;
constexpr Word n0_register = register_model::reg_n_base;
constexpr unsigned ordinary_extra_forms = 28, extra_forms = 30;
constexpr Word immediate_word = 0xfedcba, cursor = 8, stride = 3, outside = 80;
constexpr Word host_tag = 0xa0000000, mac = op<isa::Opcode::mac_register_a> | (5U << 4);
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array statuses{
    0U, status_byte, flag_n | flag_v, flag_z | flag_c, flag_e | flag_u, flag_s | flag_l};
using Data = std::array<Word, words>;
enum class Setup : std::uint8_t { immediate, memory, stride };

struct Control {
  unsigned selector, mode;
  bool publish;
  Setup setup = Setup::immediate;
  bool loop = false, conditional_only = false;
};

struct Input {
  move_model::Form form;
  Word alu = 0;
};

constexpr Word memory_move(const Word form, const unsigned reg) {
  constexpr Word fields = 0x370000;
  constexpr Word high = 0x18;
  constexpr unsigned high_shift = 17;
  return (form & ~fields) | ((reg & 7U) << 16) | ((reg & high) << high_shift);
}

struct Program {
  explicit Program(const Input input) : input(input) {
    code = {op<isa::Opcode::do_immediate> | (turns << 8),
            first + move_model::words(input.form),
            move_model::opcode(input.form, input.alu)};
    if (move_model::words(input.form) == 2) { code.push_back(input.form.extension); }
    code.insert(code.end(), {op<isa::Opcode::nop>, op<isa::Opcode::nop>});
  }

  explicit Program(const Control control) : control(control) {
    Word setup = op<isa::Opcode::move_immediate_r0> | (cursor << 8);
    if (control.setup == Setup::memory) {
      setup = memory_move(op<isa::Opcode::move_x_absolute_short>, 16) | (1U << 15) | (2U << 8);
    } else if (control.setup == Setup::stride) {
      setup = op<isa::Opcode::move_immediate_n0> | (stride << 8);
    }
    code = {op<isa::Opcode::move_x_absolute_short> | (1U << 15) | (1U << 8),
            (control.publish ? op<isa::Opcode::ifcc_u> : op<isa::Opcode::ifcc>) |
                (control.selector << 8) | (op<isa::Opcode::add_x1_a> & status_byte),
            setup | (op<isa::Opcode::asr_b> & status_byte),
            op<isa::Opcode::move_address_update> | (control.mode << 11) | (mac & status_byte),
            memory_move(op<isa::Opcode::move_x_effective>, 5) | (1U << 15) | (4U << 11),
            memory_move(op<isa::Opcode::move_y_absolute_short>, 14) | (2U << 8),
            op<isa::Opcode::move_address_update> | (control.mode << 11) | (4U << 8)};
    if (control.conditional_only) { code.resize(2); }
    code.push_back(op<isa::Opcode::stop>);
    if (control.loop) {
      code.insert(code.begin(), {op<isa::Opcode::do_immediate> | (turns << 8), first + 6});
    }
  }

  [[nodiscard]] bool looped() const { return !control || control->loop; }

  [[nodiscard]] Word address() const { return looped() ? first - 2 : first; }

  [[nodiscard]] Word count() const { return !control || control->conditional_only ? 2 : 7; }

  [[nodiscard]] Word limit() const { return count() * (looped() ? turns : 1); }

  [[nodiscard]] Word last() const {
    return first + (control ? count() - 1 : move_model::words(input.form));
  }

  Input input{};
  std::optional<Control> control;
  std::vector<Word> code;
};

struct Port final : RejectIo {
  explicit Port(const Program &program) : code(program.code) {
    windows = {MemoryWindow{.address = base, .words = x, .known = known, .writable = true},
               MemoryWindow{.address = base, .words = y, .known = known, .writable = true}};
    known.fill(1);
    std::ranges::copy(code, p.begin() + program.address());
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc >= first - 2 && pc - (first - 2) < code.size() ? std::optional{code[pc - (first - 2)]}
                                                             : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return space == Space::program ? std::span<const MemoryWindow>{}
                                   : std::span(&windows[space == Space::x ? 0U : 1U], 1);
  }

  Data p{}, x{}, y{};
  std::array<std::uint8_t, words> known{};
  std::array<MemoryWindow, 2> windows{};
  std::span<const Word> code;
};

struct Point {
  Word mode, modifier;
  std::uint64_t accumulator = 0;
  Word quota = 0;
  bool deadline = false;
  Word status = 0, offset = stride;
  bool fault = false;
};

Registers initialize(Port &port, const Program &program, const Point point) {
  auto expected = register_model::seed(point.mode);
  expected.pc = first;
  for (Word i = 0; i < words; ++i) {
    port.x[i] =
        program.control ? host_tag | (word_mask ^ i) : (i * (sign24 / 8 - 3)) ^ (sign24 + 1);
    port.y[i] = program.control ? 0 : (i * (sign24 / 16 - 1)) ^ (sign24 - 1);
  }
  if (program.control) {
    port.x[1] = host_tag | ((point.status & flag_c) != 0 ? 1U : word_mask - 1);
    port.x[2] = host_tag | cursor;
    expected.m.fill(point.modifier);
    if (point.fault) {
      port.x.fill(0);
      port.x[1] = 1;
      port.x[2] = cursor;
      expected.n[0] = outside;
      return expected;
    }
    expected.sr = point.mode | point.status;
    expected.a = (point.status & flag_c) != 0 ? sign56 - 2 : sign56 + 2;
    expected.n.fill(point.offset);
    if (program.control->setup == Setup::stride) { expected.r[0] = cursor; }
    expected.r[4] = point.modifier == linear ? word_mask : cursor + 7;
  } else {
    expected.sr = point.mode;
    expected.a = point.accumulator;
    expected.b = accumulator_mask ^ point.accumulator;
    expected.stack[1] = (std::uint64_t{previous_la} << word_bits) | 7U;
    expected.r[0] = base + (point.modifier == linear ? linear_cursor : 7U);
    expected.n[0] = stride;
    expected.m[0] = point.modifier;
  }
  if (program.looped()) {
    expected.stack[2] = (std::uint64_t{first} << word_bits) | expected.sr;
    expected.sr |= loop_flag;
    expected.la = program.last();
    expected.lc = turns;
  }
  return expected;
}

void control_step(Registers &state, const Data &x, Data &y, const Control control) {
  const auto old = state;
  switch (state.pc - first) {
    case 0: register_model::write(state, 4, x[1] & word_mask); break;
    case 1:
      if (selection_model::test_condition({.status = old.sr, .selector = control.selector})) {
        sequence_model::alu(state, old, op<isa::Opcode::add_x1_a>);
        if (!control.publish) { state.sr = old.sr; }
      }
      break;
    case 2:
      register_model::write(
          state,
          control.setup == Setup::stride ? n0_register : 16,
          std::array{cursor, x[2] & word_mask, stride}[static_cast<std::size_t>(control.setup)]);
      sequence_model::alu(state, old, op<isa::Opcode::asr_b>);
      break;
    case 3:
    case 6: {
      const unsigned reg = state.pc == first + 3 ? 0 : 4;
      state.r[reg] = address_model::updated({.modifier = old.m[reg],
                                             .cursor = old.r[reg],
                                             .offset = old.n[reg],
                                             .mode = control.mode});
      if (reg == 0) { sequence_model::alu(state, old, mac); }
      break;
    }
    case 4: register_model::write(state, 5, x.at(old.r[0]) & word_mask); break;
    case 5: y[2] = register_model::read(state, 14); break;
    default: throw std::logic_error("Invalid authored control step");
  }
  ++state.pc;
}

Word step(Registers &state, Data &x, Data &y, const Program &program) {
  Word cycles = 1;
  if (program.control) {
    // FM table A-1 (p. A-6): the short absolute store costs two cycles.
    cycles = state.pc == first + 5 ? 2 : 1;
    control_step(state, x, y, *program.control);
  } else if (state.pc == first) {
    const auto old = state;
    const auto form = program.input.form;
    auto probe = state;
    const auto address = move_model::address(probe, form);
    auto &data = form.y ? y : x;
    const auto value =
        form.mode == 6 && form.index == 4 ? form.extension : data.at(address - base) & word_mask;
    const auto output = move_model::apply(state, form, {.high = value});
    if (output.stores != 0) { data.at(output.address - base) = output.first; }
    sequence_model::alu(state, old, program.input.alu);
    state.pc += move_model::words(form);
    cycles = move_model::cycles(form);
  } else {
    ++state.pc;
  }
  if (program.looped() && state.pc == program.last() + 1) {
    if (state.lc > 1) {
      --state.lc;
      state.pc = first;
    } else {
      loop_frames::complete(state);
    }
  }
  return cycles;
}

void execute(const NativeImage &image,
             const Program &program,
             const std::uint64_t id,
             const Point point) {
  Port port(program);
  Core core(program.control ? Memory::separate({.program = port.p, .x = port.x, .y = port.y})
                            : Memory::external(),
            program.control ? nullptr : &port);
  auto expected = initialize(port, program, point);
  core.set_registers(expected);
  core.install(image, id);
  const auto p = port.p;
  auto x = port.x;
  auto y = port.y;
  Word completed = 0;
  Word cycles = 0;
  const auto check = [&](const Word target, const bool deadline) {
    const auto before = completed;
    while (completed < target) {
      cycles += step(expected, x, y, program);
      ++completed;
    }
    Fault fault;
    if (point.fault) {
      fault = {.kind = FaultKind::memory_read,
               .space = Space::x,
               .address = cursor + outside,
               .pc = first + 4};
      ++expected.pc; // Failed operand read advances PC without instruction/cycle retirement.
    }
    const auto run = deadline ? core.run_until(cycles)
                              : core.run(point.fault ? program.count() : target - before);
    const bool correct =
        run.retired == target - before && run.fault == fault && !run.yielded &&
        run.progress == Progress{.instructions = target, .estimated_cycles = cycles} &&
        core.registers() == expected && port.p == p && port.x == x && port.y == y &&
        port.reads == 0 && port.writes == 0 && port.bits == 0;
    if (!correct) {
      std::cerr << "id/mode/modifier/status/offset/quota/deadline=" << id << '/' << point.mode
                << '/' << point.modifier << '/' << point.status << '/' << point.offset << '/'
                << target << '/' << deadline << '\n';
    }
    require(correct,
            "Parallel regions preserve complete registers, flags, P/X/Y, AGU, fault context and "
            "every instruction/time boundary");
  };
  check(point.quota, point.deadline);
  if (program.control && !point.fault) { check(program.limit(), true); }
}

void qualify(const NativeImage &image, const Program &program, const std::uint64_t id) {
  constexpr std::array accumulators{
      std::uint64_t{0}, std::uint64_t{1} << 47, sign56 - 1, sign56, accumulator_mask};
  const auto values = program.control ? statuses.size() : accumulators.size();
  const Word first_quota = program.control ? 0 : 1;
  const auto quotas = program.limit() - first_quota + 1;
  // Mixed-radix rows retain the full Cartesian matrix without nested test runners.
  for (std::size_t row = 0; row < modes.size() * 2 * values * quotas * 2; ++row) {
    const auto value = (row / 12) % values;
    const auto choice = row / (12 * values);
    execute(image,
            program,
            id,
            {.mode = modes[row % modes.size()],
             .modifier = (row / 6) % 2 == 0 ? linear : wrapped,
             .accumulator = program.control ? 0 : accumulators[value],
             .quota = static_cast<Word>(choice % quotas) + first_quota,
             .deadline = !program.control && choice / quotas != 0,
             .status = program.control ? statuses[value] : 0,
             .offset = choice / quotas != 0 ? word_mask - stride + 1 : stride});
  }
  if (program.control && program.control->selector == 0 && program.control->mode == 1 &&
      program.control->publish && program.control->setup != Setup::stride && !program.looped() &&
      !program.control->conditional_only) {
    execute(image, program, id, {.mode = 0, .modifier = linear, .quota = 4, .fault = true});
  }
}

Input input(const unsigned ordinal) {
  const bool y = (ordinal & 1U) != 0;
  const auto alu =
      static_cast<Word>(y ? isa::Opcode::add_x0_a : isa::Opcode::add_y0_a) & status_byte;
  return {.form = {.kind = move_model::Kind::mixed_one,
                   .selector = (ordinal >> 2) & 3U,
                   .mode = 3,
                   .load = (ordinal & 2U) != 0,
                   .y = y,
                   .source_b = (ordinal & 16U) != 0,
                   .second_high = (ordinal & 32U) != 0},
          .alu = (ordinal & 64U) != 0 ? alu : 0};
}

std::vector<Program> programs() {
  std::vector<Program> result;
  result.reserve(2 * form_count);
  for (unsigned ordinal = 0; ordinal < form_count; ++ordinal) {
    result.emplace_back(input(ordinal));
  }
  constexpr std::array address_modes{0U, 1U, 2U, 4U, 5U, 6U, 7U};
  for (unsigned row = 0; row < extra_forms; ++row) {
    move_model::Form form{.kind = move_model::Kind::mixed_one,
                          .selector = 2,
                          .mode = 6,
                          .index = 4,
                          .extension = immediate_word,
                          .y = row % 2 != 0,
                          .source_b = true};
    if (row < ordinary_extra_forms) {
      form.mode = address_modes[row % address_modes.size()];
      form.index = 0;
      form.extension = base + 12;
      form.load = (row / 7) % 2 != 0;
      form.y = row / 14 != 0;
    }
    result.emplace_back(Input{.form = form});
  }
  for (unsigned selector = 0; selector < 16; ++selector) {
    for (const bool publish : {false, true}) {
      result.emplace_back(Control{.selector = selector, .mode = selector % 4, .publish = publish});
      result.emplace_back(
          Control{.selector = selector, .mode = 0, .publish = publish, .conditional_only = true});
    }
  }
  for (const auto setup : {Setup::memory, Setup::stride}) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      result.emplace_back(Control{.selector = 0, .mode = mode, .publish = true, .setup = setup});
    }
  }
  result.emplace_back(Control{.selector = 0, .mode = 1, .publish = true, .loop = true});
  result.emplace_back(
      Control{.selector = 7, .mode = 0, .publish = false, .setup = Setup::memory, .loop = true});
  return result;
}
}

int main() try {
  const auto cases = programs();
  const std::array entries{first, first - 2};
  const std::array mapped{
      RegionLayout{.space = Space::x, .first = base, .size = words, .tracked = true},
      RegionLayout{.space = Space::y, .first = base, .size = words, .tracked = true}};
  const std::array separate{RegionLayout{.space = Space::program, .first = 0, .size = words},
                            RegionLayout{.space = Space::x, .first = 0, .size = words},
                            RegionLayout{.space = Space::y, .first = 0, .size = words}};
  std::vector<CodeSpan> spans;
  spans.reserve(cases.size());
  std::vector<CodeVariant> variants;
  std::vector<VariantLayout> layouts;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto &program = cases[i];
    spans.push_back({.address = program.address(), .words = program.code});
    variants.push_back({.id = i,
                        .code = std::span(&spans.back(), 1),
                        .entries = std::span(entries).first(program.looped() ? 2U : 1U),
                        .failed_fetches = {}});
    layouts.push_back({.id = i,
                       .memory = program.control ? std::span<const RegionLayout>(separate)
                                                 : std::span<const RegionLayout>(mapped)});
  }
  const auto image = Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
  for (std::size_t i = 0; i < cases.size(); ++i) { qualify(image, cases[i], i); }
  std::cout << cases.size() << " parallel images, " << checks << " checks\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
