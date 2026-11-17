// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace dsp56300;
constexpr Word entry = 16, terminal = 18, body = 20, last = 21, width = 64;
constexpr Word loaded_word = 0x654321, pointer_mask = 63;
constexpr Word stop = static_cast<Word>(isa::Opcode::stop);

struct Form {
  isa::Opcode operation;
  Word source = 0;
};

constexpr std::array forms{Form{.operation = isa::Opcode::do_immediate},
                           Form{.operation = isa::Opcode::do_absolute_short},
                           Form{.operation = isa::Opcode::do_effective},
                           Form{.operation = isa::Opcode::do_forever},
                           Form{.operation = isa::Opcode::do_register, .source = 16},
                           Form{.operation = isa::Opcode::do_register, .source = 49},
                           Form{.operation = isa::Opcode::do_register, .source = 59},
                           Form{.operation = isa::Opcode::do_register, .source = 61},
                           Form{.operation = isa::Opcode::do_register, .source = 14},
                           Form{.operation = isa::Opcode::do_register, .source = 15},
                           Form{.operation = isa::Opcode::do_register, .source = 32},
                           Form{.operation = isa::Opcode::do_register, .source = 63},
                           Form{.operation = isa::Opcode::do_register, .source = 57},
                           Form{.operation = isa::Opcode::do_register, .source = 60}};
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_down | test::mode_rm,
                           test::mode_up | test::mode_rm};

struct Input {
  Form form;
  Word count;
  Word opcode;
};

std::vector<State> model(Registers initial, const Input input) {
  const auto form = input.form;
  auto count = input.count;
  std::vector<State> states{State{
      .registers = initial, .failure = {}, .execution = ExecutionState::running, .progress = {}}};
  auto s = states.back();
  const auto retire = [&](Word pc, unsigned cost) {
    s.registers.pc = pc;
    ++s.progress.instructions;
    s.progress.estimated_cycles += cost;
    states.push_back(s);
  };
  s.registers.x = (s.registers.x & ~std::uint64_t{word_mask}) | loaded_word;
  retire(entry + 1, 1);
  retire(terminal, 1);
  auto &r = s.registers;
  r.pc = body;
  const auto fail = [&](FaultKind kind, Word value) {
    s.failure = {
        .kind = kind, .space = Space::program, .address = terminal, .value = value, .pc = terminal};
    states.push_back(s);
  };
  const auto push = [&](Word high, Word low) {
    if (r.sc >= stack_entries - 1) {
      fail(FaultKind::stack, r.sp);
      return false;
    }
    r.sp = (r.sp + 1) % 64;
    ++r.sc;
    r.stack[r.sp % stack_entries] = (std::uint64_t{high} << test::word_bits) | low;
    return true;
  };
  if (!push(r.la, r.lc)) { return states; }
  const bool forever = form.operation == isa::Opcode::do_forever;
  if (forever) {
    count = r.lc;
  } else if (form.operation == isa::Opcode::do_register) {
    if (form.source == test::register_model::reg_ssh) {
      fail(FaultKind::invalid_instruction, input.opcode);
      return states;
    }
    count = test::register_model::read(r, form.source);
  }
  if (!forever && count == 0) {
    r.sp = (r.sp + pointer_mask) % 64;
    --r.sc;
    r.pc = last + 1;
  } else {
    r.lc = count;
    if (!push(body, r.sr)) { return states; }
    r.la = last;
    r.sr = (r.sr & ~(1U << 16)) | (1U << 15) | (forever ? 1U << 16 : 0U);
  }
  retire(r.pc, forever ? 4 : 5);
  s.execution = ExecutionState::stopped;
  retire(r.pc + 1, 10);
  return states;
}

void compare(Core &core, const Registers &initial, const std::vector<State> &expected) {
  const auto check = [&](const RunResult &result, std::size_t index) {
    const auto actual = core.state();
    const auto &wanted = expected[index];
    test::require(
        actual.registers == wanted.registers && actual.progress == wanted.progress &&
            actual.execution == wanted.execution && actual.failure == wanted.failure &&
            result.retired == wanted.progress.instructions && result.progress == wanted.progress &&
            result.state == wanted.execution && result.fault == wanted.failure && !result.yielded,
        "Terminal DO preserves every complete prefix and the original partial fault effects");
  };
  for (unsigned budget = 0; budget <= 5; ++budget) {
    core.reset();
    core.set_registers(initial);
    check(core.run(budget), std::min<std::size_t>(budget, expected.size() - 1));
  }
  for (std::uint64_t deadline = 0; deadline <= expected.back().progress.estimated_cycles + 1;
       ++deadline) {
    core.reset();
    core.set_registers(initial);
    std::size_t index = 0;
    while (index + 1 < expected.size() && expected[index].progress.estimated_cycles < deadline) {
      ++index;
    }
    check(core.run_until(deadline), index);
  }
}

Word encoding(const Form form, const Word count, const bool relative) {
  Word opcode = static_cast<Word>(form.operation);
  if (form.operation == isa::Opcode::do_forever) {
    opcode -= relative ? 1U : 0U;
  } else {
    opcode |= relative ? 16U : 0U;
    if (form.operation == isa::Opcode::do_immediate) { opcode |= count << 8; }
    if (form.operation == isa::Opcode::do_register) { opcode |= form.source << 8; }
    if (form.operation == isa::Opcode::do_absolute_short) { opcode |= 1U << 8; }
    if (form.operation == isa::Opcode::do_effective) { opcode |= 4U << 11; }
  }
  return opcode;
}

void exercise(const Form form, const Word count, const bool relative) {
  const auto opcode = encoding(form, count, relative);
  std::array<Word, last - entry + 2> code;
  code.fill(stop);
  code[0] = static_cast<Word>(isa::Opcode::move_x_absolute_short) | (1U << 15);
  code[1] = static_cast<Word>(isa::Opcode::nop);
  code[2] = opcode;
  code[3] = relative ? last - terminal : last;
  const std::array spans{CodeSpan{.address = entry, .words = code}};
  const std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array memory{RegionLayout{.space = Space::program, .first = 0, .size = width},
                          RegionLayout{.space = Space::x, .first = 0, .size = 2},
                          RegionLayout{.space = Space::y, .first = 0, .size = 2}};
  const std::array layouts{VariantLayout{.id = 0, .memory = memory}};
  const auto image = Core::compile({.variants = variants},
                                   {.memory = layouts,
                                    .interrupts = {},
                                    .transitions = {},
                                    .callbacks = {},
                                    .bindings = {},
                                    .region_modes = std::span<const Word>(modes)});
  std::array<Word, width> p{};
  std::array<Word, 2> x{loaded_word, count};
  std::array<Word, 2> y{};
  std::ranges::copy(code, p.begin() + entry);
  Core core(Memory::separate({.program = p, .x = x, .y = y}));
  core.install(image, 0);
  for (const Word mode : modes) {
    for (const Word pointer : {0U, 15U, 63U}) {
      for (const Word depth : {0U, 13U, 14U, 15U, 31U}) {
        auto initial = test::register_model::seed(mode);
        initial.pc = entry;
        initial.sr = mode;
        initial.sp = pointer;
        initial.sc = depth;
        initial.r[0] = form.operation == isa::Opcode::do_register && form.source == 16 ? count : 1U;
        compare(core, initial, model(initial, {.form = form, .count = count, .opcode = opcode}));
      }
    }
  }
}
}

int main() try {
  for (const auto form : forms) {
    for (const Word count : {0U, 2U}) {
      for (const bool relative : {false, true}) { exercise(form, count, relative); }
    }
  }
  std::cout << test::checks << " terminal DO prefix/deadline/stack checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
