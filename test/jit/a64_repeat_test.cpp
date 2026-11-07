// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using State = loop_model::State;
constexpr Word body_pc = start + 1, saved_lc = 0x357;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array counts{0U, 1U, 2U, 3U, loop_model::zero_repeat, word_mask};
constexpr std::array budgets{0U, 1U, 2U, 4U, 8U};

struct Seed {
  Word mode = 0, count = 0;
  bool loaded = false;
};

State seed(const Seed input) {
  State initial;
  initial.reg = register_model::seed(input.mode);
  initial.reg.sr = input.mode | status_byte;
  initial.reg.pc = body_pc;
  initial.reg.lc = input.count == loop_model::zero_repeat ? 0U : input.count;
  initial.repeat = {.remaining = input.count,
                    .pc = body_pc,
                    .lc = saved_lc,
                    .opcode = parallel_none,
                    .loaded = input.loaded};
  initial.completed = (input.count & 1U) != 0;
  return initial;
}

bool finish_body(State &state, const a64::Instruction input) {
  state.repeat.opcode = input.opcode;
  state.repeat.loaded = true;
  --state.repeat.remaining;
  if (state.repeat.remaining != 0) {
    state.reg.lc = state.repeat.remaining;
    state.reg.pc = input.pc;
    loop_model::retire(state, input.cycles);
    return true;
  }
  state.reg.lc = state.repeat.lc;
  state.completed = true;
  state.repeat.loaded = false;
  loop_model::finish(state, {.pc = input.pc, .words = input.words});
  if (state.fault.kind != FaultKind::none) { return false; }
  loop_model::retire(state, input.cycles);
  return true;
}

Word model(State &state, const a64::Instruction input, const Word budget) {
  state.boundary = detail::NativeBoundary::none;
  Word retired = 0;
  while (retired < budget && state.repeat.remaining != 0) {
    state.reg.pc = (input.pc + 1) % address_words;
    state.inhibited = false;
    if (input.opcode != 0) {
      const auto code = input.opcode & status_byte;
      const bool invalid = code == 4 || code == 8 || code == 12;
      if (!loop_model::numeric(state, {.pc = input.pc})) { break; }
      if (invalid) {
        if ((state.reg.sr & saturation_model::mode) != 0) {
          loop_model::fail(state, {.pc = input.pc}, FaultKind::unsupported_mode, state.reg.sr);
        } else {
          loop_model::fail(state,
                           {.pc = input.pc},
                           code == 8 ? FaultKind::invalid_instruction
                                     : FaultKind::unsupported_instruction,
                           code);
        }
        break;
      }
      a64_alu_model::apply(state.reg, input.opcode & status_byte);
    }
    if (!finish_body(state, input)) { break; }
    ++retired;
    if (state.boundary != detail::NativeBoundary::none) { break; }
  }
  return retired;
}

void probe(const a64::Program &program, const a64::Instruction input, const State &initial) {
  for (const auto budget : budgets) {
    auto expected = initial;
    const auto retired = model(expected, input, budget);
    loop_model::Runner runner(initial);
    require(runner.run(program, budget) == retired,
            "REP retirement follows successful complete bodies only");
    runner.expect(expected);
  }
}

void matrix(const Word mode) {
  for (Word code = 0; code <= status_byte; ++code) {
    const a64::Instruction input{.opcode = parallel_none | code, .pc = body_pc, .mode = mode};
    const a64::Program program(input, a64::Emission::repeat);
    for (const auto count : counts) {
      for (const bool loaded : {false, true}) {
        probe(program, input, seed({.mode = mode, .count = count, .loaded = loaded}));
      }
    }
  }
}

void endpoints() {
  for (const Word mode : {0U, loop_model::cm, saturation_model::mode}) {
    for (const Word forever : {0U, loop_model::fv}) {
      for (const Word count : {1U, 2U, 3U}) {
        for (const Word stack : {0U, 1U, 2U}) {
          auto initial = seed({.mode = mode | loop_model::lf | forever, .count = count});
          initial.repeat.lc = 1;
          initial.reg.la = body_pc;
          initial.reg.sc = stack;
          initial.reg.sp = register_model::pointer_mask;
          const a64::Instruction input{
              .pc = body_pc, .mode = mode | loop_model::lf | forever, .loop_address = body_pc};
          const a64::Program program(input, a64::Emission::repeat);
          probe(program, input, initial);
        }
      }
    }
  }
}

void resume(const Word mode) {
  constexpr Word mac = 0x2000d2;
  const a64::Instruction input{.opcode = mac, .pc = body_pc, .mode = mode};
  const a64::Program program(input, a64::Emission::repeat);
  auto expected = seed({.mode = mode, .count = 5});
  loop_model::Runner runner(expected);
  for (const Word budget : {0U, 1U, 2U, 1U, 4U}) {
    const auto retired = model(expected, input, budget);
    require(runner.run(program, budget) == retired,
            "REP resumes the same retained translation across changing budgets");
    runner.expect(expected);
  }
}

void invalid_bodies() {
  constexpr Word immediate = 0x0140c0;
  constexpr Word call = 0x0d0000;
  const std::array inputs{
      a64::Instruction{.opcode = immediate, .extension = 1, .pc = body_pc, .words = 2, .cycles = 2},
      a64::Instruction{.opcode = call, .pc = body_pc, .cycles = 3},
      a64::Instruction{.opcode = loop_model::enddo, .pc = body_pc},
      a64::Instruction{.opcode = 6, .pc = body_pc, .cycles = 9}};
  for (const auto input : inputs) {
    const a64::Program program(input, a64::Emission::repeat);
    for (const Word count : {0U, 1U, 2U}) {
      for (const auto budget : budgets) {
        const auto initial = seed({.count = count});
        auto expected = initial;
        if (budget != 0 && count != 0) {
          loop_model::fail(expected, {.pc = body_pc}, FaultKind::invalid_instruction, input.opcode);
        }
        loop_model::Runner runner(initial);
        require(runner.run(program, budget) == 0,
                "Invalid REP bodies fail before PC/interrupt/operand effects");
        runner.expect(expected);
      }
    }
  }
}

void ram_fault_prefix() {
  constexpr Word load_add = 0x44d840;
  constexpr Word location = 384;
  const a64::Instruction input{.opcode = load_add, .pc = body_pc};
  const a64::Program program(input, a64::Emission::repeat);
  std::array<Word, memory_words> x{};
  x[location] = 1;
  x[location + 1] = 3;
  for (const auto budget : budgets) {
    auto initial = seed({.count = 3});
    initial.reg.r[0] = location;
    auto expected = initial;
    Word retired = 0;
    while (retired < budget && expected.repeat.remaining != 0) {
      const auto address = expected.reg.r[0]++;
      expected.reg.pc = body_pc + 1;
      expected.inhibited = false;
      if (address == location + 2) {
        expected.fault = {
            .kind = FaultKind::memory_read, .space = Space::x, .address = address, .pc = body_pc};
        expected.boundary = detail::NativeBoundary::owner;
        break;
      }
      const auto old = expected.reg;
      register_model::write(expected.reg, 4, x.at(address));
      a64_alu_model::apply(expected.reg, old, 0x40);
      require(finish_body(expected, input), "Known RAM REP prefixes retire");
      ++retired;
    }
    loop_model::Runner runner(initial);
    runner.bind({{{.data = x.data(), .words = location + 2}, {}}});
    require(runner.run(program, budget) == retired,
            "Failed repeated RAM access keeps the exact committed prefix");
    runner.expect(expected);
  }
}

void zero_count_run() {
  constexpr Word clear = 0x200013;
  const a64::Instruction input{.opcode = clear, .pc = body_pc};
  const a64::Program program(input, a64::Emission::repeat);
  auto initial = seed({.count = loop_model::zero_repeat});
  initial.reg.lc = 0;
  auto expected = initial;
  const auto retired = model(expected, input, loop_model::zero_repeat);
  loop_model::Runner runner(initial);
  require(runner.run(program, loop_model::zero_repeat) == retired &&
              retired == loop_model::zero_repeat,
          "The complete zero-count REP domain executes without truncation or budget-specific "
          "translation");
  runner.expect(expected);
}

class RepeatBus {
public:
  RepeatBus(const loop_model::Runner &runner,
            std::array<Word, memory_words> &program,
            const unsigned reject)
      : runner_(runner), program_(program), reject_(reject) {}

  void visible(const State &state, const Word address) {
    visible_ = state;
    address_ = address;
  }

  [[nodiscard]] detail::NativeBus binding() noexcept {
    return {.program = {.data = program_.data(), .words = program_.size()},
            .owner = this,
            .access = access};
  }

  void expect(const unsigned calls) const {
    require(valid_ && calls_ == calls,
            "Repeated I/O sees the exact pre-retirement register/progress/latch state once per "
            "iteration");
  }

private:
  static void access(void *const owner, detail::NativeAccess &request) noexcept {
    auto &self = *static_cast<RepeatBus *>(owner);
    ++self.calls_;
    auto physical = self.visible_.reg;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    const auto &cpu = self.runner_.cpu();
    self.valid_ =
        self.valid_ && request.action == detail::BusAction::read && request.space == Space::x &&
        request.address == self.address_ && request.pc == body_pc && request.value == 0 &&
        request.failure == Fault{} && cpu.reg == physical &&
        cpu.instructions == self.visible_.instructions && cpu.cycles == self.visible_.cycles &&
        cpu.repeat_remaining == self.visible_.repeat.remaining &&
        cpu.repeat_pc == self.visible_.repeat.pc && cpu.repeat_lc == self.visible_.repeat.lc &&
        cpu.repeat_opcode == self.visible_.repeat.opcode &&
        cpu.repeat_loaded == self.visible_.repeat.loaded &&
        cpu.repeat_completed == self.visible_.completed && !cpu.interrupt_inhibited;
    constexpr Word replacement = 0x200013;
    self.program_[body_pc] = replacement;
    request.value = 3;
    if (self.calls_ == self.reject_) {
      request.failure = {
          .kind = FaultKind::io_read, .space = Space::x, .address = request.address, .pc = body_pc};
    }
  }

  const loop_model::Runner &runner_;
  std::array<Word, memory_words> &program_;
  State visible_;
  Word address_ = 0;
  unsigned reject_, calls_ = 0;
  bool valid_ = true;
};

void io_latch() {
  constexpr Word load_add = 0x44d840;
  constexpr Word io_begin = 0xffff80;
  const a64::Instruction input{.opcode = load_add, .pc = body_pc};
  const a64::Program program(input, a64::Emission::repeat);
  for (unsigned reject = 0; reject < 4; ++reject) {
    std::array<Word, memory_words> backing{};
    backing[body_pc] = load_add;
    auto expected = seed({.count = 3});
    expected.reg.r[0] = io_begin;
    loop_model::Runner runner(expected);
    RepeatBus bus(runner, backing, reject);
    runner.bind({}, bus.binding());
    unsigned calls = 0;
    while (expected.repeat.remaining != 0) {
      const auto old = expected.reg;
      const auto address = expected.reg.r[0]++;
      expected.reg.pc = body_pc + 1;
      expected.inhibited = false;
      expected.boundary = detail::NativeBoundary::owner;
      bus.visible(expected, address);
      ++calls;
      Word retired = 0;
      if (calls == reject) {
        expected.fault = {
            .kind = FaultKind::io_read, .space = Space::x, .address = address, .pc = body_pc};
      } else {
        register_model::write(expected.reg, 4, 3);
        a64_alu_model::apply(expected.reg, old, 0x40);
        require(finish_body(expected, input),
                "Repeated I/O body has no architectural fault after an accepted read");
        retired = 1;
      }
      require(runner.run(program, 8) == retired,
              "Each repeated I/O iteration returns to its owner despite a larger budget");
      runner.expect(expected);
      bus.expect(calls);
      require(backing[body_pc] != input.opcode,
              "Backing P mutation is visible while retained REP code remains immutable");
      if (expected.fault.kind != FaultKind::none) { break; }
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        matrix(scale | saturation);
        resume(scale | saturation);
      }
    }
    endpoints();
    invalid_bodies();
    ram_fault_prefix();
    zero_count_run();
    io_latch();
    std::cout << checks << " independent native A64 REP checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
