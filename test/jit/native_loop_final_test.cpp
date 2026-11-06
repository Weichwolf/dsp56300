// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
constexpr Word loop_flag = Word{1} << 15, forever_flag = Word{1} << 16;
constexpr Word loop_bits = loop_flag | forever_flag, pointer_mask = 63;
constexpr Word previous_la = 0x654321, previous_lc = 0xfedcba;
constexpr Word operand = 0xffffff, frame_pc = 0x123456;
constexpr Word multiply_pair = 5U << 4;
constexpr auto nop = isa::opcode<isa::Opcode::nop>()[0];
constexpr auto mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | multiply_pair;
constexpr auto add = isa::opcode<isa::Opcode::add_immediate_long>()[0];
constexpr auto register_add = isa::opcode<isa::Opcode::add_x0_a>()[0];
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array saved_bits{0U, loop_flag, forever_flag, loop_bits};
unsigned scenarios = 0;

struct Input {
  Word pointer = 0, count = 2, flags = 0, saved = 0;
  bool forever = false;
};

struct Expected {
  Registers state;
  Fault fault;
};

class Machine {
public:
  explicit Machine(const Word opcode)
      : opcode_(opcode), core_(Memory::separate({.program = program_, .x = x_, .y = y_})) {
    program_[start] = opcode;
    program_[start + 1] = operand;
    expected_program_ = program_;
  }

  void sample(const Word mode, const Input input) {
    if (core_.state().failure.kind != FaultKind::none) { core_.reset(); }
    auto initial = register_model::seed(mode);
    initial.sr = mode | input.flags | loop_flag | (input.forever ? forever_flag : 0);
    initial.la = start + (opcode_ == add ? 1U : 0U);
    initial.lc = 1;
    initial.sp = input.pointer;
    initial.sc = input.count;
    initial.stack[input.pointer % stack_entries] =
        (std::uint64_t{frame_pc} << word_bits) | input.saved | (input.flags ^ status_byte);
    initial.stack[(input.pointer + pointer_mask) % stack_entries] =
        (std::uint64_t{previous_la} << word_bits) | previous_lc;
    const auto expected = model(initial, input.forever);
    core_.set_registers(initial);
    const auto before = core_.state().progress;
    require(core_.run(0).retired == 0 && core_.registers() == initial &&
                core_.state().progress == before,
            "A zero grant leaves a final DO instruction and both frames untouched");
    const auto result = core_.run(1);
    const auto completed = expected.fault.kind == FaultKind::none ? 1U : 0U;
    const Progress progress{.instructions = before.instructions + completed,
                            .estimated_cycles =
                                before.estimated_cycles +
                                (std::uint64_t{completed} * (opcode_ == add ? 2U : 1U))};
    const bool matched =
        result.retired == completed && result.progress == progress &&
        result.fault == expected.fault && result.state == ExecutionState::running &&
        !result.yielded && core_.registers() == expected.state &&
        x_ == std::array<Word, memory_words>{} && y_ == x_ && program_ == expected_program_;
    if (!matched) {
      const auto actual = core_.registers();
      std::cerr << "case=" << scenarios << " opcode=" << opcode_ << " mode=" << mode
                << " flags=" << input.flags << " SP=" << input.pointer << " SC=" << input.count
                << " saved=" << input.saved << " forever=" << input.forever
                << " actual PC/SR/LA/LC/SP/SC=" << actual.pc << '/' << actual.sr << '/' << actual.la
                << '/' << actual.lc << '/' << actual.sp << '/' << actual.sc
                << " expected=" << expected.state.pc << '/' << expected.state.sr << '/'
                << expected.state.la << '/' << expected.state.lc << '/' << expected.state.sp << '/'
                << expected.state.sc << " fault=" << static_cast<unsigned>(result.fault.kind) << '/'
                << result.fault.pc << '/' << result.fault.address << '/' << result.fault.value
                << " expected_fault=" << static_cast<unsigned>(expected.fault.kind) << '/'
                << expected.fault.pc << '/' << expected.fault.address << '/' << expected.fault.value
                << " progress=" << result.progress.instructions << '/'
                << result.progress.estimated_cycles
                << " expected_progress=" << progress.instructions << '/'
                << progress.estimated_cycles << " state=" << static_cast<unsigned>(result.state)
                << " registers_equal=" << (actual == expected.state)
                << " retired=" << result.retired << '\n';
    }
    require(matched,
            "Final DO preserves independently modeled arithmetic, frames, faults and progress");
    if (completed == 0) {
      const auto again = core_.run(1);
      require(again.retired == 0 && again.fault == expected.fault && again.progress == progress &&
                  core_.registers() == expected.state,
              "A partial-pop failure is latched without replaying its instruction or frame");
    }
    ++scenarios;
  }

private:
  [[nodiscard]] Expected model(const Registers &initial, const bool forever) const {
    Expected result{.state = initial, .fault = {}};
    auto &state = result.state;
    if (opcode_ != nop) {
      auto source = initial;
      if (opcode_ == add) { source.x = operand; }
      sequence_model::alu(state, source, opcode_ == add ? register_add : opcode_);
    }
    state.pc = (initial.la + 1) & word_mask;
    if (forever) {
      state.pc = frame_pc;
      state.lc = 0;
      return result;
    }
    const auto pop = [&]() -> std::uint64_t {
      if (state.sc == 0) {
        result.fault = {.kind = FaultKind::stack, .address = start, .value = state.sp, .pc = start};
        return 0;
      }
      const auto frame = state.stack[state.sp % stack_entries];
      state.sp = (state.sp + pointer_mask) & pointer_mask;
      --state.sc;
      return frame;
    };
    const auto saved = pop();
    const auto previous = pop();
    state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
    state.la = static_cast<Word>(previous >> word_bits);
    state.lc = static_cast<Word>(previous) & word_mask;
    return result;
  }

  Word opcode_;
  std::array<Word, memory_words> program_{}, expected_program_{}, x_{}, y_{};
  Core core_;
};

void flags_matrix(Machine &machine) {
  constexpr std::array pointers{0U, 15U, 16U, pointer_mask};
  constexpr std::array counts{2U, 15U, 31U};
  for (const auto mode : modes) {
    for (const auto pointer : pointers) {
      for (const auto count : counts) {
        for (const auto saved : saved_bits) {
          for (Word flags = 0; flags <= status_byte; ++flags) {
            machine.sample(mode,
                           {.pointer = pointer, .count = count, .flags = flags, .saved = saved});
          }
        }
      }
    }
  }
}

void stack_matrix(Machine &machine) {
  constexpr std::array flags{0U, status_byte};
  for (const auto mode : {0U, mode_rm | mode_up}) {
    for (Word pointer = 0; pointer <= pointer_mask; ++pointer) {
      for (const auto saved : saved_bits) {
        for (const auto status : flags) {
          for (const auto count : {0U, 1U, 2U, 31U}) {
            machine.sample(mode,
                           {.pointer = pointer, .count = count, .flags = status, .saved = saved});
          }
          machine.sample(
              mode,
              {.pointer = pointer, .count = 0, .flags = status, .saved = saved, .forever = true});
        }
      }
    }
  }
}
}

int main() {
  try {
    for (const auto opcode : {nop, mac, add}) {
      Machine machine(opcode);
      flags_matrix(machine);
      stack_matrix(machine);
    }
    std::cout << scenarios << " independent final DO cases and " << checks
              << " frame/state/fault boundaries passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
