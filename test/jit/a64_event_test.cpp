// SPDX-License-Identifier: GPL-3.0-only
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"
#include "vector_state.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using State = loop_model::State;
using Phase = detail::VectorPhase;
constexpr Word debug = 0x200, debugcc = 0x300, illegal = 5, trap = 6, trapcc = 16;
constexpr Word interrupt_mode = 2, software = 4, pending = 8;
constexpr Word saved_debug_vector = 0x123456, saved_resume = 0x345678, saved_vector = 0xabcdee;
constexpr std::array<Word, 2> saved_words{0x654321, 0xfedcba};
constexpr Word unqualified_modes = 0x226000, saved_count = 0x456, previous_opcode = 0x200013;
constexpr std::array phases{Phase::none, Phase::pending, Phase::first, Phase::second};

State seed(const Word flags) {
  State state;
  state.reg = register_model::seed(flags);
  state.reg.sr = flags;
  constexpr std::uint64_t spread = 0x010101;
  state.reg.vba = static_cast<Word>((flags * spread) % address_words);
  state.debug_vector = saved_debug_vector;
  state.vector = {.resume = saved_resume,
                  .address = saved_vector,
                  .level = 1,
                  .phase = Phase::none,
                  .words = saved_words,
                  .loaded = (flags & 1U) != 0};
  return state;
}

bool requested(const Word opcode, const Word status) {
  return opcode == debug || opcode == illegal || opcode == trap ||
         selection_model::test_condition({.status = status, .selector = opcode % 16});
}

void request(State &state, const Word opcode, const Word pc) {
  if (!requested(opcode, state.reg.sr)) { return; }
  const bool debugging = opcode >= debug;
  state.boundary = detail::NativeBoundary::owner;
  if (!debugging) {
    if (state.vector.phase == Phase::none) {
      state.vector = {.resume =
                          static_cast<Word>((pc + (opcode == illegal ? 2U : 1U)) % address_words),
                      .address = ((state.reg.vba / 256) * 256) + (opcode == illegal ? 4U : 8U),
                      .level = 3,
                      .phase = Phase::pending};
    }
    return;
  }
  if ((state.debug_flags & interrupt_mode) == 0) {
    state.debug_flags = static_cast<std::uint8_t>(state.debug_flags | software);
    state.processing = ExecutionState::debugging;
    return;
  }
  if ((state.debug_flags & pending) == 0) {
    state.debug_vector = (state.reg.vba / 256) * 256 + 6;
    state.debug_flags = static_cast<std::uint8_t>(state.debug_flags | pending);
  }
  state.processing = ExecutionState::running;
}

unsigned cycles(const Word opcode) {
  if (opcode >= debug) { return opcode == debug ? 1U : 5U; }
  return opcode == illegal ? 5U : 9U;
}

Word model(State &state, const a64::Instruction input, const Word budget, const bool repeat) {
  state.boundary = detail::NativeBoundary::none;
  Word retired = 0;
  while (retired < budget && (!repeat || state.repeat.remaining != 0)) {
    state.reg.pc = (input.pc + 1) % address_words;
    state.inhibited = false;
    request(state, input.opcode, input.pc);
    if (repeat) {
      state.repeat.opcode = input.opcode;
      state.repeat.loaded = true;
      --state.repeat.remaining;
      if (state.repeat.remaining != 0) {
        state.reg.lc = state.repeat.remaining;
        state.reg.pc = input.pc;
      } else {
        state.reg.lc = state.repeat.lc;
        state.repeat.loaded = false;
        state.completed = true;
        loop_model::finish(state, {.pc = input.pc});
      }
    } else {
      state.completed = false;
      loop_model::finish(state, {.pc = input.pc});
    }
    if (state.fault.kind != FaultKind::none) { break; }
    loop_model::retire(state, input.cycles);
    ++retired;
    if (!repeat || state.boundary != detail::NativeBoundary::none) { break; }
  }
  return retired;
}

void probe(const a64::Program &program, const a64::Instruction input, const State &initial) {
  for (const Word budget : {0U, 1U, 8U}) {
    auto expected = initial;
    const auto retired = model(expected, input, budget, false);
    loop_model::Runner runner(initial);
    require(runner.run(program, budget) == retired,
            "Debug/exception requests retire once without ISA helpers");
    runner.expect(expected);
  }
}

void conditions() {
  for (const Word base : {debugcc, trapcc}) {
    for (Word condition = 0; condition < 16; ++condition) {
      const a64::Instruction input{.opcode = base + condition, .cycles = base == debugcc ? 5U : 9U};
      const a64::Program program(input);
      for (Word flags = 0; flags <= status_byte; ++flags) {
        auto initial = seed(flags);
        initial.debug_flags = static_cast<std::uint8_t>(flags);
        for (const auto phase : phases) {
          initial.vector.phase = phase;
          probe(program, input, initial);
        }
      }
    }
  }
}

void debug_flags() {
  const a64::Instruction input{.opcode = debug};
  const a64::Program program(input);
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (const auto processing : {ExecutionState::running,
                                  ExecutionState::waiting,
                                  ExecutionState::stopped,
                                  ExecutionState::debugging}) {
      auto initial = seed(flags);
      initial.processing = processing;
      initial.debug_flags = static_cast<std::uint8_t>(flags);
      initial.vector.phase = phases.at(flags % phases.size());
      probe(program, input, initial);
    }
  }
}

void debug_banks() {
  const a64::Instruction input{.opcode = debug};
  const a64::Program program(input);
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    for (const Word bank : {Word{1} << bit, word_mask ^ (Word{1} << bit)}) {
      for (const Word flags : {interrupt_mode, interrupt_mode | pending}) {
        auto initial = seed(bank);
        initial.reg.vba = bank;
        initial.debug_flags = static_cast<std::uint8_t>(flags);
        probe(program, input, initial);
      }
    }
  }
}

void exception_phases() {
  for (const Word opcode : {illegal, trap}) {
    for (const Word pc : {start, word_mask - 1, word_mask}) {
      const a64::Instruction input{.opcode = opcode, .pc = pc, .cycles = cycles(opcode)};
      const a64::Program program(input);
      for (Word bit = 0; bit < word_bits; ++bit) {
        for (const auto phase : phases) {
          auto initial = seed((Word{1} << bit) | unqualified_modes);
          initial.reg.pc = pc;
          initial.reg.vba = (Word{1} << bit) | status_byte;
          initial.vector.phase = phase;
          probe(program, input, initial);
        }
      }
    }
  }
}

void loop_endpoints() {
  for (const Word opcode : {debug, illegal, trap}) {
    for (const Word mode : {0U, loop_model::cm, Word{1} << 20}) {
      for (const Word count : {1U, 2U}) {
        for (const Word stack : {0U, 1U, 2U}) {
          auto initial = seed(mode | loop_model::lf);
          initial.reg.la = start;
          initial.reg.lc = count;
          initial.reg.sc = stack;
          const a64::Instruction input{
              .opcode = opcode, .mode = initial.reg.sr, .cycles = cycles(opcode)};
          const a64::Program program(input);
          probe(program, input, initial);
        }
      }
    }
  }
}

Word model_sequence(State &state,
                    const std::array<a64::Instruction, 3> &operations,
                    const Word budget) {
  Word retired = 0;
  for (const auto operation : operations) {
    if (retired == budget) { break; }
    if (operation.opcode == 0) {
      ++state.reg.pc;
      state.inhibited = state.completed = false;
      loop_model::retire(state, 1);
      ++retired;
    } else {
      retired += model(state, operation, 1, false);
    }
    if (state.boundary != detail::NativeBoundary::none) { break; }
  }
  return retired;
}

void sequences() {
  for (const Word opcode : {debug, debugcc, debugcc + 10, illegal, trap, trapcc}) {
    const a64::Instruction input{.opcode = opcode, .pc = start + 1, .cycles = cycles(opcode)};
    const std::array operations{a64::Instruction{}, input, a64::Instruction{.pc = start + 2}};
    const a64::Program program(operations);
    for (Word flags = 0; flags <= status_byte; ++flags) {
      for (const Word budget : {0U, 1U, 2U, 3U, 8U}) {
        auto expected = seed(flags);
        loop_model::Runner runner(expected);
        const auto retired = model_sequence(expected, operations, budget);
        require(runner.run(program, budget) == retired,
                "Taken events stop complete native sequences before the next instruction");
        runner.expect(expected);
      }
    }
  }
}

void invalid_repeat() {
  for (const Word opcode : {trap, trapcc, trapcc + 10}) {
    const a64::Instruction input{.opcode = opcode, .cycles = cycles(opcode)};
    const a64::Program program(input, a64::Emission::repeat);
    for (const Word count : {0U, 1U, 2U}) {
      for (const Word budget : {0U, 1U, 8U}) {
        const auto initial = [&] {
          auto state = seed(status_byte);
          state.repeat = {.remaining = count, .pc = start, .lc = saved_count};
          return state;
        }();
        auto expected = initial;
        if (count != 0 && budget != 0) {
          loop_model::fail(expected, {.pc = start}, FaultKind::invalid_instruction, opcode);
        }
        loop_model::Runner runner(initial);
        require(runner.run(program, budget) == 0,
                "TRAP and TRAPcc stay prohibited under REP before any event or CPU effect");
        runner.expect(expected);
      }
    }
  }
}

void repeats() {
  for (const Word opcode : {debug, debugcc, debugcc + 10, illegal}) {
    const a64::Instruction input{.opcode = opcode, .cycles = cycles(opcode)};
    const a64::Program program(input, a64::Emission::repeat);
    for (const Word flags : {0U, 2U, 4U, 8U, 10U, status_byte}) {
      auto expected = seed(status_byte);
      expected.debug_flags = static_cast<std::uint8_t>(flags);
      expected.repeat = {.remaining = 3, .pc = start, .lc = saved_count, .opcode = previous_opcode};
      loop_model::Runner runner(expected);
      for (const Word budget : {0U, 1U, 8U, 2U, 1U}) {
        const auto retired = model(expected, input, budget, true);
        require(runner.run(program, budget) == retired,
                "Retained repeated event instructions preserve one pending vector and latch");
        runner.expect(expected);
      }
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    conditions();
    debug_flags();
    debug_banks();
    exception_phases();
    loop_endpoints();
    sequences();
    repeats();
    invalid_repeat();
    std::cout << checks << " independent native A64 debug/exception checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
