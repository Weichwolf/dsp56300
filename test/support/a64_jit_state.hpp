// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_loop_state.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "support.hpp"
#include "vector_state.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace dsp56300::test::a64 {
class Machine {
public:
  explicit Machine(const loop_model::State &initial)
      : jit_(cpu_,
             program_,
             {.owner = this,
              .execute = forbidden,
              .moves = forbidden,
              .memory = {{{.data = x_.data(), .words = x_.size()},
                          {.data = y_.data(), .words = y_.size()}}},
              .cacheable_program = false},
             {.program = {.data = program_.data(), .words = program_.size()}, .failure = &fault_},
             {.vector = &vector_, .debug_vector = &debug_vector_}) {
    constexpr Word unsupported = 7;
    program_.fill(unsupported);
    initialize(initial);
  }

  void initialize(const loop_model::State &initial) {
    cpu_ = {};
    cpu_.reg = initial.reg;
    constexpr unsigned padding = 8;
    cpu_.reg.a <<= padding;
    cpu_.reg.b <<= padding;
    cpu_.instructions = initial.instructions;
    cpu_.cycles = initial.cycles;
    cpu_.repeat_remaining = initial.repeat.remaining;
    cpu_.repeat_pc = initial.repeat.pc;
    cpu_.repeat_lc = initial.repeat.lc;
    cpu_.repeat_opcode = initial.repeat.opcode;
    cpu_.repeat_loaded = initial.repeat.loaded;
    cpu_.repeat_completed = initial.completed;
    cpu_.interrupt_inhibited = initial.inhibited;
    cpu_.processing = initial.processing;
    cpu_.debug_flags = initial.debug_flags;
    vector_ = initial.vector;
    debug_vector_ = initial.debug_vector;
    fault_ = initial.fault;
  }

  [[nodiscard]] detail::Jit &jit() { return jit_; }

  [[nodiscard]] auto &program() { return program_; }

  [[nodiscard]] auto &x() { return x_; }

  [[nodiscard]] auto &y() { return y_; }

  void expect(const loop_model::State &expected) const {
    auto physical = expected.reg;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    const bool matched =
        cpu_.reg == physical && cpu_.instructions == expected.instructions &&
        cpu_.cycles == expected.cycles && cpu_.repeat_remaining == expected.repeat.remaining &&
        cpu_.repeat_pc == expected.repeat.pc && cpu_.repeat_lc == expected.repeat.lc &&
        cpu_.repeat_opcode == expected.repeat.opcode &&
        cpu_.repeat_loaded == expected.repeat.loaded &&
        cpu_.repeat_completed == expected.completed && cpu_.processing == expected.processing &&
        cpu_.interrupt_inhibited == expected.inhibited &&
        cpu_.debug_flags == expected.debug_flags && fault_ == expected.fault &&
        vector_.resume == expected.vector.resume && vector_.address == expected.vector.address &&
        vector_.level == expected.vector.level && vector_.phase == expected.vector.phase &&
        vector_.words == expected.vector.words && vector_.loaded == expected.vector.loaded &&
        debug_vector_ == expected.debug_vector && helpers_ == 0;
    if (!matched) {
      std::cerr << std::hex << "pc=" << cpu_.reg.pc << " expected_pc=" << expected.reg.pc
                << " sr=" << cpu_.reg.sr << " expected_sr=" << expected.reg.sr << std::dec
                << " fault=" << static_cast<unsigned>(fault_.kind)
                << " expected_fault=" << static_cast<unsigned>(expected.fault.kind)
                << " instructions=" << cpu_.instructions
                << " expected_instructions=" << expected.instructions << '\n';
    }
    require(matched, "Jit factory paths preserve complete independent state without ISA helpers");
  }

private:
  static Word forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
    ++static_cast<Machine *>(owner)->helpers_;
    return 0;
  }

  std::array<Word, memory_words * 2> program_{};
  std::array<Word, memory_words> x_{}, y_{};
  detail::Cpu cpu_;
  Fault fault_;
  detail::VectorState vector_;
  Word debug_vector_ = 0;
  unsigned helpers_ = 0;
  detail::Jit jit_;
};
}
