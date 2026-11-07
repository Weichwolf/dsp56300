// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_native.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "support.hpp"
#include "vector_state.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>

namespace dsp56300::test::loop_model {
struct Repeat {
  Word remaining = 0, pc = 0, lc = 0, opcode = 0;
  bool loaded = false;
};

struct State {
  Registers reg{};
  Repeat repeat;
  std::uint64_t instructions = std::numeric_limits<std::uint64_t>::max(),
                cycles = std::numeric_limits<std::uint64_t>::max();
  Fault fault;
  detail::VectorState vector;
  Word debug_vector = 0;
  ExecutionState processing = ExecutionState::running;
  std::uint8_t debug_flags = 0;
  detail::NativeBoundary boundary = detail::NativeBoundary::none;
  bool completed = true, inhibited = true, program_written = false;
};

inline void increment(std::uint64_t &value, const std::uint64_t amount) {
  const auto room = std::numeric_limits<std::uint64_t>::max() - value;
  value = amount <= room ? value + amount : amount - room - 1;
}

inline void retire(State &state, const unsigned cycles) {
  increment(state.instructions, 1);
  increment(state.cycles, cycles);
}

class Runner {
public:
  explicit Runner(const State &initial)
      : context_{
            .runtime = {.owner = &helpers_, .execute = forbidden, .moves = forbidden, .memory = {}},
            .events = {.vector = &vector_, .debug_vector = &debug_vector_}} {
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
  }

  void bind(const std::array<detail::JitMemory, 2> memory, const detail::NativeBus bus = {}) {
    context_.runtime.memory = memory;
    context_.bus = bus;
  }

  void peripherals(Io *const io) { context_.events.io = io; }

  void cache(void *const owner, detail::CacheState *(*storage)(void *) noexcept) {
    context_.events.cache_owner = owner;
    context_.events.cache = storage;
  }

  [[nodiscard]] const detail::Cpu &cpu() const { return cpu_; }

  Word run(const a64::Program &program, const Word budget) {
    require(context_.failure.kind == FaultKind::none, "A failed native run cannot be resumed");
    context_.budget = budget;
    context_.boundary = detail::NativeBoundary::none;
    return detail::enter_block(program.entry(), cpu_, context_);
  }

  Word chain(const a64::Program &program,
             const Word budget,
             const detail::NativeIndex &index,
             const detail::NativeCatalog &catalog) {
    require(context_.failure.kind == FaultKind::none, "A failed native chain cannot be resumed");
    context_.boundary = detail::NativeBoundary::none;
    return detail::enter_blocks(program.entry(), cpu_, context_, budget, index, &catalog);
  }

  void expect(const State &expected) const {
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
        cpu_.repeat_completed == expected.completed &&
        cpu_.interrupt_inhibited == expected.inhibited && cpu_.processing == expected.processing &&
        cpu_.debug_flags == expected.debug_flags && vector_.resume == expected.vector.resume &&
        vector_.address == expected.vector.address && vector_.level == expected.vector.level &&
        vector_.phase == expected.vector.phase && vector_.words == expected.vector.words &&
        vector_.loaded == expected.vector.loaded && debug_vector_ == expected.debug_vector &&
        context_.boundary == expected.boundary && context_.failure == expected.fault &&
        context_.program_written == expected.program_written && helpers_ == 0;
    if (!matched) {
      std::cerr << std::hex << "pc=" << cpu_.reg.pc << " expected_pc=" << expected.reg.pc
                << " sr=" << cpu_.reg.sr << " expected_sr=" << expected.reg.sr
                << " la=" << cpu_.reg.la << " expected_la=" << expected.reg.la
                << " lc=" << cpu_.reg.lc << " expected_lc=" << expected.reg.lc
                << " sp=" << cpu_.reg.sp << " expected_sp=" << expected.reg.sp
                << " sc=" << cpu_.reg.sc << " expected_sc=" << expected.reg.sc
                << " fault_value=" << context_.failure.value
                << " expected_fault_value=" << expected.fault.value << std::dec
                << " fault=" << static_cast<unsigned>(context_.failure.kind)
                << " expected_fault=" << static_cast<unsigned>(expected.fault.kind)
                << " remaining=" << cpu_.repeat_remaining
                << " expected_remaining=" << expected.repeat.remaining
                << " instructions=" << cpu_.instructions
                << " expected_instructions=" << expected.instructions << '\n';
    }
    require(matched,
            "Complete native loop/REP state, progress, faults and ownership match independent "
            "expectations");
  }

private:
  static Word forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
    ++*static_cast<unsigned *>(owner);
    return 0;
  }

  detail::Cpu cpu_;
  detail::NativeContext context_;
  unsigned helpers_ = 0;
  detail::VectorState vector_;
  Word debug_vector_ = 0;
};
}
