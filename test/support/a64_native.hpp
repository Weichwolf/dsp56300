// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "support.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <span>
#include <vector>

namespace dsp56300::test::a64 {
struct Instruction {
  Word opcode = 0, extension = 0, pc = start, mode = 0;
  unsigned words = 1, cycles = 1;
};

class Program {
public:
  explicit Program(const Instruction input) : Program(std::span<const Instruction>(&input, 1)) {}

  explicit Program(const std::span<const Instruction> inputs) {
    std::vector<detail::BlockInstruction> operations;
    operations.reserve(inputs.size());
    for (const auto input : inputs) {
      auto decoded = detail::decode(input.opcode);
      decoded.extension = input.extension;
      require(decoded.words == input.words && decoded.cycles == input.cycles,
              "Authored instruction length/base cost differs from its independent expectation");
      operations.push_back({.instruction = decoded, .pc = input.pc, .mode = input.mode});
    }
    detail::a64::Emitter emitter;
    entry_ = arena_.append(emitter.generate(operations));
  }

  [[nodiscard]] const void *entry() const noexcept { return entry_; }

private:
  detail::ExecutableArena arena_;
  const void *entry_ = nullptr;
};

struct Expected {
  Registers reg{};
  Word retired = 1;
  std::uint64_t cycles = 1;
  Fault failure{};
  ExecutionState processing = ExecutionState::running;
  detail::NativeBoundary boundary = detail::NativeBoundary::none;
  bool inhibited = false, repeat_completed = false;
};

class Invocation {
public:
  explicit Invocation(const Registers &initial)
      : context_{.runtime = {
                     .owner = &helpers_, .execute = forbidden, .moves = forbidden, .memory = {}}} {
    cpu_.reg = initial;
    cpu_.reg.a <<= detail::padding_bits;
    cpu_.reg.b <<= detail::padding_bits;
    cpu_.instructions = cpu_.cycles = std::numeric_limits<std::uint64_t>::max();
    cpu_.repeat_completed = cpu_.interrupt_inhibited = true;
  }

  Invocation(const Invocation &) = delete;
  Invocation &operator=(const Invocation &) = delete;

  void run(const Program &program, const Word budget = 1) {
    context_.budget = budget;
    retired_ = detail::enter_block(program.entry(), cpu_, context_);
  }

  void expect(const Expected &expected, const Word opcode) const {
    auto physical = expected.reg;
    physical.a <<= detail::padding_bits;
    physical.b <<= detail::padding_bits;
    const bool matched =
        cpu_.reg == physical && retired_ == expected.retired && helpers_ == 0 &&
        cpu_.instructions == wrapped(expected.retired) && cpu_.cycles == wrapped(expected.cycles) &&
        cpu_.processing == expected.processing && cpu_.interrupt_inhibited == expected.inhibited &&
        cpu_.repeat_completed == expected.repeat_completed &&
        context_.failure == expected.failure && context_.boundary == expected.boundary &&
        cpu_.repeat_remaining == 0 && !cpu_.repeat_loaded && cpu_.debug_flags == 0;
    if (!matched) {
      auto actual = cpu_.reg;
      actual.a >>= detail::padding_bits;
      actual.b >>= detail::padding_bits;
      std::cerr << std::hex << "opcode=" << opcode << " pc=" << actual.pc
                << " expected_pc=" << expected.reg.pc << " sr=" << actual.sr
                << " expected_sr=" << expected.reg.sr << " sp=" << actual.sp
                << " expected_sp=" << expected.reg.sp << " sc=" << actual.sc
                << " expected_sc=" << expected.reg.sc << " fault_value=" << context_.failure.value
                << " expected_fault_value=" << expected.failure.value << std::dec
                << " retired=" << retired_ << " expected_retired=" << expected.retired
                << " cycles=" << cpu_.cycles << " expected_cycles=" << wrapped(expected.cycles)
                << " kind=" << static_cast<unsigned>(context_.failure.kind)
                << " expected_kind=" << static_cast<unsigned>(expected.failure.kind) << '\n';
    }
    require(matched, "A64 native state/progress/fault differs from independent expectations");
  }

private:
  detail::Cpu cpu_;
  detail::NativeContext context_;

  static std::uint32_t forbidden(void *const owner,
                                 const detail::BlockInstruction * /*operation*/) noexcept {
    ++*static_cast<unsigned *>(owner);
    return 0;
  }

  static std::uint64_t wrapped(const std::uint64_t count) noexcept {
    return count == 0 ? std::numeric_limits<std::uint64_t>::max() : count - 1;
  }

  unsigned helpers_ = 0;
  Word retired_ = 0;
};
}
