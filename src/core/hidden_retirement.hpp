// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "instruction.hpp"

#include <cstdint>

namespace dsp56300::detail {
enum class RetirementField : std::uint8_t {
  repeat_remaining,
  repeat_pc,
  repeat_lc,
  repeat_opcode,
  repeat_loaded,
  repeat_completed,
  interrupt_inhibited,
  processing,
  debug_flags,
  vector_resume,
  vector_address,
  vector_level,
  vector_phase,
  vector_words,
  debug_vector,
  repeated_instruction,
  count
};

constexpr std::uint32_t retirement_bit(const RetirementField field) noexcept {
  return std::uint32_t{1} << static_cast<unsigned>(field);
}

// No register file, absolute counters, cache or transient operand/frame context.
struct RetirementValues {
  Word repeat_remaining = 0, repeat_pc = 0, repeat_lc = 0, repeat_opcode = 0;
  bool repeat_loaded = false, repeat_completed = false, interrupt_inhibited = false;
  ProcessingState processing = ProcessingState::running;
  std::uint8_t debug_flags = 0;
  VectorState vector;
  Word debug_vector = 0;
  Instruction repeated;
};

// Written fields must come from actual native writer hooks, never state differences.
struct HiddenRetirement {
  std::uint32_t fields = 0;
  std::uint8_t debug_mask = 0;
  RetirementValues value;
  Progress delta;
};

[[nodiscard]] RetirementValues retirement_values(const Cpu &cpu,
                                                 const VectorState &vector,
                                                 Word debug_vector,
                                                 const Instruction &repeated) noexcept;

// Caller owns exactly-once native retirement and any separately owned cache effects.
// This merge alone does not admit overlapping execution or establish hardware clocks.
void merge_hidden_retirement(Cpu &cpu,
                             VectorState &vector,
                             Word &debug_vector,
                             Instruction &repeated,
                             const HiddenRetirement &result);
} // namespace dsp56300::detail
