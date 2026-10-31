// SPDX-License-Identifier: GPL-3.0-only
#include "hidden_retirement.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "instruction.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail {
namespace {
bool written(const HiddenRetirement &result, const RetirementField field) noexcept {
  return (result.fields & retirement_bit(field)) != 0;
}

void merge_repeat(Cpu &cpu, const HiddenRetirement &result) noexcept {
  const auto &value = result.value;
  if (written(result, RetirementField::repeat_remaining)) {
    cpu.repeat_remaining = value.repeat_remaining;
  }
  if (written(result, RetirementField::repeat_pc)) { cpu.repeat_pc = value.repeat_pc; }
  if (written(result, RetirementField::repeat_lc)) { cpu.repeat_lc = value.repeat_lc; }
  if (written(result, RetirementField::repeat_opcode)) { cpu.repeat_opcode = value.repeat_opcode; }
  if (written(result, RetirementField::repeat_loaded)) { cpu.repeat_loaded = value.repeat_loaded; }
  if (written(result, RetirementField::repeat_completed)) {
    cpu.repeat_completed = value.repeat_completed;
  }
}

void merge_cpu_control(Cpu &cpu, const HiddenRetirement &result) noexcept {
  const auto &value = result.value;
  if (written(result, RetirementField::interrupt_inhibited)) {
    cpu.interrupt_inhibited = value.interrupt_inhibited;
  }
  if (written(result, RetirementField::processing)) { cpu.processing = value.processing; }
  if (written(result, RetirementField::debug_flags)) {
    cpu.debug_flags = static_cast<std::uint8_t>((cpu.debug_flags & ~result.debug_mask) |
                                                (value.debug_flags & result.debug_mask));
  }
}

void merge_vector(VectorState &vector, const HiddenRetirement &result) noexcept {
  const auto &value = result.value.vector;
  if (written(result, RetirementField::vector_resume)) { vector.resume = value.resume; }
  if (written(result, RetirementField::vector_address)) { vector.address = value.address; }
  if (written(result, RetirementField::vector_level)) { vector.level = value.level; }
  if (written(result, RetirementField::vector_phase)) { vector.phase = value.phase; }
}
}

RetirementValues retirement_values(const Cpu &cpu,
                                   const VectorState &vector,
                                   const Word debug_vector,
                                   const Instruction &repeated) noexcept {
  return {.repeat_remaining = cpu.repeat_remaining,
          .repeat_pc = cpu.repeat_pc,
          .repeat_lc = cpu.repeat_lc,
          .repeat_opcode = cpu.repeat_opcode,
          .repeat_loaded = cpu.repeat_loaded,
          .repeat_completed = cpu.repeat_completed,
          .interrupt_inhibited = cpu.interrupt_inhibited,
          .processing = cpu.processing,
          .debug_flags = cpu.debug_flags,
          .vector = vector,
          .debug_vector = debug_vector,
          .repeated = repeated};
}

void merge_hidden_retirement(Cpu &cpu,
                             VectorState &vector,
                             Word &debug_vector,
                             Instruction &repeated,
                             const HiddenRetirement &result) {
  constexpr auto known = retirement_bit(RetirementField::count) - 1;
  if ((result.fields & ~known) != 0) {
    throw std::logic_error("Unknown native hidden retirement writer");
  }
  merge_repeat(cpu, result);
  merge_cpu_control(cpu, result);
  merge_vector(vector, result);
  if (written(result, RetirementField::debug_vector)) { debug_vector = result.value.debug_vector; }
  if (written(result, RetirementField::repeated_instruction)) { repeated = result.value.repeated; }
  cpu.instructions = wrapped_add(cpu.instructions, result.delta.instructions);
  cpu.cycles = wrapped_add(cpu.cycles, result.delta.estimated_cycles);
}
} // namespace dsp56300::detail
