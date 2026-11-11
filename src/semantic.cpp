// SPDX-License-Identifier: GPL-3.0-only
#include "semantic.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
extern "C" const std::array<Word, semantic_abi_fields> dsp56300_semantic_abi{
    3,
    word_mask,
    sizeof(Instruction),
    offsetof(Instruction, opcode),
    offsetof(Instruction, extension),
    offsetof(Instruction, operation),
    offsetof(Instruction, words),
    offsetof(Instruction, cycles),
    static_cast<Word>(Operation::conditional_debug) + 1};
}

namespace dsp56300 {
// Native owners and every continuing native edge have already admitted this state.
void Core::Impl::assume_image_admission() const noexcept {
  const bool admitted = image != nullptr && active_image != nullptr && image_ready &&
                        fault.kind == FaultKind::none &&
                        cpu.processing == ExecutionState::running && !yield_requested &&
                        !debug_ready() && !instruction_cache_active();
  __builtin_assume(admitted);
}

// Portable ISA, REP and DO effects are available to semantic IR translation
// without entering an architecture-specific native execution owner.
void Core::Impl::step_semantic(const detail::Instruction &decoded) {
  current_pc = cpu.reg.pc;
  const bool repeating = cpu.repeat_remaining != 0;
  if (repeating && (decoded.words != 1 || !detail::repeatable(decoded.operation))) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return;
  }
  current_words = decoded.words;
  cpu.interrupt_inhibited = false;
  cpu.reg.pc = (current_pc + decoded.words) & word_mask;
  execute_operation(decoded);
  if (fault.kind != FaultKind::none) { return; }
  if (repeating) {
    cpu.repeat_opcode = decoded.opcode;
    repeated = decoded;
    cpu.repeat_loaded = true;
    --cpu.repeat_remaining;
    if (cpu.repeat_remaining != 0) {
      cpu.reg.lc = cpu.repeat_remaining;
      cpu.reg.pc = current_pc;
      retire_instruction(decoded.cycles);
      return;
    }
    cpu.reg.lc = cpu.repeat_lc;
    cpu.repeat_completed = true;
    cpu.repeat_loaded = false;
  } else if (!detail::is_repeat(decoded.operation)) {
    cpu.repeat_completed = false;
  }
  finish_loop();
  if (fault.kind == FaultKind::none) { retire_instruction(decoded.cycles); }
}

detail::SemanticResult Core::Impl::semantic_execute(void *const owner,
                                                    const detail::Instruction &decoded) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  const auto pc = self.cpu.reg.pc;
  const auto previous_status = self.cpu.reg.sr;
  self.step_semantic(decoded);
  // Synchronous callbacks cannot replace the executing owner's installed image.
  const bool installed = self.image != nullptr;
  __builtin_assume(installed);
  self.check_interrupt_window(previous_status);
  if (self.fault.kind != FaultKind::none) {
    return {.pc = self.cpu.reg.pc, .flow = detail::SemanticFlow::fault};
  }
  if (self.cpu.processing != ExecutionState::running || self.debug_ready() ||
      self.exception_ready() || self.instruction_cache_active() || self.yield_requested) {
    return {.pc = self.cpu.reg.pc, .flow = detail::SemanticFlow::owner};
  }
  const auto next = (pc + decoded.words) & word_mask;
  return {.pc = self.cpu.reg.pc,
          .flow = self.cpu.reg.pc == next ? detail::SemanticFlow::sequential
                                          : detail::SemanticFlow::branch};
}

std::uint64_t Core::Impl::semantic_entry(void *const owner,
                                         const Word pc,
                                         const detail::Instruction &decoded) noexcept {
  // A prepared instruction site owns this exact fetch address. Carry that
  // constant into its effects instead of hiding it behind an unrelated load.
  auto &self = *static_cast<Impl *>(owner);
  self.assume_image_admission();
  const bool ordinary = !self.exception_ready();
  __builtin_assume(ordinary);
  self.cpu.reg.pc = pc;
  constexpr unsigned flow_shift = 32;
  const auto result = semantic_execute(owner, decoded);
  return result.pc | (static_cast<std::uint64_t>(result.flow) << flow_shift);
}

std::uint64_t Core::Impl::semantic_vector_entry(void *const owner,
                                                const detail::Instruction &decoded) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  self.assume_image_admission();
  const bool inserted = self.exception_ready();
  __builtin_assume(inserted);
  if (self.vector.phase == VectorPhase::pending) {
    self.cpu.reg.pc = self.vector.resume;
    self.cpu.repeat_completed = false;
    self.vector.phase = VectorPhase::first;
  }
  self.current_pc =
      (self.vector.address + (self.vector.phase == VectorPhase::second ? 1U : 0U)) & word_mask;
  self.current_words = decoded.words;
  const auto previous_status = self.cpu.reg.sr;
  self.step_vector_semantic(decoded);
  const bool installed = self.image != nullptr;
  __builtin_assume(installed);
  self.check_interrupt_window(previous_status);
  auto flow = detail::SemanticFlow::branch;
  if (self.fault.kind != FaultKind::none) {
    flow = detail::SemanticFlow::fault;
  } else if (self.vector.phase == VectorPhase::none ||
             self.cpu.processing != ExecutionState::running || self.debug_ready() ||
             self.instruction_cache_active() || self.yield_requested) {
    flow = detail::SemanticFlow::owner;
  }
  const auto next =
      self.vector.phase == VectorPhase::none
          ? self.cpu.reg.pc
          : (self.vector.address + (self.vector.phase == VectorPhase::second ? 1U : 0U)) &
                word_mask;
  constexpr unsigned flow_shift = 32;
  return next | (static_cast<std::uint64_t>(flow) << flow_shift);
}
} // namespace dsp56300
