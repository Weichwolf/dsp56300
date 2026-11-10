// SPDX-License-Identifier: GPL-3.0-only
#include "semantic.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

namespace dsp56300 {
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
  synchronize_cache_mode();
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
} // namespace dsp56300
