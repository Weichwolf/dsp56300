// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction_boundary.hpp"
#include "jit.hpp"
#include "translation.hpp"

#include <cstdint>

namespace dsp56300 {
std::uint32_t Core::Impl::jit_execute(void *const owner,
                                      const detail::NativeOp *const operation) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  if (self.bounded_execution && detail::requires_owner(operation->instruction)) {
    self.bounded_memory_blocked = true;
    return 0;
  }
  self.step_decoded(operation->instruction);
  self.synchronize_program_view();
  if (self.fault.kind != FaultKind::none) { return 0; }
  self.jit->helper_retired();
  const auto next = (operation->pc + operation->instruction.words) & word_mask;
  return self.cpu.processing == ProcessingState::running &&
                 (self.cpu.debug_flags & detail::debug_interrupt_pending) == 0 &&
                 self.vector.phase == VectorPhase::none && !self.instruction_cache_active() &&
                 self.cpu.reg.pc == next && self.cpu.repeat_remaining == 0 &&
                 !self.jit->exit_requested() &&
                 (self.cpu.reg.sr & detail::numeric_mode_mask) == operation->mode &&
                 detail::active_loop_address(self.cpu.reg) == operation->loop_address
             ? 2U
             : 1U;
}

std::uint32_t Core::Impl::jit_moves(void *const owner,
                                    const detail::NativeOp *const operation) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  if (self.bounded_execution && detail::requires_owner(operation->instruction)) {
    self.bounded_memory_blocked = true;
    return 0;
  }
  self.current_pc = operation->pc;
  self.current_words = operation->instruction.words;
  self.cpu.interrupt_inhibited = false;
  self.cpu.reg.pc = (operation->pc + operation->instruction.words) & word_mask;
  self.latched = {self.cpu.reg.a, self.cpu.reg.b, self.cpu.reg.x, self.cpu.reg.y};
  self.moves(operation->instruction);
  self.synchronize_program_view();
  return self.fault.kind == FaultKind::none ? 1U : 0U;
}
} // namespace dsp56300
