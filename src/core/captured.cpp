// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "translation.hpp"

#include <stdexcept>

namespace dsp56300 {
RunResult Core::run_instruction(const InstructionWords words) {
  impl_->validate_staged_mutation();
  if (impl_->executing || impl_->fault.kind != FaultKind::none ||
      impl_->cpu.processing != ProcessingState::running) {
    throw std::logic_error("Captured execution requires healthy stopped running state");
  }
  const auto decoded = Impl::decode_words(words);
  const auto fetch = instruction_fetch();
  if (fetch.exception_vector || impl_->debug_ready() || impl_->instruction_cache_active() ||
      words.address != fetch.address ||
      (fetch.latched_opcode && *fetch.latched_opcode != words.opcode)) {
    throw std::invalid_argument("Captured execution differs from ordinary CPU fetch ownership");
  }
  const detail::ExecutionGuard guard(impl_->executing);
  impl_->last_outcome.reset();
  impl_->synchronize_program_view();
  impl_->yield_requested = false;
  RunResult result;
  const bool repeating = impl_->cpu.repeat_remaining != 0;
  if (impl_->jit) {
    result.dispatches =
        impl_->jit->run_captured({.instruction = decoded,
                                  .pc = words.address,
                                  .mode = impl_->cpu.reg.sr & detail::numeric_mode_mask,
                                  .loop_address = detail::active_loop_address(impl_->cpu.reg)});
    if (result.dispatches != 0 && repeating) { impl_->repeated = decoded; }
  }
  if (result.dispatches == 0 && impl_->fault.kind == FaultKind::none) {
    impl_->step_decoded(decoded);
    impl_->synchronize_program_view();
    if (impl_->fault.kind == FaultKind::none) {
      result.dispatches = 1;
      if (impl_->jit) { impl_->jit->helper_retired(); }
    }
  }
  result.progress = progress();
  result.fault = fault();
  result.state = processing_state();
  result.yielded = impl_->yield_requested;
  return result;
}
} // namespace dsp56300
