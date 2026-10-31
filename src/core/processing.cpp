// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"

#include <stdexcept>

namespace dsp56300 {
ProcessingState Core::processing_state() const noexcept {
  return impl_->cpu.processing;
}

void Core::resume_from_stop() {
  impl_->validate_staged_mutation();
  if (impl_->executing) { throw std::logic_error("STOP wake requires stopped host execution"); }
  if (impl_->cpu.processing != ProcessingState::stopped || impl_->fault.kind != FaultKind::none) {
    throw std::invalid_argument("Only a nonfaulted STOP state admits IRQA wake completion");
  }
  impl_->cpu.processing = ProcessingState::running;
}

void Core::Impl::execute_processing(const detail::Instruction &decoded) {
  if (decoded.operation == detail::Operation::wait) {
    mark_hidden(detail::RetirementField::processing);
    cpu.processing = ProcessingState::waiting;
    return;
  }
  if (decoded.operation == detail::Operation::stop) {
    mark_hidden(detail::RetirementField::processing);
    cpu.processing = ProcessingState::stopped;
    return;
  }
  if (staged_computing()) {
    static_cast<void>(sample_instruction(
        {.kind = EffectKind::reset, .address = current_pc, .value = decoded.opcode}));
    return;
  }
  if (io == nullptr || !io->reset_peripherals(current_pc)) {
    fail(FaultKind::host_event, Space::program, current_pc, decoded.opcode);
  }
}
} // namespace dsp56300
