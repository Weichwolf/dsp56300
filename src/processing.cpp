// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "io.hpp"

#include <stdexcept>

namespace dsp56300 {
ExecutionState Core::processing_state() const noexcept {
  return impl_->cpu.processing;
}

void Core::resume_from_stop() {

  if (impl_->executing) { throw std::logic_error("STOP wake requires stopped host execution"); }
  if (impl_->cpu.processing != ExecutionState::stopped || impl_->fault.kind != FaultKind::none) {
    throw std::invalid_argument("Only a nonfaulted STOP state admits IRQA wake completion");
  }
  impl_->cpu.processing = ExecutionState::running;
}

void Core::Impl::execute_processing(const detail::Instruction &decoded) {
  if (decoded.operation == detail::Operation::wait) {

    cpu.processing = ExecutionState::waiting;
    return;
  }
  if (decoded.operation == detail::Operation::stop) {

    cpu.processing = ExecutionState::stopped;
    return;
  }

  if (io == nullptr || !dsp56300_io_reset(io, current_pc)) {
    fail(FaultKind::host_event, Space::program, current_pc, decoded.opcode);
  }
  image_io_completed();
}
} // namespace dsp56300
