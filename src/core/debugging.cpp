// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300 {
using namespace detail;

DebugStatus Core::debug_status() const noexcept {
  return {.interrupt_mode = (impl_->cpu.debug_flags & debug_interrupt_mode) != 0,
          .software_occurrence = (impl_->cpu.debug_flags & debug_software_occurrence) != 0};
}

void Core::set_debug_interrupt_mode(const bool enabled) {
  impl_->validate_staged_mutation();
  if (impl_->executing) { throw std::logic_error("Debug control requires stopped host execution"); }
  const auto flags = impl_->cpu.debug_flags & ~debug_interrupt_mode;
  impl_->cpu.debug_flags = static_cast<std::uint8_t>(flags | (enabled ? debug_interrupt_mode : 0));
}

void Core::request_debug() {
  impl_->validate_staged_mutation();
  if (impl_->executing) { throw std::logic_error("Debug requests require stopped host execution"); }
  if (impl_->fault.kind != FaultKind::none || impl_->cpu.processing == ProcessingState::debugging) {
    throw std::invalid_argument("Debug request requires a nonfaulted, nondebugging CPU");
  }
  impl_->request_debug(false);
}

void Core::resume_from_debug() {
  impl_->validate_staged_mutation();
  if (impl_->executing) { throw std::logic_error("Debug resume requires stopped host execution"); }
  if (impl_->fault.kind != FaultKind::none || impl_->cpu.processing != ProcessingState::debugging) {
    throw std::invalid_argument("Only a nonfaulted debugging CPU can resume");
  }
  impl_->cpu.debug_flags &= static_cast<std::uint8_t>(~debug_software_occurrence);
  impl_->cpu.processing = ProcessingState::running;
}

void Core::Impl::request_debug(const bool software) {
  if ((cpu.debug_flags & debug_interrupt_mode) != 0) {
    if ((cpu.debug_flags & debug_interrupt_pending) == 0) {
      constexpr Word vector_bank = 0xffff00;
      constexpr Word debug_offset = 6;
      mark_hidden(RetirementField::debug_vector);
      mark_debug(debug_interrupt_pending);
      debug_vector = (cpu.reg.vba & vector_bank) | debug_offset;
      cpu.debug_flags |= debug_interrupt_pending;
    }
    mark_hidden(RetirementField::processing);
    cpu.processing = ProcessingState::running;
    return;
  }
  mark_debug(debug_software_occurrence);
  mark_hidden(RetirementField::processing);
  cpu.debug_flags = static_cast<std::uint8_t>((cpu.debug_flags & ~debug_software_occurrence) |
                                              (software ? debug_software_occurrence : 0));
  cpu.processing = ProcessingState::debugging;
}

void Core::Impl::execute_debug(const Instruction &decoded) {
  if (decoded.operation == Operation::conditional_debug &&
      !condition({.status = cpu.reg.sr, .selector = decoded.opcode & 15U})) {
    return;
  }
  request_debug(true);
}

void Core::Impl::admit_debug() {
  mark_vector();
  mark_debug(debug_interrupt_pending);
  mark_hidden(RetirementField::interrupt_inhibited);
  vector = {
      .resume = cpu.reg.pc, .address = debug_vector, .level = 3, .phase = VectorPhase::pending};
  cpu.debug_flags &= static_cast<std::uint8_t>(~debug_interrupt_pending);
  cpu.interrupt_inhibited = false;
}
} // namespace dsp56300
