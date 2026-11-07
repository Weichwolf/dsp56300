// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "translation.hpp"

#include <optional>
#include <stdexcept>

namespace dsp56300 {
detail::Instruction Core::Impl::decode_words(const InstructionWords words, const bool complete) {
  if (words.address > word_mask || words.opcode > word_mask ||
      (words.extension && *words.extension > word_mask)) {
    throw std::invalid_argument("Captured instruction word exceeds 24 bits");
  }
  auto decoded = detail::decode(words.opcode);
  if ((complete && words.extension.has_value() != (decoded.words == 2)) ||
      (words.extension && decoded.words != 2)) {
    throw std::invalid_argument("Captured instruction has missing or unexpected extension");
  }
  decoded.extension = words.extension.value_or(0);
  return decoded;
}

InstructionFetch Core::instruction_fetch() const {
  if (impl_->executing) {
    throw std::logic_error("Instruction fetch metadata requires stopped unowned execution");
  }
  if (impl_->debug_ready()) {
    return {.address = impl_->debug_vector, .latched_opcode = std::nullopt};
  }
  if (impl_->exception_ready()) {
    const auto slot = impl_->vector.phase == Impl::VectorPhase::second ? 1U : 0U;
    std::optional<Word> latch;
    if (impl_->vector.loaded) { latch = impl_->vector.words.at(slot); }
    return {.address = (impl_->vector.address +
                        (impl_->vector.phase == Impl::VectorPhase::second ? 1U : 0U)) &
                       word_mask,
            .latched_opcode = latch,
            .exception_vector = true};
  }
  return {.address = impl_->cpu.reg.pc,
          .latched_opcode = impl_->cpu.repeat_remaining != 0 && impl_->cpu.repeat_loaded
                                ? std::optional(impl_->repeated.opcode)
                                : std::nullopt};
}

RunResult Core::run_instruction(const InstructionWords words) {

  if (impl_->executing || impl_->fault.kind != FaultKind::none ||
      impl_->cpu.processing != ExecutionState::running) {
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
  impl_->synchronize_program_view();
  impl_->yield_requested = false;
  RunResult result;
  const bool repeating = impl_->cpu.repeat_remaining != 0;
  if (impl_->jit) {
    result.retired =
        impl_->jit->run_captured({.instruction = decoded,
                                  .pc = words.address,
                                  .mode = impl_->cpu.reg.sr & detail::numeric_mode_mask,
                                  .loop_address = detail::active_loop_address(impl_->cpu.reg)});
    if (result.retired != 0 && repeating) { impl_->repeated = decoded; }
  }
  if (result.retired == 0 && impl_->fault.kind == FaultKind::none) {
    impl_->step_decoded(decoded);
    impl_->synchronize_program_view();
    if (impl_->fault.kind == FaultKind::none) { result.retired = 1; }
  }
  result.progress = progress();
  result.fault = fault();
  result.state = processing_state();
  result.yielded = impl_->yield_requested;
  return result;
}

bool Core::interrupt_ready(const InstructionWords words) const {
  return interrupt_ready_captured(words, true);
}

bool Core::interrupt_ready_opcode(const InstructionWords words) const {
  if (words.extension) {
    throw std::invalid_argument("Opcode eligibility query has unexpected extension");
  }
  return interrupt_ready_captured(words, false);
}

bool Core::interrupt_ready_captured(const InstructionWords words, const bool complete) const {
  const auto decoded = Impl::decode_words(words, complete);
  if (words.address != impl_->cpu.reg.pc) {
    throw std::invalid_argument("Interrupt metadata differs from the current normal PC");
  }
  if (!impl_->cpu.repeat_completed) { return interrupt_ready(); }
  if (detail::is_repeat(decoded.operation)) { return false; }
  // Only the old RAM-peek tail depends on the completed-REP marker.
  return (impl_->cpu.processing == ExecutionState::running ||
          impl_->cpu.processing == ExecutionState::waiting) &&
         (impl_->cpu.debug_flags & detail::debug_interrupt_pending) == 0 &&
         fault().kind == FaultKind::none && impl_->vector.phase == Impl::VectorPhase::none &&
         impl_->cpu.repeat_remaining == 0 && !impl_->cpu.interrupt_inhibited;
}

void Core::request_interrupt(const Word vector, const unsigned level) {
  if (impl_->executing) { throw std::logic_error("Interrupt requests require stopped execution"); }
  constexpr unsigned maximum_level = 3;
  constexpr unsigned level_shift = 8;
  const auto mask = (impl_->cpu.reg.sr >> level_shift) & maximum_level;
  if (vector > word_mask || (vector & 1U) != 0 || level > maximum_level || level < mask ||
      !interrupt_ready()) {
    throw std::invalid_argument("Deferred interrupt request is not admissible");
  }
  impl_->cpu.processing = ExecutionState::running;
  impl_->vector = {.resume = impl_->cpu.reg.pc,
                   .address = vector,
                   .level = level,
                   .phase = Impl::VectorPhase::first};
  impl_->cpu.repeat_completed = false;
}

void Core::request_interrupt(const Word vector,
                             const unsigned level,
                             const InstructionWords foreground) {
  if (impl_->executing) { throw std::logic_error("Interrupt requests require stopped execution"); }
  constexpr unsigned maximum_level = 3;
  constexpr unsigned level_shift = 8;
  const auto mask = (impl_->cpu.reg.sr >> level_shift) & maximum_level;
  if (vector > word_mask || (vector & 1U) != 0 || level > maximum_level || level < mask ||
      !interrupt_ready(foreground)) {
    throw std::invalid_argument("Captured interrupt request is not admissible");
  }
  impl_->cpu.processing = ExecutionState::running;
  impl_->vector = {.resume = impl_->cpu.reg.pc,
                   .address = vector,
                   .level = level,
                   .phase = Impl::VectorPhase::first};
  impl_->cpu.repeat_completed = false;
}
} // namespace dsp56300
