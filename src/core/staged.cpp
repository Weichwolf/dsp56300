// SPDX-License-Identifier: GPL-3.0-only
#include "staged.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dsp56300 {
namespace {
detail::Instruction decoded_words(const InstructionWords words, const bool complete = true) {
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

FaultKind rejected_effect(const InstructionEffect effect) {
  switch (effect.kind) {
    case EffectKind::read:
    case EffectKind::fetch_opcode:
    case EffectKind::fetch_extension: return FaultKind::memory_read;
    case EffectKind::write: return FaultKind::memory_write;
    case EffectKind::bit: return FaultKind::io_bit;
    case EffectKind::reset: return FaultKind::host_event;
    case EffectKind::address_update:
    case EffectKind::move_update:
    case EffectKind::alu_update:
    case EffectKind::unit_update:
    case EffectKind::pcu_update:
    case EffectKind::retire:
    default: throw std::logic_error("No rejection class for retirement");
  }
}
}

EffectTicket::EffectTicket(std::shared_ptr<const bool> owner, const Position position)
    : owner_(std::move(owner)), epoch_(position.epoch), position_(position.ordinal) {}

void Core::Impl::invalidate_effect_tickets() {
  if (effect_epoch == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Instruction effect generations exhausted");
  }
  ++effect_epoch;
}

void Core::Impl::validate_staged_mutation() const {
  if (staged_active()) { throw std::logic_error("Pending instruction owns CPU mutation"); }
}

InstructionFetch Core::instruction_fetch() const {
  if (impl_->executing || impl_->staged_active()) {
    throw std::logic_error("Instruction fetch metadata requires stopped unowned execution");
  }
  if (impl_->debug_ready()) {
    return {.address = impl_->debug_vector, .latched_opcode = std::nullopt};
  }
  if (impl_->exception_ready()) {
    const auto slot = impl_->vector.phase == Impl::VectorPhase::second ? 1U : 0U;
    std::optional<Word> latch;
    if (const auto words = impl_->vector.words) { latch = words->at(slot); }
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

std::optional<InstructionOutcome> Core::instruction_outcome() const noexcept {
  const auto outcome = impl_->last_outcome;
  if (impl_->executing || impl_->staged_active() || fault().kind != FaultKind::none || !outcome ||
      outcome->instructions != impl_->cpu.instructions) {
    return std::nullopt;
  }
  return outcome;
}

std::optional<PendingInstruction> Core::pending_instruction() const {
  if (!impl_->staged_active() || impl_->fault.kind != FaultKind::none) { return std::nullopt; }
  const auto &selected = impl_->staged->pending;
  if (!selected) { return std::nullopt; }
  return PendingInstruction{
      .ticket = EffectTicket(impl_->effect_owner,
                             {.epoch = impl_->effect_epoch, .ordinal = impl_->staged->used}),
      .effect = *selected,
      // A native opcode-fetch effect may still be filling an instruction cache
      // line. Its requested word is not owned until capture_program_words returns.
      .opcode = !impl_->staged->fetch_words || selected->kind != EffectKind::fetch_opcode
                    ? std::optional(impl_->staged->input.opcode)
                    : std::nullopt,
      .computed = impl_->staged->computed};
}

InstructionProgress Core::begin_instruction(const InstructionWords words,
                                            const InstructionPublication publication) {
  return begin_owned(words, publication);
}

InstructionProgress Core::begin_fetch(const InstructionPublication publication) {
  return begin_owned(std::nullopt, publication);
}

InstructionProgress Core::begin_owned(const std::optional<InstructionWords> captured,
                                      const InstructionPublication publication) {
  impl_->validate_staged_mutation();
  if (impl_->executing || impl_->fault.kind != FaultKind::none ||
      impl_->cpu.processing != ProcessingState::running) {
    throw std::logic_error("Instruction computation requires healthy stopped running state");
  }
  const auto fetch = instruction_fetch();
  const auto words = captured.value_or(InstructionWords{.address = fetch.address});
  const auto decoded = captured ? decoded_words(words) : detail::Instruction{};
  if (publication != InstructionPublication::retirement &&
      publication != InstructionPublication::address_updates &&
      publication != InstructionPublication::move_updates &&
      publication != InstructionPublication::alu_updates &&
      publication != InstructionPublication::unit_updates &&
      publication != InstructionPublication::pcu_updates) {
    throw std::invalid_argument("Unknown instruction publication mode");
  }
  if (captured && (words.address != fetch.address ||
                   (fetch.latched_opcode && *fetch.latched_opcode != words.opcode))) {
    throw std::invalid_argument("Captured instruction differs from the actual CPU fetch/latch");
  }
  if (!impl_->effect_owner) { impl_->effect_owner = std::make_shared<const bool>(false); }
  if (!impl_->staged) { impl_->staged = std::make_unique<detail::StagedState>(); }
  const auto base = impl_->execution_state();
  impl_->invalidate_effect_tickets();
  auto &work = *impl_->staged;
  // Only [0, used) owns accepted effects. Computation replaces all scratch
  // before publication; keep inactive record storage out of the reset path.
  work.used = 0;
  work.planning_disabled = false;
  work.base = base;
  work.input = words;
  work.decoded = decoded;
  work.fetch_words = !captured;
  work.latched_opcode = fetch.latched_opcode;
  work.publication = publication;
  work.active = true;
  const detail::ExecutionGuard guard(impl_->executing);
  impl_->compute_instruction();
  return {.pending = pending_instruction(), .progress = progress(), .fault = fault()};
}

Progress Core::retirement_delta(const EffectTicket &ticket) const {
  if (impl_->executing || !impl_->staged_active() || !impl_->staged->complete ||
      impl_->fault.kind != FaultKind::none || ticket.owner_ != impl_->effect_owner ||
      ticket.epoch_ != impl_->effect_epoch || ticket.position_ != impl_->staged->used) {
    throw std::invalid_argument("Retirement delta requires the current PCU retirement ticket");
  }
  const auto &work = *impl_->staged;
  if (work.publication != InstructionPublication::pcu_updates || !work.pending ||
      work.pending->kind != EffectKind::retire) {
    throw std::invalid_argument("Retirement delta requires the current PCU retirement ticket");
  }
  return work.hidden_result.delta;
}

InstructionProgress Core::advance_instruction(const EffectTicket &ticket) {
  if (impl_->executing || !impl_->staged_active() || !impl_->staged->pending ||
      impl_->fault.kind != FaultKind::none || ticket.owner_ != impl_->effect_owner ||
      ticket.epoch_ != impl_->effect_epoch || ticket.position_ != impl_->staged->used) {
    throw std::invalid_argument("Stale, foreign or unavailable instruction effect ticket");
  }
  const detail::ExecutionGuard guard(impl_->executing);
  auto &work = *impl_->staged;
  const auto selected = work.pending;
  if (!selected) { throw std::logic_error("Instruction lost its selected effect"); }
  const auto effect = *selected;
  if (effect.kind == EffectKind::retire) {
    if (!work.complete) { throw std::logic_error("Retirement lost its native computation"); }
    impl_->retire_owned();
    impl_->last_outcome = InstructionOutcome{.instructions = impl_->cpu.instructions,
                                             .address = work.input.address,
                                             .opcode = work.input.opcode,
                                             .control_transfer = work.control_transfer};
    work.active = false;
    work.pending.reset();
    return {.progress = progress(), .fault = fault(), .retired = true};
  }
  if (work.used == work.records.size()) {
    throw std::overflow_error("Instruction effect capacity exhausted");
  }
  Word response = 0;
  if (effect.kind == EffectKind::address_update) {
    if (!effect.address_update) { throw std::logic_error("Native address writeback is missing"); }
  } else if (effect.kind == EffectKind::move_update) {
    if (!effect.move_update) { throw std::logic_error("Native MOVE writeback is missing"); }
  } else if (effect.kind == EffectKind::alu_update) {
    if (!effect.alu_update) { throw std::logic_error("Native ALU writeback is missing"); }
  } else if (effect.kind == EffectKind::unit_update) {
    if (!effect.unit_update) { throw std::logic_error("Native scalar-unit writeback is missing"); }
  } else if (effect.kind == EffectKind::pcu_update) {
    if (!effect.pcu_update) { throw std::logic_error("Native PCU writeback is missing"); }
  } else if (!impl_->perform_instruction(effect, response)) {
    impl_->current_pc = effect.pc;
    impl_->fail(rejected_effect(effect),
                effect.space,
                effect.address,
                effect.kind == EffectKind::bit ? effect.bit : effect.value);
    work.pending.reset();
    return {.progress = progress(), .fault = fault()};
  }
  work.records.at(work.used++) = {.effect = effect, .response = response};
  impl_->resume_instruction(effect);
  return {.pending = pending_instruction(), .progress = progress(), .fault = fault()};
}

bool Core::interruptible(const InstructionWords words) const {
  return interruptible_captured(words, true);
}

bool Core::interruptible_opcode(const InstructionWords words) const {
  if (words.extension) {
    throw std::invalid_argument("Opcode eligibility query has unexpected extension");
  }
  return interruptible_captured(words, false);
}

bool Core::interruptible_captured(const InstructionWords words, const bool complete) const {
  const auto decoded = decoded_words(words, complete);
  if (impl_->staged_active()) { return false; }
  if (words.address != impl_->cpu.reg.pc) {
    throw std::invalid_argument("Interrupt metadata differs from the current normal PC");
  }
  if (!impl_->cpu.repeat_completed) { return interruptible(); }
  if (detail::is_repeat(decoded.operation)) { return false; }
  // Only the old RAM-peek tail depends on the completed-REP marker.
  return (impl_->cpu.processing == ProcessingState::running ||
          impl_->cpu.processing == ProcessingState::waiting) &&
         (impl_->cpu.debug_flags & detail::debug_interrupt_pending) == 0 &&
         fault().kind == FaultKind::none && impl_->vector.phase == Impl::VectorPhase::none &&
         impl_->cpu.repeat_remaining == 0 && !impl_->cpu.interrupt_inhibited;
}

void Core::request_interrupt(const Word vector,
                             const unsigned level,
                             const InstructionWords foreground) {
  impl_->validate_staged_mutation();
  if (impl_->executing) { throw std::logic_error("Interrupt requests require stopped execution"); }
  constexpr unsigned maximum_level = 3;
  constexpr unsigned level_shift = 8;
  const auto mask = (impl_->cpu.reg.sr >> level_shift) & maximum_level;
  if (vector > word_mask || (vector & 1U) != 0 || level > maximum_level || level < mask ||
      !interruptible(foreground)) {
    throw std::invalid_argument("Captured interrupt request is not admissible");
  }
  impl_->cpu.processing = ProcessingState::running;
  impl_->vector = {.resume = impl_->cpu.reg.pc,
                   .address = vector,
                   .level = level,
                   .phase = Impl::VectorPhase::first};
  impl_->cpu.repeat_completed = false;
}
} // namespace dsp56300
