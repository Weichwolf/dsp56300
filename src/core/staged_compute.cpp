// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"
#include "staged.hpp"

#include <cstddef>
#include <exception>
#include <stdexcept>

namespace dsp56300 {
void Core::Impl::publish_address_update(const AddressWriteback update, const Word address) const {
  if (!staged_computing() || staged->publication == InstructionPublication::retirement ||
      fault.kind != FaultKind::none) {
    return;
  }
  static_cast<void>(sample_instruction(
      {.kind = EffectKind::address_update, .address = address, .address_update = update}));
}

void Core::Impl::apply_publication(const InstructionEffect &effect) {
  if (effect.address_update) {
    const auto update = *effect.address_update;
    cpu.reg.r.at(update.index) = update.value;
  }
  if (effect.move_update) { apply_move(*effect.move_update); }
  if (effect.alu_update) { apply_alu(*effect.alu_update); }
  if (effect.unit_update) { apply_unit(*effect.unit_update); }
  if (effect.pcu_update) { apply_pcu(*effect.pcu_update); }
}

void Core::Impl::restore_published_results() {
  for (std::size_t index = 0; index < staged->used; ++index) {
    apply_publication(staged->records[index].effect);
  }
}

Word Core::Impl::sample_instruction(InstructionEffect effect) const {
  effect.pc = current_pc;
  effect.omr = staged->base.cpu.reg.omr;
  return staged->sample(effect, cpu.reg);
}

detail::Instruction Core::Impl::captured_instruction(const Word address) const {
  if (!staged_computing() || staged->input.address != address) {
    throw std::logic_error("Native execution requested a different captured instruction");
  }
  return staged->decoded;
}

void Core::Impl::compute_instruction() {
  auto error = compute_attempt();
  if (staged->planned != 0 && (error || fault.kind != FaultKind::none)) {
    // Nothing in the planned CPU suffix has been published or sent to Io.
    // Replay without planning to retain the original first boundary and fault order.
    staged->planning_disabled = true;
    error = compute_attempt();
  }
  finish_computation(error);
}

std::exception_ptr Core::Impl::compute_attempt() {
  auto &work = *staged;
  restore_execution(work.base);
  work.cursor = 0;
  work.alu_result = {};
  work.unit_result = {};
  work.pcu_result = {};
  work.hidden_result = {};
  work.pcu_source = false;
  work.control_transfer = false;
  work.unit_active = false;
  work.pending.reset();
  work.following.reset();
  work.planned = work.plan_index = 0;
  work.complete = false;
  work.computing = true;
  work.planning = plan_instruction();
  std::exception_ptr error;
  try {
    if (work.fetch_words) { capture_program_words(); }
    if (fault.kind == FaultKind::none) { step(); }
    if (fault.kind == FaultKind::none && work.cursor != work.used) {
      throw std::logic_error("Instruction did not consume every accepted effect");
    }
    if (fault.kind == FaultKind::none) {
      work.candidate = execution_state();
      if (pcu_publication()) {
        work.hidden_result.value = detail::retirement_values(cpu, vector, debug_vector, repeated);
      }
      work.complete = true;
      const InstructionEffect retirement{.kind = EffectKind::retire,
                                         .address = work.input.address,
                                         .value = cpu.reg.pc,
                                         .pc = work.input.address,
                                         .omr = work.base.cpu.reg.omr};
      work.pending = retirement;
    }
  } catch (const detail::InstructionPause &) {
    // Retain the native prefix, never a complete result, at the owned bus boundary.
    work.candidate = execution_state();
  } catch (...) {
    error = std::current_exception();
    work.pending.reset();
  }
  work.computing = false;
  work.planning = false;
  return error;
}

bool Core::Impl::perform_instruction(const InstructionEffect effect, Word &response) const {
  if (io == nullptr) { return false; }
  switch (effect.kind) {
    case EffectKind::read:
    case EffectKind::fetch_opcode:
    case EffectKind::fetch_extension: {
      const bool accepted = io->read(effect.space, effect.address, effect.pc, response);
      response &= word_mask;
      return accepted;
    }
    case EffectKind::write: return io->write(effect.space, effect.address, effect.pc, effect.value);
    case EffectKind::bit: {
      bool previous = false;
      const bool accepted =
          io->bit(effect.space, effect.address, effect.pc, effect.bit, effect.action, previous);
      response = previous ? 1U : 0U;
      return accepted;
    }
    case EffectKind::reset: return io->reset_peripherals(effect.pc);
    case EffectKind::address_update:
    case EffectKind::move_update:
    case EffectKind::alu_update:
    case EffectKind::unit_update:
    case EffectKind::pcu_update:
    case EffectKind::retire:
    default: throw std::logic_error("CPU publication is not an external bus operation");
  }
}

} // namespace dsp56300
