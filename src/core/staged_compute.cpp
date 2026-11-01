// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"
#include "staged.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace dsp56300::detail {
Word StagedState::sample(const InstructionEffect effect) {
  if (!active || !computing) { throw std::logic_error("No active instruction computation"); }
  if (cursor < used) {
    const auto &saved = records.at(cursor++);
    if (saved.effect != effect) { throw std::logic_error("Accepted instruction effect changed"); }
    return saved.response;
  }
  if (used == capacity) { throw std::overflow_error("Instruction effect capacity exhausted"); }
  pending = effect;
  throw InstructionPause{};
}
} // namespace dsp56300::detail

namespace dsp56300 {
void Core::Impl::publish_address_update(const AddressWriteback update, const Word address) const {
  if (!staged_computing() || staged->publication == InstructionPublication::retirement ||
      fault.kind != FaultKind::none) {
    return;
  }
  static_cast<void>(sample_instruction(
      {.kind = EffectKind::address_update, .address = address, .address_update = update}));
}

void Core::Impl::restore_published_results() {
  for (std::size_t index = 0; index < staged->used; ++index) {
    const auto &effect = staged->records[index].effect;
    if (effect.address_update) {
      const auto update = *effect.address_update;
      cpu.reg.r.at(update.index) = update.value;
    }
    if (effect.move_update) { apply_move(*effect.move_update); }
    if (effect.alu_update) { apply_alu(*effect.alu_update); }
    if (effect.unit_update) { apply_unit(*effect.unit_update); }
    if (effect.pcu_update) { apply_pcu(*effect.pcu_update); }
  }
}

Word Core::Impl::sample_instruction(InstructionEffect effect) const {
  effect.pc = current_pc;
  effect.omr = staged->base.cpu.reg.omr;
  return staged->sample(effect);
}

detail::Instruction Core::Impl::captured_instruction(const Word address) const {
  if (!staged_computing() || staged->input.address != address) {
    throw std::logic_error("Native execution requested a different captured instruction");
  }
  return staged->decoded;
}

void Core::Impl::compute_instruction() {
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
  work.complete = false;
  work.computing = true;
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
      work.pending = InstructionEffect{.kind = EffectKind::retire,
                                       .address = work.input.address,
                                       .value = cpu.reg.pc,
                                       .pc = work.input.address,
                                       .omr = work.base.cpu.reg.omr};
    }
  } catch (const detail::InstructionPause &) {
    // Retain the native prefix, never a complete result, at the owned bus boundary.
    work.candidate = execution_state();
  } catch (...) {
    work.computing = false;
    restore_execution(work.base);
    restore_published_results();
    work.pending.reset();
    fail(FaultKind::host_event, Space::program, work.input.address, work.input.opcode);
    throw;
  }
  work.computing = false;
  work.computed = cpu.reg;
  work.computed.a >>= detail::padding_bits;
  work.computed.b >>= detail::padding_bits;
  const auto failure = fault;
  // Cache mutations belong to the completed native prefix, not CPU retirement.
  // Only one staged instruction may own the core at a time.
  auto cache = std::move(icache);
  restore_execution(work.base);
  icache = std::move(cache);
  restore_published_results();
  if (failure.kind != FaultKind::none) {
    fault = failure;
    work.pending.reset();
  }
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
