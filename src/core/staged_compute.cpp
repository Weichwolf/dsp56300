// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"
#include "staged.hpp"

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dsp56300::detail {
Word StagedState::sample(const InstructionEffect effect) {
  if (!active || !computing) { throw std::logic_error("No active instruction computation"); }
  if (collecting_pcu) {
    if (effect.kind != EffectKind::pcu_update || pending || used != 0) {
      throw std::logic_error("Pure control instruction produced an unexpected effect");
    }
    pending = effect;
    return 0; // CPU publication has no external response operand.
  }
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
namespace {
bool collect_pcu(const detail::StagedState &work) {
  const auto &base = work.base;
  if (work.fetch_words || work.used != 0 ||
      work.publication != InstructionPublication::pcu_updates || base.cache ||
      (base.cpu.reg.sr & (detail::cache_enable | detail::loop_flag)) != 0 ||
      base.cpu.repeat_remaining != 0 || base.vector.phase != detail::VectorPhase::none ||
      (base.cpu.debug_flags & detail::debug_interrupt_pending) != 0) {
    return false;
  }
  const auto &decoded = work.decoded;
  if (decoded.operation == detail::Operation::nop ||
      decoded.operation == detail::Operation::jump_short) {
    return true;
  }
  constexpr unsigned absolute_mode = 6;
  return decoded.operation == detail::Operation::jump_effective &&
         ((decoded.opcode >> 11) & 7U) == absolute_mode && ((decoded.opcode >> 8) & 7U) == 0;
}
}

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
  work.deferred_retirement.reset();
  work.complete = false;
  work.computing = true;
  work.collecting_pcu = collect_pcu(work);
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
      if (work.collecting_pcu && work.pending) {
        work.deferred_retirement = retirement;
      } else {
        work.pending = retirement;
      }
    }
  } catch (const detail::InstructionPause &) {
    // Retain the native prefix, never a complete result, at the owned bus boundary.
    work.candidate = execution_state();
  } catch (...) {
    work.computing = false;
    work.collecting_pcu = false;
    restore_execution(work.base);
    restore_published_results();
    work.pending.reset();
    fail(FaultKind::host_event, Space::program, work.input.address, work.input.opcode);
    throw;
  }
  work.computing = false;
  work.collecting_pcu = false;
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

void Core::Impl::resume_instruction(const InstructionEffect &effect) {
  auto &work = *staged;
  if (!work.deferred_retirement) {
    compute_instruction();
    return;
  }
  if (effect.kind != EffectKind::pcu_update || !effect.pcu_update) {
    throw std::logic_error("Deferred control retirement lost its PCU writer");
  }
  apply_pcu(*effect.pcu_update);
  work.pending = std::exchange(work.deferred_retirement, std::nullopt);
}
} // namespace dsp56300
