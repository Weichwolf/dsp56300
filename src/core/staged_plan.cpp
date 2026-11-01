// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "instruction.hpp"
#include "staged.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dsp56300::detail {
namespace {
bool cpu_publication(const EffectKind kind) {
  return kind == EffectKind::address_update || kind == EffectKind::move_update ||
         kind == EffectKind::alu_update || kind == EffectKind::unit_update ||
         kind == EffectKind::pcu_update;
}
}

Word StagedState::sample(const InstructionEffect effect, const Registers &registers) {
  if (!active || !computing) { throw std::logic_error("No active instruction computation"); }
  if (cursor < used) {
    const auto &saved = records.at(cursor++);
    if (saved.effect != effect) { throw std::logic_error("Accepted instruction effect changed"); }
    return saved.response;
  }
  if (used == capacity) { throw std::overflow_error("Instruction effect capacity exhausted"); }
  if (planning && cpu_publication(effect.kind) && planned < plan_capacity &&
      used + planned < capacity && (registers.sr & (cache_enable | loop_flag)) == 0) {
    auto &next = plan.at(planned++);
    next.effect = effect;
    next.computed = registers;
    next.computed.a >>= padding_bits;
    next.computed.b >>= padding_bits;
    return 0; // CPU writers have no external response operand.
  }
  pending = effect;
  throw InstructionPause{};
}

bool StagedState::capacity_replay() const noexcept {
  return plan_index == planned && used == capacity &&
         (!following || following->kind != EffectKind::retire);
}
} // namespace dsp56300::detail

namespace dsp56300 {
bool Core::Impl::plan_instruction() const {
  const auto &work = *staged;
  const auto &base = work.base;
  if (work.planning_disabled || work.fetch_words ||
      work.publication != InstructionPublication::pcu_updates || base.cache ||
      (base.cpu.reg.sr & (detail::cache_enable | detail::loop_flag)) != 0 ||
      base.cpu.repeat_remaining != 0 || base.vector.phase != detail::VectorPhase::none ||
      (base.cpu.debug_flags & detail::debug_interrupt_pending) != 0) {
    return false;
  }
  using enum detail::Operation;
  static constexpr std::array fallback{repeat_immediate,
                                       loop_immediate,
                                       loop_register,
                                       loop_short,
                                       loop_effective,
                                       loop_forever,
                                       break_loop,
                                       repeat_register,
                                       repeat_short,
                                       repeat_effective,
                                       end_loop,
                                       illegal,
                                       trap,
                                       conditional_trap,
                                       cache_flush,
                                       cache_flush_unlocked,
                                       cache_free,
                                       cache_lock,
                                       cache_unlock,
                                       cache_lock_relative,
                                       cache_unlock_relative,
                                       reset_peripherals,
                                       wait,
                                       stop,
                                       debug,
                                       conditional_debug};
  return std::ranges::find(fallback, work.decoded.operation) == fallback.end();
}

void Core::Impl::finish_computation(const std::exception_ptr &error) {
  auto &work = *staged;
  work.computed = cpu.reg;
  work.computed.a >>= detail::padding_bits;
  work.computed.b >>= detail::padding_bits;
  auto failure = fault;
  if (error) {
    restore_execution(work.base);
    restore_published_results();
    fail(FaultKind::host_event, Space::program, work.input.address, work.input.opcode);
    failure = fault;
  } else {
    // Cache mutations keep their existing native-prefix ownership.
    auto cache = std::move(icache);
    restore_execution(work.base);
    icache = std::move(cache);
    restore_published_results();
    fault = failure;
  }
  if (failure.kind != FaultKind::none) { work.pending.reset(); }
  if (work.planned == 0) {
    if (error) { std::rethrow_exception(error); }
    return;
  }
  work.following = work.pending;
  work.following_computed = work.computed;
  work.pending = work.plan[0].effect;
  work.computed = work.plan[0].computed;
}

void Core::Impl::resume_instruction(const InstructionEffect &effect) {
  auto &work = *staged;
  if (work.plan_index == work.planned) {
    compute_instruction();
    return;
  }
  if (effect != work.plan.at(work.plan_index).effect) {
    throw std::logic_error("Planned CPU publication lost its owned writer");
  }
  apply_publication(effect);
  ++work.plan_index;
  if (work.plan_index != work.planned) {
    work.pending = work.plan.at(work.plan_index).effect;
    work.computed = work.plan.at(work.plan_index).computed;
    return;
  }
  if (work.capacity_replay()) {
    // The old executor diagnoses the next operation while accepting the last
    // available CPU writer, before it can offer an unusable extra ticket.
    compute_instruction();
    return;
  }
  work.pending = std::exchange(work.following, std::nullopt);
  work.computed = work.following_computed;
}
} // namespace dsp56300
