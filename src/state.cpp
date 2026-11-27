// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace dsp56300 {
namespace {
constexpr Word stack_pointer_mask = 0x3f;
constexpr Word stack_counter_mask = 0x1f;
}

bool Core::Impl::valid_registers(const Registers &state) noexcept {
  const std::array words{state.pc,
                         state.sr,
                         state.omr,
                         state.la,
                         state.lc,
                         state.sp,
                         state.sc,
                         state.sz,
                         state.ep,
                         state.vba};
  const auto invalid = [](const Word word) { return word > word_mask; };
  return !std::ranges::any_of(words, invalid) && !std::ranges::any_of(state.r, invalid) &&
         !std::ranges::any_of(state.n, invalid) && !std::ranges::any_of(state.m, invalid) &&
         !std::ranges::any_of(state.stack, [](const auto word) { return word > long_mask; }) &&
         state.x <= long_mask && state.y <= long_mask && state.a <= accumulator_mask &&
         state.b <= accumulator_mask && state.sp <= stack_pointer_mask &&
         state.sc <= stack_counter_mask;
}

bool Core::Impl::plain_foreground() const noexcept {
  return image_ready && active_image != nullptr && fault.kind == FaultKind::none &&
         cpu.processing == ExecutionState::running && cpu.repeat_remaining == 0 &&
         !cpu.repeat_loaded && !cpu.repeat_completed && !cpu.interrupt_inhibited &&
         cpu.debug_flags == 0 && vector.phase == VectorPhase::none && !instruction_cache_active();
}

std::uint32_t Core::Impl::semantic_foreground(void *const owner,
                                              const Word pc,
                                              const std::uint32_t quota,
                                              const std::uint64_t window) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  auto &cpu = self.cpu;
  if (self.io == nullptr || self.next_images != nullptr || self.yield_requested ||
      cpu.reg.pc != pc || !self.plain_foreground()) {
    return 0;
  }
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  const auto extension = self.proven_deadline > cpu.cycles ? self.proven_deadline - cpu.cycles : 0;
  const ForegroundGrant grant{.pc = pc,
                              .instructions = quota,
                              .precise_cycles = window,
                              .foreground_cycles = std::max(window, extension),
                              .variant = self.active_image->id,
                              .revision = self.image_revision};
  const auto *offer = self.io->offer_foreground(grant);
  // A committed offer exits the native image. Its next entry selects the
  // received scaling mode and complete architectural loop state afresh.
  constexpr Word transferable_status =
      0xff | detail::scale_up | detail::scale_down | detail::loop_flag | detail::forever_flag;
  if (offer == nullptr || offer->work.instructions == 0 || offer->work.instructions > quota ||
      offer->work.instructions > maximum - cpu.instructions || offer->work.estimated_cycles == 0 ||
      offer->work.estimated_cycles > grant.foreground_cycles ||
      offer->work.estimated_cycles > maximum - cpu.cycles || offer->registers.omr != cpu.reg.omr ||
      ((offer->registers.sr ^ cpu.reg.sr) & ~transferable_status) != 0 ||
      !self.active_image->instructions.contains(offer->last_pc) ||
      !self.active_image->instructions.contains(offer->registers.pc) ||
      !valid_registers(offer->registers)) {
    return 0;
  }
  if (offer->publish != nullptr) { offer->publish(offer->context); }
  cpu.reg = offer->registers;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions += offer->work.instructions;
  cpu.cycles += offer->work.estimated_cycles;
  self.current_pc = offer->last_pc;
  return static_cast<std::uint32_t>(offer->work.instructions);
}

bool Core::capture_foreground(ForegroundCommit &output) const noexcept {
  const auto &self = *impl_;
  const auto &cpu = self.cpu;
  if (self.executing || !self.plain_foreground() || cpu.instructions == 0 || cpu.repeat_pc != 0 ||
      cpu.repeat_lc != 0 || cpu.repeat_opcode != 0 || self.repeated != detail::Instruction{} ||
      self.debug_vector != 0) {
    return false;
  }
  output = {.registers = registers(), .work = progress(), .last_pc = self.current_pc};
  return true;
}

Registers Core::registers() const {
  auto result = impl_->cpu.reg;
  result.a >>= detail::padding_bits;
  result.b >>= detail::padding_bits;
  return result;
}

void Core::set_registers(const Registers &state) {
  if (impl_->next_images != nullptr) {
    throw std::logic_error(
        "Register initialization requires reset before a pending image transfer");
  }

  if (impl_->executing || impl_->terminal_fault) {
    throw std::logic_error("Register initialization requires stopped DSP execution");
  }
  if (!Impl::valid_registers(state)) {
    throw std::invalid_argument("DSP register value exceeds its architectural width");
  }
  impl_->cpu.reg = state;
  impl_->cpu.reg.a <<= detail::padding_bits;
  impl_->cpu.reg.b <<= detail::padding_bits;
  impl_->synchronize_cache_mode();
}
} // namespace dsp56300
