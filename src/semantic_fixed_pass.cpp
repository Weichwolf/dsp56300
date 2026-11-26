// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "io.hpp"
#include "jit/fixed_pass.hpp"
#include "jit/fixed_pass_memory.hpp"
#include "semantic.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <tuple>
#include <utility>

namespace dsp56300 {
namespace {
using detail::SemanticFlow;
constexpr unsigned flow_shift = 32;

std::uint64_t outcome(const std::uint32_t retired, const SemanticFlow flow) noexcept {
  return (static_cast<std::uint64_t>(flow) << flow_shift) | retired;
}

struct PassBounds {
  Progress start, budget;
  std::uint64_t stable;
};

bool offer_handoff(auto &core,
                   const detail::PassEffects &effects,
                   const PassBounds bounds) noexcept {
  const auto now = core.cpu.cycles;
  if (core.proven_deadline <= now || core.io == nullptr || effects.device_reads.empty() ||
      core.fixed_pass_handoff_cycle == now) {
    return false;
  }
  constexpr auto end = std::numeric_limits<std::uint64_t>::max();
  const auto start = bounds.start.estimated_cycles;
  const auto ordinary =
      std::min(bounds.stable, start + std::min(bounds.budget.estimated_cycles, end - start));
  const auto extended = std::min(core.proven_deadline,
                                 core.io->stable_masked_reads_until(
                                     effects.device_reads, effects.device_masks, effects.writes));
  // A return pays for itself only if the device interval admits additional
  // complete turns beyond the ordinary local deadline.
  if (extended <= ordinary || (extended - ordinary - 1) / effects.cost.cycles < 2) { return false; }
  core.fixed_pass_handoff_cycle = now;
  core.fixed_pass_handoff_pending = true;
  core.yield_requested = true;
  return true;
}

std::uint64_t fold_pass(auto &core,
                        const detail::PassEffects &effects,
                        const PassBounds bounds,
                        detail::PassCommit &commit,
                        const bool handoff) noexcept {
  auto &cpu = core.cpu;
  const auto cost = effects.cost;
  constexpr auto end = std::numeric_limits<std::uint64_t>::max();
  const auto count = static_cast<std::uint32_t>(cpu.instructions - bounds.start.instructions);
  if (handoff && offer_handoff(core, effects, bounds)) {
    return outcome(count, SemanticFlow::owner);
  }
  if (bounds.stable <= cpu.cycles) { return outcome(count, SemanticFlow::branch); }
  const auto time = std::min({bounds.budget.estimated_cycles - commit.cycles,
                              bounds.stable - cpu.cycles,
                              end - cpu.cycles});
  const auto work = std::min(bounds.budget.instructions - count, end - cpu.instructions);
  const auto extra = std::min(time / cost.cycles, work / cost.instructions);
  if (extra == 0) { return outcome(count, SemanticFlow::branch); }
  cpu.instructions += extra * cost.instructions;
  cpu.cycles += extra * cost.cycles;
  commit.cycles = cpu.cycles - bounds.start.estimated_cycles;
  return outcome(static_cast<std::uint32_t>(cpu.instructions - bounds.start.instructions),
                 SemanticFlow::folded);
}

template <typename Observe>
bool observe_device_reads(const Io *const io,
                          const detail::PassEffects &effects,
                          const Observe &observe) noexcept {
  for (std::size_t i = 0; i < effects.device_reads.size(); ++i) {
    Word value = 0;
    if (io == nullptr || !io->stable_read_value(effects.device_reads[i], value) ||
        !observe(i, value & effects.device_masks[i])) {
      return false;
    }
  }
  return true;
}

bool matches_revision(const Io *const io, const std::uint64_t revision) noexcept {
  return io == nullptr || dsp56300_io_revision(*io) == revision;
}

bool same_execution_state(const auto &saved, const auto &owner) noexcept {
  // Folded passes advance progress counters while preserving the rest of the state.
  const auto control = [](const detail::Cpu &cpu) {
    return std::tie(cpu.repeat_remaining,
                    cpu.repeat_pc,
                    cpu.repeat_lc,
                    cpu.repeat_opcode,
                    cpu.repeat_loaded,
                    cpu.repeat_completed,
                    cpu.interrupt_inhibited,
                    cpu.processing,
                    cpu.debug_flags);
  };
  return saved.current_pc == owner.current_pc && saved.debug_vector == owner.debug_vector &&
         saved.vector == owner.vector && control(saved.cpu) == control(owner.cpu) &&
         saved.cpu.reg == owner.cpu.reg;
}

bool retained_pass_matches(const auto &owner,
                           const detail::PassEffects &effects,
                           const auto proof) noexcept {
  const auto &memo = owner.fixed_pass_memo;
  if (!memo || memo->proof != proof || memo->revision != owner.image_revision ||
      !same_execution_state(memo->state, owner)) {
    return false;
  }
  return detail::same_pass_memory(std::span(memo->cells).first(memo->cell_count)) &&
         observe_device_reads(owner.io, effects, [&](const std::size_t index, const Word value) {
           return memo->device_values[index] == value;
         });
}
}

std::uint64_t Core::Impl::semantic_fixed_pass(void *const owner,
                                              const Word head,
                                              const FixedPassEntry proof,
                                              const std::uint64_t window,
                                              detail::PassWord *const scratch,
                                              const std::uint32_t quota,
                                              detail::PassCommit &commit) noexcept {
  return checked_fixed_pass(owner, head, proof, window, scratch, quota, commit, 0);
}

std::uint64_t Core::Impl::checked_fixed_pass(void *const owner,
                                             const Word head,
                                             const FixedPassEntry proof,
                                             const std::uint64_t window,
                                             detail::PassWord *const scratch,
                                             const std::uint32_t quota,
                                             detail::PassCommit &commit,
                                             const std::uint64_t retained_horizon) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  auto &cpu = self.cpu;
  commit.next = head;
  commit.cycles = 0;
  const auto available = [&] {
    return self.image_ready && self.image_storage_admitted && self.active_image != nullptr &&
           self.next_images == nullptr && self.fault.kind == FaultKind::none &&
           !self.yield_requested && self.vector.phase == VectorPhase::none &&
           !self.instruction_cache_active() && cpu.processing == ExecutionState::running &&
           cpu.debug_flags == 0 && !cpu.interrupt_inhibited && cpu.repeat_remaining == 0 &&
           !cpu.repeat_loaded && cpu.reg.pc == head;
  };
  if (!available()) { return outcome(0, SemanticFlow::sequential); }
  const auto *const variant = self.active_image;
  const auto found = variant->fixed_passes.find(head);
  if (found == variant->fixed_passes.end()) { return outcome(0, SemanticFlow::sequential); }
  const auto &effects = found->second;
  const auto cost = effects.cost;
  constexpr auto end = std::numeric_limits<std::uint64_t>::max();
  const auto until = [&] {
    if (effects.device_reads.empty()) { return end; }
    return self.io != nullptr ? self.io->stable_reads_until(effects.device_reads) : 0;
  };
  const auto horizon = retained_horizon == 0 ? until() : retained_horizon;
  const auto instructions = cpu.instructions;
  const auto start = cpu.cycles;
  if (horizon <= start || std::min({horizon - start, window, end - start}) / cost.cycles < 2 ||
      std::min<std::uint64_t>(quota, end - instructions) / cost.instructions < 2) {
    return outcome(0, SemanticFlow::sequential);
  }
  const auto revision = self.image_revision;
  if (!matches_revision(self.io, revision)) { return outcome(0, SemanticFlow::sequential); }
  PassBounds bounds{.start = {.instructions = instructions, .estimated_cycles = start},
                    .budget = {.instructions = quota, .estimated_cycles = window},
                    .stable = horizon};
  if (retained_pass_matches(self, effects, proof)) {
    return fold_pass(self, effects, bounds, commit, retained_horizon == 0);
  }
  if (retained_horizon != 0) { return outcome(0, SemanticFlow::sequential); }

  const FixedPassState before{.cpu = cpu,
                              .vector = self.vector,
                              .current_pc = self.current_pc,
                              .debug_vector = self.debug_vector};
  const auto flat = [&](const std::span<Word> words, const std::size_t bank) {
    return MemoryWindow{.words = words.first(std::min(words.size(), std::size_t{detail::io_begin})),
                        .known = {},
                        .writable = true,
                        .aliases_program = self.aliases_program[bank]};
  };
  const std::array x{flat(self.buffers.x, 1)};
  const std::array y{flat(self.buffers.y, 2)};
  const auto windows = self.external ? self.image_windows : detail::PassWindows{x, y};
  const std::span writes(scratch, effects.writes.size());
  for (auto &entry : writes) { std::construct_at(std::addressof(entry)); }
  std::array<detail::PassWord, fixed_pass_observations> cells{};
  detail::PassFootprint footprint{.cells = cells};
  std::array<Word, fixed_pass_observations> devices{};
  const bool reusable = self.canonical_data && effects.device_reads.size() <= devices.size() &&
                        observe_device_reads(self.io, effects, [&](std::size_t i, Word value) {
                          devices[i] = value;
                          return true;
                        });
  if (!detail::capture_pass_memory(effects,
                                   cpu.reg,
                                   windows,
                                   writes,
                                   self.canonical_data,
                                   reusable ? &footprint : nullptr)) {
    return outcome(0, SemanticFlow::sequential);
  }

  const auto result = proof(owner);
  const auto flow = static_cast<SemanticFlow>(result >> flow_shift);
  commit.next = static_cast<Word>(result);
  const auto count = static_cast<std::uint32_t>(cpu.instructions - instructions);
  commit.cycles = cpu.cycles - start;
  if (flow == SemanticFlow::fault || flow == SemanticFlow::owner) { return outcome(count, flow); }
  if (count != cost.instructions || commit.cycles != cost.cycles || !available() ||
      self.active_image != variant || self.image_revision != revision ||
      !matches_revision(self.io, revision) || !same_execution_state(before, self) ||
      !detail::same_pass_memory(writes)) {
    return outcome(count, SemanticFlow::branch);
  }
  if (reusable && footprint.complete) {
    self.fixed_pass_memo = FixedPassMemo{.state = before,
                                         .cells = cells,
                                         .device_values = devices,
                                         .proof = proof,
                                         .revision = revision,
                                         .cell_count = footprint.used};
  }
  bounds.stable = std::min(horizon, until());
  return fold_pass(self, effects, bounds, commit, reusable && footprint.complete);
}

RunResult Core::Impl::advance_proven_pass(const std::uint32_t quota) {
  auto &self = *this;
  const auto &memo = self.fixed_pass_memo;
  const auto head = self.cpu.reg.pc;
  if (!memo || quota == 0 || self.proven_deadline <= self.cpu.cycles || self.executing ||
      self.terminal_fault || self.io == nullptr || self.active_image == nullptr ||
      memo->state.cpu.reg.pc != head) {
    return {};
  }
  if (std::exchange(self.fixed_pass_handoff_pending, false) && self.next_images == nullptr &&
      self.image_ready && self.fixed_pass_handoff_cycle == self.cpu.cycles) {
    self.yield_requested = false;
  }
  const auto found = self.active_image->fixed_passes.find(head);
  if (found == self.active_image->fixed_passes.end()) { return {}; }
  const detail::ExecutionGuard guard(self.executing);
  const auto &effects = found->second;
  const auto exclusive = std::min(self.proven_deadline,
                                  self.io->stable_masked_reads_until(
                                      effects.device_reads, effects.device_masks, effects.writes));
  if (exclusive <= self.cpu.cycles) { return {}; }
  detail::PassCommit commit{};
  const auto result = Impl::checked_fixed_pass(
      &self, head, memo->proof, exclusive - self.cpu.cycles - 1, nullptr, quota, commit, exclusive);
  if ((result >> flow_shift) != static_cast<unsigned>(SemanticFlow::folded)) { return {}; }
  return {.retired = static_cast<std::uint32_t>(result),
          .state = self.cpu.processing,
          .progress = {.instructions = self.cpu.instructions, .estimated_cycles = self.cpu.cycles},
          .fault = {},
          .yielded = false};
}
}
