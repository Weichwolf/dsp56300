// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "io.hpp"
#include "jit.hpp"
#include "memory_views.hpp"
#include "semantic.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>

namespace dsp56300 {
std::uint64_t Core::Impl::semantic_paired_read(void *const owner,
                                               const std::uint32_t budget,
                                               const detail::SemanticPairedPortRead &read,
                                               const std::uint64_t window,
                                               std::uint64_t &cycles) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  auto &cpu = self.cpu;
  auto &registers = cpu.reg;
  cycles = 0;
  const auto &[body, end, agu_index, per_turn, transfer] = read;
  const auto cycle_cost = std::uint64_t{transfer.cycles};
  const auto modes = registers.sr & detail::numeric_mode_mask;
  // Preserve the terminal DO restore, admitted IRQ transitions and suspended REP.
  if (self.io == nullptr || !self.image_storage_admitted || self.exception_ready() ||
      registers.pc != body || registers.la != end || registers.lc <= 1 || registers.sc < 2 ||
      (registers.sr & (detail::loop_flag | detail::forever_flag)) != detail::loop_flag ||
      (modes != 0 && modes != detail::rounding_mode && modes != detail::scale_up &&
       modes != detail::scale_down) ||
      (registers.m[agu_index] & detail::modifier_mask) != detail::modifier_mask ||
      cpu.repeat_remaining != 0 || cpu.repeat_loaded || cpu.repeat_completed ||
      cpu.interrupt_inhibited ||
      (registers.stack[registers.sp & (stack_entries - 1)] >> detail::word_bits) != body) {
    return 0;
  }
  const auto turns = static_cast<Word>(std::min(
      {std::uint64_t{registers.lc - 1}, std::uint64_t{budget / per_turn}, window / cycle_cost}));
  if (turns == 0 || turns > word_mask - registers.r[agu_index]) { return 0; }
  const auto requested_cycles = turns * cycle_cost;
  const auto requested_retirement = turns * per_turn;
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (requested_cycles > maximum - cpu.cycles ||
      requested_retirement > maximum - cpu.instructions) {
    return 0;
  }
  constexpr unsigned count_shift = 32;
  const auto request = (std::uint64_t{turns} << count_shift) | registers.r[agu_index];
  const auto transferred = dsp56300_io_paired_read(self.io, &transfer, request);
  if (transferred == 0) { return 0; }
  const auto completed = static_cast<Word>(transferred >> count_shift);
  if (completed == 0 || completed > turns) { std::terminate(); }
  const auto elapsed = completed * cycle_cost;
  const auto retired = completed * per_turn;

  constexpr unsigned main_shift = detail::word_bits + detail::padding_bits;
  constexpr auto a1_mask = std::uint64_t{word_mask} << main_shift;
  constexpr Word high_byte = 0xff0000;
  const auto word = static_cast<Word>(transferred) & word_mask;
  registers.a = (registers.a & ~a1_mask) | (std::uint64_t{word} << main_shift);
  registers.x = (registers.x & word_mask) | (std::uint64_t{word & high_byte} << detail::word_bits);
  constexpr Word flags = detail::carry | detail::overflow | detail::negative | detail::zero;
  registers.sr &= ~flags;
  registers.sr |= word == 0 ? detail::zero : 0;
  registers.sr |= (word & detail::sign24_bit) != 0 ? detail::negative : 0;
  registers.r[agu_index] += completed;
  registers.lc -= completed;
  self.current_pc = end;
  cpu.cycles += elapsed;
  cpu.instructions += retired;
  cycles = elapsed;
  self.image_io_completed();
  constexpr unsigned owner_shift = 32;
  return retired | (std::uint64_t{self.yield_requested} << owner_shift);
}

std::uint64_t Core::Impl::semantic_region(void *const owner,
                                          const std::uint32_t budget,
                                          const detail::SemanticRegion &region,
                                          const std::uint64_t window,
                                          std::uint64_t &cycles) noexcept {
  cycles = 0;
  auto &self = *static_cast<Impl *>(owner);
  self.assume_image_admission();
  const bool ordinary = !self.exception_ready();
  __builtin_assume(ordinary);
  constexpr unsigned pc_shift = 32;
  if (!self.image_storage_admitted) {
    return static_cast<std::uint64_t>(self.cpu.reg.pc) << pc_shift;
  }
  detail::NativeContext context{
      .runtime = {.memory = {detail::JitMemory{.data = detail::dsp56300_memory_direct()
                                                           ? self.views[1].read.data.data()
                                                           : self.buffers.x.data(),
                                               .words = self.buffers.x.size(),
                                               .aliases_program = self.aliases_program[1]},
                             detail::JitMemory{.data = detail::dsp56300_memory_direct()
                                                           ? self.views[2].read.data.data()
                                                           : self.buffers.y.data(),
                                               .words = self.buffers.y.size(),
                                               .aliases_program = self.aliases_program[2]}},
                  .windows = self.image_windows,
                  .writes = {self.views[1].write.data.data(), self.views[2].write.data.data()},
                  .admitted_memory = true},
      .budget = budget,
      .cycle_window = window};
  const auto before = self.cpu.cycles;
  const auto retired = region.entry(&self.cpu, &context);
  cycles = self.cpu.cycles - before;
  if (retired != 0) { self.current_pc = region.pc; }
  return retired | (static_cast<std::uint64_t>(self.cpu.reg.pc) << pc_shift);
}
}
