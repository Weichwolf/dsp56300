// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "image.hpp"
#include "image_entry.hpp"
#include "io.hpp"
#include "jit/llvm/image.hpp"
#include "program_cache.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <llvm/ADT/StringExtras.h>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300 {
namespace {
using Windows = std::array<std::span<const MemoryWindow>, 2>;

Windows windows(Io *const io) {
  Windows result;
  if (io == nullptr) { return result; }
  constexpr std::array spaces{Space::x, Space::y};
  for (std::size_t bank = 0; bank < result.size(); ++bank) {
    result[bank] = io->memory_windows(spaces[bank]);
    Word end = 0;
    for (const auto &window : result[bank]) {
      if (window.words.empty() || window.address < end || window.address >= detail::io_begin ||
          window.words.size() > detail::io_begin - window.address ||
          (!window.known.empty() && window.known.size() != window.words.size())) {
        throw std::invalid_argument("Invalid DSP image memory window");
      }
      end = window.address + static_cast<Word>(window.words.size());
    }
  }
  return result;
}

bool equal(const RegionLayout &left, const RegionLayout &right) {
  return left.space == right.space && left.first == right.first && left.size == right.size &&
         left.writable == right.writable && left.tracked == right.tracked &&
         left.aliases_program == right.aliases_program;
}

std::vector<RegionLayout> layout(const Buffers buffers,
                                 const Windows &windows,
                                 const bool external,
                                 const std::array<bool, 3> &aliases) {
  std::vector<RegionLayout> result;
  if (external) {
    constexpr std::array spaces{Space::x, Space::y};
    for (std::size_t bank = 0; bank < windows.size(); ++bank) {
      for (const auto &window : windows[bank]) {
        result.push_back({.space = spaces[bank],
                          .first = window.address,
                          .size = window.words.size(),
                          .writable = window.writable,
                          .tracked = !window.known.empty(),
                          .aliases_program = window.aliases_program});
      }
    }
  } else {
    const std::array banks{buffers.program, buffers.x, buffers.y};
    constexpr std::array spaces{Space::program, Space::x, Space::y};
    for (std::size_t bank = 0; bank < banks.size(); ++bank) {
      result.push_back({.space = spaces[bank],
                        .first = 0,
                        .size = banks[bank].size(),
                        .aliases_program = bank != 0 && aliases[bank]});
    }
  }
  return result;
}

void validate_program(const auto &core, const detail::ImageContracts::Variant &selected) {
  for (const auto &[address, expected] : selected.words) {
    const auto actual = core.known_program_word(address);
    if (!actual || *actual != expected) {
      throw std::invalid_argument("DSP image code mismatch at P:$" +
                                  llvm::utohexstr(address, true, 6));
    }
  }
  for (const auto address : selected.failed_fetches) {
    if (core.known_program_word(address)) {
      throw std::invalid_argument("DSP expected failed fetch has readable source at P:$" +
                                  llvm::utohexstr(address, true, 6));
    }
  }
}

void handle_image_miss(auto &core,
                       const bool vector_fetch,
                       const Word address,
                       const bool retired) {
  if (!vector_fetch && core.active_image->failed_fetches.contains(core.cpu.reg.pc)) {
    core.current_pc = core.cpu.reg.pc;
    const auto value = core.fetch_program(core.cpu.reg.pc);
    if (core.fault.kind == FaultKind::none) { core.unprepared(core.cpu.reg.pc, value); }
    return;
  }
  if (vector_fetch || !core.image_transition(retired)) {
    core.unprepared(vector_fetch ? address : core.cpu.reg.pc);
  }
}

}

void Core::install(const NativeImage &image, const std::uint64_t variant) {
  auto &self = *impl_;
  if (self.executing) {
    throw std::logic_error("Image installation requires stopped DSP execution");
  }
  const detail::ExecutionGuard guard(self.executing);
  if (!image.impl_) { throw std::invalid_argument("Cannot install an empty DSP image"); }
  const auto &selected = image.impl_->variant(variant);
  if (self.next_image != nullptr &&
      (image.impl_ != self.image || selected.id != self.next_image->id)) {
    throw std::invalid_argument("DSP transfer requires its declared target image variant");
  }
  if (self.debug_ready() || self.instruction_cache_active()) {
    throw std::invalid_argument("DSP image does not prepare the current debug/cache fetch form");
  }
  if (self.vector.phase != Impl::VectorPhase::none &&
      (!image.impl_->interrupts.contains({self.vector.address, self.vector.level}) ||
       (self.vector.loaded &&
        image.impl_->vector_entry(self.vector.address, self.vector.words) == nullptr))) {
    throw std::invalid_argument("DSP image does not prepare the retained vector continuation");
  }
  const auto revision = self.io != nullptr ? dsp56300_io_revision(*self.io) : 0;
  const auto granted = self.external ? windows(self.io) : Windows{};
  if (selected.fixed_memory &&
      !std::ranges::equal(selected.memory,
                          layout(self.buffers, granted, self.external, self.aliases_program),
                          equal)) {
    throw std::invalid_argument("DSP image memory layout differs from this owner");
  }
  validate_program(self, selected);
  if (self.cpu.repeat_loaded) {
    const auto found = selected.instructions.find(self.cpu.reg.pc);
    if (found == selected.instructions.end() || found->second != self.repeated) {
      throw std::invalid_argument("DSP image differs from the latched REP instruction");
    }
  }
  if (self.io != nullptr && revision != dsp56300_io_revision(*self.io)) {
    throw std::invalid_argument("DSP image binding changed during installation");
  }
  self.image = image.impl_;
  self.active_image = &selected;
  self.image_windows = granted;
  self.image_revision = revision;
  self.program_omr = self.cpu.reg.omr;
  self.image_ready = true;
  self.next_image = nullptr;
}

void Core::Impl::unprepared(const Word address, const Word value) noexcept {
  current_pc = cpu.reg.pc;
  fail(FaultKind::unprepared_execution, Space::program, address, value);
}

RunResult Core::Impl::run_image(const std::uint32_t quota) {
  RunResult result;
  if (next_image != nullptr) { yield_requested = true; }
  while (result.retired < quota && cpu.cycles < cycle_deadline && fault.kind == FaultKind::none &&
         cpu.processing == ExecutionState::running && !yield_requested) {
    if (!image_ready || (external && program_omr != cpu.reg.omr) ||
        (io != nullptr && image_revision != dsp56300_io_revision(*io)) || debug_ready() ||
        instruction_cache_active()) {
      unprepared(cpu.reg.pc);
      break;
    }
    const bool vector_fetch = exception_ready();
    const auto entry = vector_fetch ? image_vector() : active_image->entry;
    if (entry == nullptr) { break; }
    const auto address = image_address(vector_fetch);
    const auto outcome = detail::dsp56300_image_enter(
        entry, this, address, quota - result.retired, cycle_deadline - cpu.cycles);
    constexpr unsigned reason_shift = 32;
    const auto retired = static_cast<std::uint32_t>(outcome);
    result.retired += retired;
    if ((outcome >> reason_shift) == static_cast<unsigned>(detail::ImageExit::unprepared)) {
      handle_image_miss(*this, vector_fetch, address, result.retired != 0);
    }
    if (retired == 0) { break; }
  }
  static_cast<void>(image_transition(result.retired != 0));
  result.progress = {.instructions = cpu.instructions, .estimated_cycles = cpu.cycles};
  result.fault = fault;
  result.state = cpu.processing;
  result.yielded = yield_requested;
  return result;
}

Word Core::Impl::image_address(const bool vector_fetch) const noexcept {
  if (!vector_fetch) { return cpu.reg.pc; }
  return (vector.address + (vector.phase == VectorPhase::second ? 1U : 0U)) & word_mask;
}

bool Core::Impl::image_transition(const bool eligible) {
  if (!eligible || next_image != nullptr || active_image->transitions.empty() ||
      fault.kind != FaultKind::none || cpu.processing == ExecutionState::debugging ||
      exception_ready() || debug_ready() || instruction_cache_active()) {
    return false;
  }
  const auto transfer = active_image->transitions.find(cpu.reg.pc);
  if (transfer == active_image->transitions.end()) { return false; }
  next_image = &image->variant(transfer->second);
  image_ready = false;
  yield_requested = true;
  return true;
}

bool Core::Impl::image_interrupt(const Word address, const unsigned level) {
  if (next_image != nullptr) {
    throw std::logic_error("DSP interrupt admission requires the pending image installation");
  }
  if (!image_ready || (external && program_omr != cpu.reg.omr) ||
      (io != nullptr && image_revision != dsp56300_io_revision(*io)) || debug_ready() ||
      instruction_cache_active() || !image->interrupts.contains({address, level}) ||
      !active_image->words.contains(address) || !active_image->words.contains(address + 1)) {
    unprepared(address, level);
    return false;
  }
  return true;
}

detail::ImageEntry Core::Impl::image_vector() {
  if (!image->interrupts.contains({vector.address, vector.level})) {
    unprepared(vector.address, vector.level);
    return nullptr;
  }
  auto words = vector.words;
  if (!vector.loaded) {
    const auto first = active_image->words.find(vector.address);
    const auto second = active_image->words.find(vector.address + 1);
    if (first == active_image->words.end() || second == active_image->words.end()) {
      unprepared(vector.address);
      return nullptr;
    }
    words = {first->second, second->second};
  }
  const auto entry = image->vector_entry(vector.address, words);
  if (entry == nullptr) {
    unprepared(vector.address);
    return nullptr;
  }
  vector.words = words;
  vector.loaded = true;
  return entry;
}

void Core::Impl::image_word_written(const detail::ProgramCache::Write transfer) noexcept {
  if (active_image == nullptr) { return; }
  const auto found = active_image->words.find(transfer.address);
  if (found == active_image->words.end() || found->second == transfer.value) { return; }
  image_ready = false;
  yield_requested = true;
}

void Core::Impl::image_io_completed() noexcept {
  if (!image || io == nullptr) { return; }
  const bool remapped = external && program_omr != cpu.reg.omr;
  if (!remapped && image_revision == dsp56300_io_revision(*io)) { return; }
  image_ready = false;
  yield_requested = true;
  if (!external) { return; }
  constexpr std::array spaces{Space::x, Space::y};
  for (std::size_t bank = 0; bank < image_windows.size(); ++bank) {
    image_windows[bank] = io->memory_windows(spaces[bank]);
    Word end = 0;
    for (const auto &window : image_windows[bank]) {
      if (window.words.empty() || window.address < end || window.address >= detail::io_begin ||
          window.words.size() > detail::io_begin - window.address ||
          (!window.known.empty() && window.known.size() != window.words.size())) {
        fail(FaultKind::host_event, spaces[bank], window.address);
        image_windows = {};
        return;
      }
      end = window.address + static_cast<Word>(window.words.size());
    }
  }
}
}
