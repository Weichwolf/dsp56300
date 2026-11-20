// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "image.hpp"
#include "io.hpp"
#include "jit.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/prefixes.hpp"
#include "memory_views.hpp"
#include "native_call.hpp"
#include "program_cache.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <llvm/ADT/StringExtras.h>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300 {

std::optional<Word>
detail::ImageContracts::Variant::source_word(const Word address) const noexcept {
  // NOLINTNEXTLINE(misc-include-cleaner) -- <algorithm> provides the ranges overload.
  const auto after = std::ranges::upper_bound(program, address, {}, &ProgramRun::first);
  if (after == program.begin()) { return {}; }
  const auto &run = *(after - 1);
  const auto offset = address - run.first;
  return offset < run.words.size() ? std::optional(run.words[offset]) : std::nullopt;
}

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

Windows windows(const auto &core) {
  if (core.complete_memory()) {
    detail::validate_memory_views(core.regions, core.views);
    return {core.regions[1], core.regions[2]};
  }
  return core.external ? windows(core.io) : Windows{};
}

bool storage(const auto &core, const Windows &granted) {
  // The exact region strategy uses one virtual pointer per memory stream.
  // Distinct aliases instead need the ordered, checked instruction lowering.
  if (core.complete_memory() && std::ranges::any_of(core.views, [](const auto &view) {
        return view.read.data.data() != view.write.data.data();
      })) {
    return false;
  }
  const detail::JitRuntime memory{
      .memory = {detail::JitMemory{.data = core.buffers.x.data(),
                                   .words = core.buffers.x.size(),
                                   .aliases_program = core.aliases_program[1]},
                 detail::JitMemory{.data = core.buffers.y.data(),
                                   .words = core.buffers.y.size(),
                                   .aliases_program = core.aliases_program[2]}},
      .windows = granted};
  return detail::admit_region_memory(std::as_bytes(std::span(&core, 1)), memory);
}

void validate_direct(const auto &core, const auto &image) {
  if (image.memory_mode != CompileOptions::MemoryMode::direct) { return; }
  if (!core.complete_memory()) {
    throw std::invalid_argument("Direct DSP execution requires complete memory views");
  }
  for (const auto &bank : core.regions) {
    for (const auto &region : bank) {
      if (!region.known.empty() || (region.writable && region.aliases_program)) {
        throw std::invalid_argument(
            "Direct DSP data requires untracked storage without writable program aliases");
      }
    }
  }
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

std::vector<RegionLayout> layout(const std::array<std::span<const MemoryWindow>, 3> &regions) {
  std::vector<RegionLayout> result;
  for (std::size_t bank = 0; bank < regions.size(); ++bank) {
    for (const auto &region : regions[bank]) {
      result.push_back({.space = static_cast<Space>(bank),
                        .first = region.address,
                        .size = region.words.size(),
                        .writable = region.writable,
                        .tracked = !region.known.empty(),
                        .aliases_program = region.aliases_program});
    }
  }
  return result;
}

std::size_t matching_program_words(const MemoryWindow &window,
                                   const Word address,
                                   const std::span<const Word> expected) {
  const auto offset = address - window.address;
  auto count = expected.size();
  if (!window.known.empty()) {
    const auto known = window.known.subspan(offset, count);
    count = static_cast<std::size_t>(std::ranges::find(known, std::uint8_t{0}) - known.begin());
  }
  const auto actual = window.words.subspan(offset, count);
  if (count == 0 || std::memcmp(actual.data(), expected.data(), actual.size_bytes()) == 0) {
    return count;
  }
  return static_cast<std::size_t>(std::ranges::mismatch(actual, expected).in1 - actual.begin());
}

void validate_program(const auto &core, const detail::ImageContracts::Variant &selected) {
  // Observe writes through the stopped owner's other host mapping before reading P.
  std::atomic_signal_fence(std::memory_order_acq_rel);
  for (const auto &run : selected.program) {
    for (std::size_t offset = 0; offset < run.words.size();) {
      const auto address = run.first + static_cast<Word>(offset);
      const auto *window = core.memory_window(Space::program, address);
      std::size_t matched = 0;
      std::size_t count = 1;
      if (window != nullptr) {
        count =
            std::min(run.words.size() - offset, window->words.size() - (address - window->address));
        matched = matching_program_words(
            *window, address, std::span<const Word>(run.words).subspan(offset, count));
      } else {
        matched = core.known_program_word(address) == run.words[offset] ? 1U : 0U;
      }
      if (matched != count) {
        throw std::invalid_argument("DSP image code mismatch at P:$" +
                                    llvm::utohexstr(address + matched, true, 6));
      }
      offset += count;
    }
  }
  for (const auto address : selected.failed_fetches) {
    if (core.known_program_word(address)) {
      throw std::invalid_argument("DSP expected failed fetch has readable source at P:$" +
                                  llvm::utohexstr(address, true, 6));
    }
  }
}

bool retained_repeat(const auto &core) {
  if (!core.cpu.repeat_loaded || core.cpu.repeat_remaining == 0 ||
      core.cpu.repeat_pc != core.cpu.reg.pc || core.active_image == nullptr) {
    return false;
  }
  const auto captured = core.active_image->instructions.find(core.cpu.reg.pc);
  return captured != core.active_image->instructions.end() && captured->second == core.repeated;
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

std::uint64_t Core::foreground_prefix() const {
  const auto &self = *impl_;
  if (self.executing || self.fault != Fault{} || !self.image_ready || self.next_images != nullptr ||
      self.active_image == nullptr ||
      (self.active_image->fixed_memory && !self.image_storage_admitted) ||
      self.vector.phase != Impl::VectorPhase::none || self.instruction_cache_active() ||
      (self.external && self.program_omr != self.cpu.reg.omr) ||
      (self.io != nullptr && self.image_revision != dsp56300_io_revision(*self.io))) {
    return 0;
  }
  return detail::native_prefix(self.active_image->instructions,
                               self.active_image->memory,
                               self.active_image->prefixes,
                               self.cpu);
}

void Core::install(const NativeImage &image, const std::uint64_t variant) {
  auto &self = *impl_;
  if (self.executing || self.terminal_fault) {
    throw std::logic_error("Image installation requires stopped DSP execution");
  }
  const detail::ExecutionGuard guard(self.executing);
  if (!image.impl_) { throw std::invalid_argument("Cannot install an empty DSP image"); }
  const auto &selected = image.impl_->variant(variant);
  validate_direct(self, *image.impl_);
  if (self.next_images != nullptr &&
      (image.impl_ != self.image ||
       std::ranges::find(*self.next_images, selected.id) == self.next_images->end())) {
    throw std::invalid_argument("DSP transfer requires a declared target image variant");
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
  const auto granted = windows(self);
  const auto actual = self.complete_memory()
                          ? layout(self.regions)
                          : layout(self.buffers, granted, self.external, self.aliases_program);
  if (selected.fixed_memory && !std::ranges::equal(selected.memory, actual, equal)) {
    throw std::invalid_argument("DSP image memory layout differs from this owner");
  }
  validate_program(self, selected);
  if (self.cpu.repeat_loaded) {
    const auto found = selected.instructions.find(self.cpu.reg.pc);
    if (found == selected.instructions.end() || found->second != self.repeated) {
      throw std::invalid_argument("DSP image differs from the latched REP instruction");
    }
  }
  const bool admitted = image.impl_->memory_mode == CompileOptions::MemoryMode::direct ||
                        (selected.fixed_memory && storage(self, granted));
  if (self.io != nullptr && revision != dsp56300_io_revision(*self.io)) {
    throw std::invalid_argument("DSP image binding changed during installation");
  }
  auto native_call =
      image.impl_->memory_mode == CompileOptions::MemoryMode::direct
          ? std::make_unique<detail::NativeCall>(image.impl_->text_ranges, self.views)
          : nullptr;
  self.native_call = std::move(native_call);
  self.image = image.impl_;
  self.active_image = &selected;
  self.image_windows = granted;
  self.image_storage_admitted = admitted;
  self.image_revision = revision;
  self.program_omr = self.cpu.reg.omr;
  self.image_ready = true;
  self.next_images = nullptr;
}

void Core::Impl::unprepared(const Word address, const Word value) noexcept {
  current_pc = cpu.reg.pc;
  fail(FaultKind::unprepared_execution, Space::program, address, value);
}

RunResult Core::Impl::run_image(const std::uint32_t quota) {
  RunResult result;
  if (next_images != nullptr) { yield_requested = true; }
  while (result.retired < quota && cpu.cycles < cycle_deadline && fault.kind == FaultKind::none &&
         cpu.processing == ExecutionState::running && !yield_requested) {
    if ((!image_ready && !retained_repeat(*this)) || (external && program_omr != cpu.reg.omr) ||
        (io != nullptr && image_revision != dsp56300_io_revision(*io)) || debug_ready() ||
        instruction_cache_active()) {
      unprepared(cpu.reg.pc);
      break;
    }
    const bool vector_fetch = exception_ready();
    const auto entry = vector_fetch ? image_vector() : active_image->entry;
    if (entry == nullptr) { break; }
    const auto address = image_address(vector_fetch);
    // A captured REP opcode survives P writes, but no fresh instruction may use
    // the invalidated binding after that exact architectural continuation ends.
    const auto allowance = image_ready ? quota - result.retired
                                       : std::min(quota - result.retired, cpu.repeat_remaining);
    const auto outcome = call_image(entry, address, allowance, cycle_deadline - cpu.cycles);
    constexpr unsigned reason_shift = 32;
    const auto retired = static_cast<std::uint32_t>(outcome);
    result.retired += retired;
    if ((outcome >> reason_shift) == static_cast<unsigned>(detail::ImageExit::unprepared)) {
      handle_image_miss(*this, vector_fetch, address, result.retired != 0);
    }
    if (!image_ready) { static_cast<void>(image_transition(result.retired != 0)); }
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
  if (!eligible || next_images != nullptr || active_image->transitions.empty() ||
      fault.kind != FaultKind::none || cpu.processing == ExecutionState::debugging ||
      exception_ready() || debug_ready() || instruction_cache_active()) {
    return false;
  }
  const auto transfer = active_image->transitions.find(cpu.reg.pc);
  if (transfer == active_image->transitions.end()) { return false; }
  next_images = &transfer->second;
  image_ready = false;
  yield_requested = true;
  return true;
}

bool Core::Impl::image_interrupt(const Word address, const unsigned level) {
  if (next_images != nullptr) {
    throw std::logic_error("DSP interrupt admission requires the pending image installation");
  }
  if (!image_ready || (external && program_omr != cpu.reg.omr) ||
      (io != nullptr && image_revision != dsp56300_io_revision(*io)) || debug_ready() ||
      instruction_cache_active() || !image->interrupts.contains({address, level}) ||
      !active_image->source_word(address) || !active_image->source_word(address + 1)) {
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
    const auto first = active_image->source_word(vector.address);
    const auto second = active_image->source_word(vector.address + 1);
    if (!first || !second) {
      unprepared(vector.address);
      return nullptr;
    }
    words = {*first, *second};
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
  const auto word = active_image->source_word(transfer.address);
  if (!word) { return; }
  const auto value =
      detail::dsp56300_memory_direct()
          ? static_cast<const volatile Word *>(views[0].read.data.data())[transfer.address]
          : transfer.value;
  if (*word == value) { return; }
  image_ready = false;
  yield_requested = true;
}

void Core::Impl::image_io_completed() noexcept {
  if (!image) { return; }
  const bool remapped = external && program_omr != cpu.reg.omr;
  const bool revised = io != nullptr && image_revision != dsp56300_io_revision(*io);
  if (!remapped && !revised) { return; }
  image_ready = false;
  yield_requested = true;
  if (!external || complete_memory() || io == nullptr) { return; }
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
