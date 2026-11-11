// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "image.hpp"
#include "image_entry.hpp"
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
}

void Core::install(const NativeImage &image, const std::uint64_t variant) {
  auto &self = *impl_;
  if (self.executing) {
    throw std::logic_error("Image installation requires stopped DSP execution");
  }
  const detail::ExecutionGuard guard(self.executing);
  if (!image.impl_) { throw std::invalid_argument("Cannot install an empty DSP image"); }
  const auto &selected = image.impl_->variant(variant);
  if (self.exception_ready() || self.debug_ready() || self.instruction_cache_active()) {
    throw std::invalid_argument("DSP image does not prepare the current vector/cache fetch form");
  }
  const auto revision = self.io != nullptr ? self.io->program_revision() : 0;
  const auto granted = self.external ? windows(self.io) : Windows{};
  if (selected.fixed_memory &&
      !std::ranges::equal(selected.memory,
                          layout(self.buffers, granted, self.external, self.aliases_program),
                          equal)) {
    throw std::invalid_argument("DSP image memory layout differs from this owner");
  }
  for (const auto &[address, expected] : selected.words) {
    const auto actual = self.known_program_word(address);
    if (!actual || *actual != expected) {
      throw std::invalid_argument("DSP image code mismatch at P:$" +
                                  llvm::utohexstr(address, true, 6));
    }
  }
  if (self.cpu.repeat_loaded) {
    const auto found = selected.instructions.find(self.cpu.reg.pc);
    if (found == selected.instructions.end() || found->second != self.repeated) {
      throw std::invalid_argument("DSP image differs from the latched REP instruction");
    }
  }
  if (self.io != nullptr && revision != self.io->program_revision()) {
    throw std::invalid_argument("DSP image binding changed during installation");
  }
  self.image = image.impl_;
  self.active_image = &selected;
  self.image_windows = granted;
  self.image_revision = revision;
  self.image_ready = true;
}

void Core::Impl::unprepared(const Word address, const Word value) noexcept {
  current_pc = cpu.reg.pc;
  fail(FaultKind::unprepared_execution, Space::program, address, value);
}

RunResult Core::Impl::run_image(const std::uint32_t quota) {
  RunResult result;
  if (quota != 0 && cpu.cycles < cycle_deadline && fault.kind == FaultKind::none &&
      cpu.processing == ExecutionState::running) {
    if (!image_ready || (io != nullptr && image_revision != io->program_revision()) ||
        exception_ready() || debug_ready() || instruction_cache_active()) {
      unprepared(cpu.reg.pc);
    } else {
      const auto window = cycle_deadline - cpu.cycles;
      const auto outcome =
          detail::dsp56300_image_enter(active_image->entry, this, cpu.reg.pc, quota, window);
      constexpr unsigned reason_shift = 32;
      result.retired = static_cast<std::uint32_t>(outcome);
      if ((outcome >> reason_shift) == static_cast<unsigned>(detail::ImageExit::unprepared)) {
        unprepared(cpu.reg.pc);
      }
    }
  }
  result.progress = {.instructions = cpu.instructions, .estimated_cycles = cpu.cycles};
  result.fault = fault;
  result.state = cpu.processing;
  result.yielded = yield_requested;
  return result;
}

void Core::Impl::image_word_written(const detail::ProgramCache::Write transfer) noexcept {
  if (active_image == nullptr) { return; }
  const auto found = active_image->words.find(transfer.address);
  if (found == active_image->words.end() || found->second == transfer.value) { return; }
  image_ready = false;
  yield_requested = true;
}

void Core::Impl::image_io_completed() noexcept {
  if (!image || io == nullptr || image_revision == io->program_revision()) { return; }
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
