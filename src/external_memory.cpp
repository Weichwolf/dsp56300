// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "io.hpp"
#include "program_cache.hpp"

#include <optional>

namespace dsp56300 {
const MemoryWindow *Core::Impl::memory_window(const Space space,
                                              const Word address) const noexcept {
  if (io == nullptr || space == Space::program || address >= detail::io_begin) { return nullptr; }
  const auto windows = image ? image_windows[space == Space::x ? 0 : 1] : io->memory_windows(space);
  for (const auto &window : windows) {
    if (address >= window.address && address - window.address < window.words.size()) {
      return &window;
    }
  }
  return nullptr;
}

void Core::request_yield() noexcept {
  if (!impl_->executing) { return; }
  impl_->yield_requested = true;
}

std::optional<Word> Core::Impl::jit_program(void *owner, const Word address) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  if (self.io == nullptr || address > word_mask) { return std::nullopt; }
  const auto value = self.io->inspect_program(address);
  if (!value || *value > word_mask) { return std::nullopt; }
  return value;
}

Word Core::Impl::read_external(const Space space, const Word address) {
  if (fault.kind != FaultKind::none) { return 0; }
  if (space == Space::program && cacheable(address) && icache) {
    if (const auto cached = icache->peek(address)) { return *cached; }
  }
  if (const auto *window = memory_window(space, address)) {
    const auto offset = address - window->address;
    if (window->known.empty() || (offset < window->known.size() && window->known[offset] != 0)) {
      return window->words[offset] & word_mask;
    }
  }
  Word value = 0;
  if (address > word_mask || io == nullptr ||
      !dsp56300_io_read(*io, space, address, current_pc, value)) {
    const bool peripheral = space != Space::program && address >= detail::io_begin;
    fail(peripheral ? FaultKind::io_read : FaultKind::memory_read, space, address);
  }
  image_io_completed();
  return value & word_mask;
}

void Core::Impl::write_external(const Space space, const Word address, const Word value) {
  if (fault.kind != FaultKind::none) { return; }
  if (const auto *window = memory_window(space, address);
      window != nullptr && window->writable && !window->aliases_program) {
    const auto offset = address - window->address;
    if (window->known.empty() || offset < window->known.size()) {
      window->words[offset] = value & word_mask;
      if (!window->known.empty()) { window->known[offset] = 1; }
      return;
    }
  }
  if (address > word_mask || io == nullptr ||
      !dsp56300_io_write(*io, space, address, current_pc, value & word_mask)) {
    const bool peripheral = space != Space::program && address >= detail::io_begin;
    fail(peripheral ? FaultKind::io_write : FaultKind::memory_write, space, address, value);
    return;
  }
  if (space == Space::program) {
    image_word_written({.address = address, .value = value & word_mask});
  }
  image_io_completed();
  if (space == Space::program && cacheable(address) && icache) {
    icache->write({.address = address, .value = value});
  }
}
}
