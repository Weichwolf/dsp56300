// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "program_cache.hpp"

#include <optional>

namespace dsp56300 {
Backend Core::backend() const noexcept {
  return impl_->options.backend;
}

void Core::request_yield() noexcept {
  if (!impl_->executing) { return; }
  impl_->yield_requested = true;
  if (impl_->jit) { impl_->jit->request_yield(); }
}

std::optional<Word> Core::Impl::jit_program(void *owner, const Word address) noexcept {
  const auto &self = *static_cast<Impl *>(owner);
  if (self.io == nullptr || address > word_mask) { return std::nullopt; }
  const auto value = self.io->inspect_program(address);
  return value && *value <= word_mask ? value : std::nullopt;
}

void Core::Impl::synchronize_program_view() {
  if (!external || !jit || io == nullptr) { return; }
  const auto next = io->program_revision();
  if (!windows_bound || next != program_revision || program_omr != cpu.reg.omr) {
    if (!bind_memory_windows()) { return; }
    if (program_omr != cpu.reg.omr) {
      yield_requested = true;
      jit->request_yield();
    }
    jit->invalidate();
    program_revision = next;
    program_omr = cpu.reg.omr;
  }
}

Word Core::Impl::read_external(const Space space, const Word address) {
  if (fault.kind != FaultKind::none) { return 0; }
  if (space == Space::program && cacheable(address) && icache) {
    if (const auto cached = icache->peek(address)) { return *cached; }
  }
  Word value = 0;
  if (address > word_mask || io == nullptr || !io->read(space, address, current_pc, value)) {
    const bool peripheral = space != Space::program && address >= detail::io_begin;
    fail(peripheral ? FaultKind::io_read : FaultKind::memory_read, space, address);
  }
  return value & word_mask;
}

void Core::Impl::write_external(const Space space, const Word address, const Word value) {
  if (fault.kind != FaultKind::none) { return; }
  if (address > word_mask || io == nullptr ||
      !io->write(space, address, current_pc, value & word_mask)) {
    const bool peripheral = space != Space::program && address >= detail::io_begin;
    fail(peripheral ? FaultKind::io_write : FaultKind::memory_write, space, address, value);
    return;
  }
  if (space == Space::program && jit) { jit->invalidate(); }
  if (space == Space::program && cacheable(address) && icache) {
    icache->write({.address = address, .value = value});
  }
}
}
