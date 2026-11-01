// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "program_cache.hpp"

namespace dsp56300 {
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
  if (space == Space::program && cacheable(address) && icache) {
    icache->write({.address = address, .value = value});
  }
}
}
