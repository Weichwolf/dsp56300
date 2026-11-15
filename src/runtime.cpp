// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "io.hpp"

namespace dsp56300 {
void Core::Impl::check_interrupt_window(const Word previous_status) noexcept {
  constexpr Word interrupt_mask = 0x300;
  if (yield_requested || fault.kind != FaultKind::none || io == nullptr ||
      ((previous_status ^ cpu.reg.sr) & interrupt_mask) == 0) {
    return;
  }
  if (dsp56300_io_pending(*io, cpu.reg.sr)) { yield_requested = true; }
  image_io_completed();
}

} // namespace dsp56300
