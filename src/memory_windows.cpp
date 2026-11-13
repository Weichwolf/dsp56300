// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "io.hpp"
#include "jit.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace dsp56300 {
Fault Core::Impl::bind_memory_windows() {
  constexpr std::array spaces{Space::x, Space::y};
  std::array<std::span<const MemoryWindow>, 2> windows;
  for (std::size_t bank = 0; bank < windows.size(); ++bank) {
    windows[bank] = detail::io_windows(*io, spaces[bank], cpu.reg.omr);
    Word end = 0;
    for (const auto &window : windows[bank]) {
      if (window.address < end || window.address >= detail::io_begin || window.words.empty() ||
          window.words.size() > detail::io_begin - window.address ||
          (!window.known.empty() && window.known.size() != window.words.size())) {
        return {.kind = FaultKind::host_event,
                .space = spaces[bank],
                .address = window.address,
                .pc = cpu.reg.pc};
      }
      end = window.address + static_cast<Word>(window.words.size());
    }
  }
  if (!jit->bind_windows(windows)) {
    return {.kind = FaultKind::host_event,
            .space = Space::program,
            .address = cpu.reg.pc,
            .pc = cpu.reg.pc};
  }
  windows_bound = true;
  return {};
}
}
