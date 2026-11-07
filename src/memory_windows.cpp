// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace dsp56300 {
bool Core::Impl::bind_memory_windows() {
  constexpr std::array spaces{Space::x, Space::y};
  std::array<std::span<const MemoryWindow>, 2> windows;
  for (std::size_t bank = 0; bank < windows.size(); ++bank) {
    windows[bank] = io->memory_windows(spaces[bank]);
    Word end = 0;
    for (const auto &window : windows[bank]) {
      if (window.address < end || window.address >= detail::io_begin || window.words.empty() ||
          window.words.size() > detail::io_begin - window.address ||
          (!window.known.empty() && window.known.size() != window.words.size())) {
        current_pc = cpu.reg.pc;
        fail(FaultKind::host_event, spaces[bank], window.address);
        return false;
      }
      end = window.address + static_cast<Word>(window.words.size());
    }
  }
  if (!jit->bind_windows(windows)) {
    current_pc = cpu.reg.pc;
    fail(FaultKind::host_event, Space::program, current_pc);
    return false;
  }
  windows_bound = true;
  return true;
}
}
