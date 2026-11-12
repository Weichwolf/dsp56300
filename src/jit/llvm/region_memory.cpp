// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region_memory.hpp"

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::detail {
namespace {
bool overlap(const std::uintptr_t first,
             const std::size_t first_size,
             const std::uintptr_t second,
             const std::size_t second_size) {
  return first <= second ? second - first < first_size : first - second < second_size;
}

bool storage(const Cpu &cpu,
             const NativeContext &owner,
             const void *pointer,
             const std::size_t bytes) {
  const auto first = std::bit_cast<std::uintptr_t>(pointer);
  return !overlap(first, bytes, std::bit_cast<std::uintptr_t>(&cpu), sizeof(cpu)) &&
         !overlap(first, bytes, std::bit_cast<std::uintptr_t>(&owner), sizeof(owner));
}

std::optional<MemoryWindow> window(const NativeContext &owner, const RegionMemory &memory) {
  if (memory.bank >= owner.runtime.windows.size()) { return {}; }
  const auto views = owner.runtime.windows[memory.bank];
  if ((memory.flags & region_windowed) != 0) {
    if (views.size() != memory.regions || memory.index >= views.size()) { return {}; }
    return views[memory.index];
  }
  const auto &flat = owner.runtime.memory[memory.bank];
  if (!views.empty() || memory.regions != 1 || memory.index != 0 || flat.data == nullptr) {
    return {};
  }
  return MemoryWindow{.words = {flat.data, std::min(flat.words, std::size_t{io_begin})},
                      .known = {},
                      .writable = true,
                      .aliases_program = flat.aliases_program};
}

bool knowledge(const JitRuntime &runtime, const MemoryWindow &selected) {
  const auto first = std::bit_cast<std::uintptr_t>(selected.words.data());
  for (const auto views : runtime.windows) {
    for (const auto &view : views) {
      if (!view.known.empty() && overlap(first,
                                         selected.words.size_bytes(),
                                         std::bit_cast<std::uintptr_t>(view.known.data()),
                                         view.known.size())) {
        return false;
      }
    }
  }
  return true;
}

MemoryWindow bound_window(const JitRuntime &runtime, const RegionMemory &memory) {
  if ((memory.flags & region_windowed) != 0) { return runtime.windows[memory.bank][memory.index]; }
  const auto &flat = runtime.memory[memory.bank];
  return {.words = {flat.data, std::min(flat.words, std::size_t{io_begin})},
          .known = {},
          .writable = true,
          .aliases_program = flat.aliases_program};
}

bool admitted_window(const std::span<const std::byte> state,
                     const JitRuntime &runtime,
                     const MemoryWindow &view) {
  const auto outside = [&](const auto bytes) {
    return !overlap(std::bit_cast<std::uintptr_t>(bytes.data()),
                    bytes.size(),
                    std::bit_cast<std::uintptr_t>(state.data()),
                    state.size());
  };
  return !view.words.empty() && view.words.data() != nullptr &&
         (view.known.empty() || view.known.size() == view.words.size()) &&
         outside(std::as_bytes(view.words)) && outside(std::as_bytes(view.known)) &&
         (!view.writable || view.aliases_program || knowledge(runtime, view));
}

bool matching(const MemoryWindow &view, const RegionMemory &memory) {
  const auto flags = (view.writable ? region_writable : 0U) |
                     (!view.known.empty() ? region_tracked : 0U) |
                     (view.aliases_program ? region_program_alias : 0U);
  constexpr Word layout_flags = region_writable | region_tracked | region_program_alias;
  return view.address == memory.first && view.words.size() == memory.words &&
         (memory.flags & layout_flags) == flags && !view.words.empty() &&
         view.words.data() != nullptr &&
         (view.known.empty() || view.known.size() == view.words.size()) &&
         ((memory.flags & region_writing) == 0 || (view.writable && !view.aliases_program));
}
}

bool admit_region_memory(const std::span<const std::byte> state,
                         const JitRuntime &runtime) noexcept {
  for (std::size_t bank = 0; bank < runtime.windows.size(); ++bank) {
    if (!runtime.windows[bank].empty()) {
      for (const auto &view : runtime.windows[bank]) {
        if (!admitted_window(state, runtime, view)) { return false; }
      }
    } else if (const auto &flat = runtime.memory[bank]; flat.data != nullptr) {
      const MemoryWindow view{.words = {flat.data, flat.words},
                              .known = {},
                              .writable = true,
                              .aliases_program = flat.aliases_program};
      if (!admitted_window(state, runtime, view)) { return false; }
    }
  }
  return true;
}

Word dsp56300_region_memory(const Cpu *const cpu,
                            const NativeContext *const owner,
                            const RegionMemory *const memory,
                            Word **const data,
                            std::uint8_t **const known) noexcept {
  if (owner->runtime.admitted_memory) {
#ifndef NDEBUG
    const auto checked = window(*owner, *memory);
    assert(checked && matching(*checked, *memory));
#endif
    const auto view = bound_window(owner->runtime, *memory);
    assert(storage(*cpu, *owner, view.words.data(), view.words.size_bytes()));
    assert(view.known.empty() || storage(*cpu, *owner, view.known.data(), view.known.size()));
    assert((memory->flags & region_writing) == 0 || knowledge(owner->runtime, view));
    *data = view.words.data();
    *known = view.known.empty() ? nullptr : view.known.data();
    return 1;
  }
  const auto selected = window(*owner, *memory);
  if (!selected) { return 0; }
  const auto &view = *selected;
  const bool writing = (memory->flags & region_writing) != 0;
  if (!matching(view, *memory) || (writing && !knowledge(owner->runtime, view)) ||
      !storage(*cpu, *owner, view.words.data(), view.words.size_bytes()) ||
      (!view.known.empty() && !storage(*cpu, *owner, view.known.data(), view.known.size()))) {
    return 0;
  }
  *data = view.words.data();
  *known = view.known.empty() ? nullptr : view.known.data();
  return 1;
}
}
