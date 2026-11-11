// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region_memory.hpp"

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <algorithm>
#include <bit>
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

bool knowledge(const NativeContext &owner, const MemoryWindow &selected) {
  const auto first = std::bit_cast<std::uintptr_t>(selected.words.data());
  for (const auto views : owner.runtime.windows) {
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
}

Word dsp56300_region_memory(const Cpu *const cpu,
                            const NativeContext *const owner,
                            const RegionMemory *const memory,
                            Word **const data,
                            std::uint8_t **const known) noexcept {
  const auto selected = window(*owner, *memory);
  if (!selected) { return 0; }
  const auto &view = *selected;
  const auto flags = (view.writable ? region_writable : 0U) |
                     (!view.known.empty() ? region_tracked : 0U) |
                     (view.aliases_program ? region_program_alias : 0U);
  constexpr Word layout_flags = region_writable | region_tracked | region_program_alias;
  const bool writing = (memory->flags & region_writing) != 0;
  if (view.address != memory->first || view.words.size() != memory->words ||
      (memory->flags & layout_flags) != flags || view.words.empty() ||
      view.words.data() == nullptr ||
      (!view.known.empty() && view.known.size() != view.words.size()) ||
      (writing && (!view.writable || view.aliases_program || !knowledge(*owner, view))) ||
      !storage(*cpu, *owner, view.words.data(), view.words.size_bytes()) ||
      (!view.known.empty() && !storage(*cpu, *owner, view.known.data(), view.known.size()))) {
    return 0;
  }
  *data = view.words.data();
  *known = view.known.empty() ? nullptr : view.known.data();
  return 1;
}
}
