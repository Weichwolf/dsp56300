// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"
#include "move.hpp"
#include "value_ranges.hpp"

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
bool stable_load(const NativeMove &move) {
  return move.load && !move.immediate() && move.reg < r_register_base &&
         (move.address.mode == 4 || move.address.mode == 6);
}

bool same_address(const NativeMove &load, const NativeMove &store) {
  if (load.space != store.space || load.address.index != store.address.index) { return false; }
  if (load.address.mode == 6) {
    return store.address.mode == 6 && load.address.extra == store.address.extra;
  }
  return store.address.mode == 1 || store.address.mode == 3 || store.address.mode == 4;
}
}

std::vector<BlockEmitter::WindowReuse>
BlockEmitter::plan_window_reuse(const std::span<const BlockInstruction> operations) const {
  std::vector<WindowReuse> result(operations.size());
  // XMM14/15 are free in ordinary entries. A failed RAM guard executes the
  // complete instruction and leaves, so no owner call can invalidate a witness.
  if (repeat_mode_ || region_mode_ != RegionMode::ordinary || range_mode_ != RangeMode::automatic) {
    return result;
  }
  for (std::size_t index = 1; index < operations.size(); ++index) {
    const auto &before = operations[index - 1];
    const auto &after = operations[index];
    if (at_loop_end(before) || !direct_memory(before) || !direct_memory(after)) { continue; }
    const auto reads = plan_moves(before.instruction);
    const auto writes = plan_moves(after.instruction);
    if (reads.long_pair || writes.long_pair || reads.count == 0 || writes.count == 0) { continue; }
    const auto loads = std::span(reads.transfers).first(reads.count);
    const auto stores = std::span(writes.transfers).first(writes.count);
    if (!std::ranges::all_of(loads, stable_load) ||
        !std::ranges::all_of(stores, [](const auto &move) { return !move.load; })) {
      continue;
    }
    for (const auto &store : stores) {
      const auto bank = store.space == Space::x ? 0U : 1U;
      if (runtime_.windows[bank].empty()) { continue; }
      if (std::ranges::any_of(loads,
                              [&store](const auto &load) { return same_address(load, store); })) {
        result[index - 1].capture |= Word{1} << bank;
        result[index].restore |= Word{1} << bank;
      }
    }
  }
  return result;
}
}
