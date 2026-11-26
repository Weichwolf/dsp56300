// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/fixed_pass.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
struct PassCommit {
  Word next;
  std::uint64_t cycles;
};

struct PassWord {
  const Word *word = nullptr;
  const std::uint8_t *known = nullptr;
  Word before = 0;
  std::uint8_t known_before = 0;
};

struct PassFootprint {
  std::span<PassWord> cells;
  std::size_t used = 0;
  bool complete = true;

  void retain(const PassWord &cell) noexcept {
    if (used == cells.size()) {
      complete = false;
      return;
    }
    cells[used++] = cell;
  }
};

using PassWindows = std::array<std::span<const MemoryWindow>, 2>;

// The caller owns the current grant and rechecks its revision before using the witness.
[[nodiscard]] bool capture_pass_memory(const PassEffects &effects,
                                       const Registers &registers,
                                       const PassWindows &windows,
                                       std::span<PassWord> writes,
                                       bool canonical_data,
                                       PassFootprint *footprint = nullptr) noexcept;
[[nodiscard]] bool same_pass_memory(std::span<const PassWord> writes) noexcept;
}
