// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/move.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct RegionControl {
  Word target = 0;
  unsigned selector = 16, reg = agu_registers;
  bool valid = false;
};

struct RegionWindow {
  Word address = 0;
  std::size_t size = 0;
  bool tracked = false, writable = true, aliases_program = false;
  Word *data = nullptr;
  std::uint8_t *known = nullptr;
};

struct RegionPlan {
  struct Step {
    NativeMoves memory;
    NativeRegisterMove registers;
  };

  struct Extent {
    std::int64_t minimum = 0, maximum = 0;
    bool accessed = false, writing = false;
  };

  std::span<const BlockInstruction> operations;
  std::vector<Step> steps;
  std::array<std::vector<RegionWindow>, 2> windows;
  std::array<bool, 2> windowed{};
  std::array<std::array<Extent, agu_registers>, 2> extents{};
  std::array<std::int64_t, agu_registers> deltas{};
  std::array<bool, agu_registers> changed_r{}, changed_n{}, agu{};
  RegionControl control;
  Word cost = 0, count = 0;
  bool batch = false, computation = false;
};

[[nodiscard]] bool permitted(const RegionWindow &window, bool writing);
[[nodiscard]] std::optional<RegionPlan> plan_region(std::span<const BlockInstruction> operations,
                                                    const JitRuntime &runtime);
[[nodiscard]] std::optional<RegionPlan> plan_region(std::span<const BlockInstruction> operations,
                                                    std::span<const RegionLayout> memory);
}
