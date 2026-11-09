// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <array>
#include <optional>
#include <span>

namespace dsp56300::detail {
enum class RegionMode : unsigned char { automatic, ordinary, guarded };
enum class LoopResidency : unsigned char { automatic, disabled };
enum class LoopWindowMode : unsigned char { automatic, disabled };

struct RegionAddress {
  Word bound = io_begin, linear_limit = io_begin, advances = 0, modifier = 0;
  Word stride_steps = 0, stride_accesses = 0;
  std::optional<Word> constant;
  bool accessed = false, updated = false;
};

struct ResidentPlan {
  std::array<RegionAddress, agu_registers> addresses;
  Word transfers = 0;
  bool valid = false, loop = false;
};

[[nodiscard]] JitRuntime resident_runtime(JitRuntime runtime);
[[nodiscard]] ResidentPlan resident_plan(std::span<const BlockInstruction> operations,
                                         const std::array<JitMemory, 2> &memory,
                                         const std::array<Word, agu_registers> &modifiers);
} // namespace dsp56300::detail
