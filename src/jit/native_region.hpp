// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"

#include <array>
#include <span>

namespace dsp56300::detail {
enum class RegionMode : unsigned char { automatic, ordinary, guarded };

struct RegionAddress {
  Word bound = io_begin, linear_limit = io_begin, advances = 0, modifier = 0;
  bool accessed = false, updated = false;
};

struct NativeRegion {
  std::array<RegionAddress, agu_registers> addresses;
  Word transfers = 0;
  bool valid = false, loop = false;
};

[[nodiscard]] NativeRegion plan_native_region(std::span<const NativeOp> operations,
                                              const std::array<JitMemory, 2> &memory,
                                              const std::array<Word, agu_registers> &modifiers);
} // namespace dsp56300::detail
