// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
inline constexpr Word region_writable = 1, region_tracked = 2, region_program_alias = 4,
                      region_windowed = 8, region_writing = 16;

struct RegionMemory {
  Word bank, index, regions, first, words, flags;
};

[[nodiscard]] bool admit_region_memory(std::span<const std::byte> state,
                                       const JitRuntime &runtime) noexcept;

// Resolve actual owner storage after checking the prepared layout. No device or
// CPU effects; a rejected binding leaves both output pointers untouched.
extern "C" Word dsp56300_region_memory(const Cpu *cpu,
                                       const NativeContext *owner,
                                       const RegionMemory *memory,
                                       Word **data,
                                       std::uint8_t **known) noexcept;
}
