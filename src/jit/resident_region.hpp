// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "value_graph.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct RegionRoot {
  Space space = Space::x;
  unsigned address = 0;
};

struct alignas(16) RegionCursor {
  Word *pointer = nullptr;
  std::uint64_t stride = 0, inner_stride = 0, inner_step_bytes = 0;
  std::int64_t step_bytes = 0;
};

struct alignas(32) RegionVector {
  std::array<std::uint64_t, 4> words{};
};

struct ReverseCursor {
  std::uint64_t index = 0;
  Word mask = 0, shift = 0;
};

struct RegionFrame {
  std::array<RegionCursor, 2 * agu_registers> cursors;
  std::array<std::uint64_t, 6> invariants{}, results{};
  std::array<std::uint64_t, 4> flags{}, preceding_flags{};
  std::array<Word, agu_registers> addresses{};
  RegionVector growth, limits, preceding_growth, preceding_limits;
  const std::uint64_t *constants = nullptr;
  const std::uint64_t *offsets = nullptr;
  RegionVector *spills = nullptr;
  Word iterations = 0;
  std::array<ReverseCursor, agu_registers> reverse{};
};

struct RegionFootprint {
  std::uintptr_t first = 0;
  std::uint64_t bytes = 0, stride = 0, inner_span = 0;
};

struct ResidentRegion {
  ValueGraph graph;
  JitRuntime runtime;
  std::array<MemoryWindow, 2> memory;
  std::vector<RegionRoot> roots;
  std::vector<RegionFootprint> footprints;
  std::vector<unsigned> stream_roots;
  std::vector<std::uint64_t> constants, offsets;
  std::vector<RegionVector> spills;
  std::array<std::optional<Word>, agu_registers> prefix{};
  std::array<GraphAddress, agu_registers> inner_addresses{};
  RegionFrame frame;
  Word entry = 0, last = 0, mode = 0, inner_entry = 0, extra_cycles = 0, inner_iterations = 1;
  Word reverse_addresses = 0;
  bool nested = false;
  std::array<const void *, 3> native{};
  unsigned lanes = 4, growth_shift = 0;
};

[[nodiscard]] GraphSupport resident_region_support() noexcept;

[[nodiscard]] std::unique_ptr<ResidentRegion>
resident_region_plan(std::span<const Word> program,
                     const Registers &state,
                     const JitRuntime &runtime,
                     std::span<const BlockInstruction> operations);
[[nodiscard]] std::vector<std::uint8_t> emit_resident_region(ResidentRegion &region,
                                                             unsigned lanes);
[[nodiscard]] std::vector<std::uint8_t>
select_resident_region(ResidentRegion &region, std::span<const std::uint8_t> fallback);
[[nodiscard]] bool enter_resident_region(const void *entry, RegionFrame &frame) noexcept;
[[nodiscard]] std::uint32_t
run_resident_region(Cpu *cpu, NativeContext *context, ResidentRegion *region) noexcept;
} // namespace dsp56300::detail
