// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <compare>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct ImagePlan;

struct FixedPassCost {
  std::uint32_t instructions;
  std::uint64_t cycles;
  bool operator==(const FixedPassCost &) const = default;
};

using PassCell = StableRead;

struct PassAddress {
  Word offset;
  std::optional<PassCell> base = std::nullopt;
  auto operator<=>(const PassAddress &) const = default;
};

struct PassRead {
  Space space;
  PassAddress address;
  auto operator<=>(const PassRead &) const = default;
};

struct PassEffects {
  FixedPassCost cost;
  std::vector<PassRead> reads;
  std::vector<PassCell> writes, device_reads;
  std::vector<Word> device_masks;
  std::uint8_t linear = 0;
};

[[nodiscard]] std::optional<PassEffects> fixed_pass_effects(const ImagePlan &image,
                                                            std::span<const Word> path,
                                                            std::span<const PassCell> stable_reads);
}
