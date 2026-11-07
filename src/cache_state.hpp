// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
inline constexpr std::size_t cache_sectors = 8, cache_sector_words = 256, cache_valid_bits = 64;

struct CacheValidity {
  std::array<std::uint64_t, cache_sector_words / cache_valid_bits> words{};

  [[nodiscard]] bool test(const std::size_t offset) const {
    return (words.at(offset / cache_valid_bits) &
            (std::uint64_t{1} << (offset % cache_valid_bits))) != 0;
  }

  void set(const std::size_t offset) {
    words.at(offset / cache_valid_bits) |= std::uint64_t{1} << (offset % cache_valid_bits);
  }

  void reset() { words.fill(0); }
};

struct CacheSector {
  CacheValidity valid;
  Word tag = 0;
  bool locked = false;
};

struct CacheState {
  std::array<CacheSector, cache_sectors> sectors{};
  std::array<std::size_t, cache_sectors> order{};
  unsigned sector_words = 0;
};
}
