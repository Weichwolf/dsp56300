// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
// Architectural DSP instruction storage, independent of decoded/native host caches.
class ProgramCache {
public:
  static constexpr std::size_t sector_count = 8, maximum_sector_words = 256;

  struct Sector {
    std::bitset<maximum_sector_words> valid;
    Word tag = 0;
    bool locked = false;
  };

  struct Write {
    Word address = 0, value = 0;
  };

  explicit ProgramCache(unsigned sector_words);
  using Reader = std::optional<Word> (*)(void *context, Word address);
  [[nodiscard]] std::optional<Word> peek(Word address) const;
  [[nodiscard]] std::optional<Word>
  fetch(Word address, std::span<const Word> memory, bool burst, Word cacheable_end);
  [[nodiscard]] std::optional<Word> fetch(
      Word address, std::size_t size, bool burst, Word cacheable_end, Reader reader, void *context);
  void write(Write transfer);
  void lock(Word address, bool locked);
  void free();
  void flush(bool unlocked_only);
  [[nodiscard]] const std::array<Sector, sector_count> &sectors() const noexcept;
  [[nodiscard]] std::span<const Word> contents(std::size_t slot) const noexcept;
  [[nodiscard]] const std::array<std::size_t, sector_count> &order() const noexcept;

private:
  [[nodiscard]] std::size_t find(Word address) const;
  [[nodiscard]] std::size_t acquire(Word address);
  void touch(std::size_t slot);
  std::array<Sector, sector_count> sectors_{};
  std::vector<Word> words_;
  // MRU first, LRU last. Locked entries remain in the order but cannot be replaced.
  std::array<std::size_t, sector_count> order_{};
  unsigned sector_words_;
};
} // namespace dsp56300::detail
