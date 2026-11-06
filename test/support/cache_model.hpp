// SPDX-License-Identifier: GPL-3.0-only
// Independent associative map and age-based cache model, not the executor's LRU list.
#pragma once
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>

namespace dsp56300::test::cache_model {
inline constexpr std::size_t slots = 8;

struct Write {
  Word address = 0, value = 0;
};

struct Sector {
  Word tag = 0;
  std::map<Word, Word> words;
  std::uint64_t age = 0;
  bool locked = false;
};

struct Model {
  explicit Model(const Word words) : width(words) { flush(false); }

  Word width;
  std::uint64_t clock = slots;
  std::array<Sector, slots> sectors{};

  [[nodiscard]] std::size_t locate(const Word address) const {
    for (std::size_t index = 0; index < slots; ++index) {
      if (sectors[index].tag == address / width) { return index; }
    }
    return slots;
  }

  [[nodiscard]] std::size_t allocate(const Word address) {
    const auto found = locate(address);
    if (found != slots) { return found; }
    std::size_t selected = slots;
    for (std::size_t index = 0; index < slots; ++index) {
      if (!sectors[index].locked &&
          (selected == slots || sectors[index].age < sectors[selected].age)) {
        selected = index;
      }
    }
    if (selected != slots) {
      sectors[selected].tag = address / width;
      sectors[selected].words.clear();
    }
    return selected;
  }

  [[nodiscard]] std::optional<Word> peek(const Word address) const {
    const auto found = locate(address);
    if (found == slots) { return std::nullopt; }
    const auto &words = sectors[found].words;
    const auto word = words.find(address % width);
    if (word == words.end()) { return std::nullopt; }
    return word->second;
  }

  [[nodiscard]] std::optional<Word>
  fetch(const Word address, const std::span<const Word> memory, const bool burst, const Word end) {
    if (address >= memory.size()) { return std::nullopt; }
    const auto found = allocate(address);
    if (found == slots) { return memory[address] % (word_mask + 1); }
    sectors[found].age = ++clock;
    if (const auto value = peek(address)) { return value; }
    Word next = address;
    do {
      sectors[found].words[next % width] = memory[next] % (word_mask + 1);
      ++next;
    } while (burst && next % 4 != 0 && next < end && next < memory.size());
    return peek(address);
  }

  void write(const Write transfer) {
    const auto address = transfer.address;
    const auto found = locate(address);
    if (found != slots) {
      sectors[found].words[address % width] = transfer.value % (word_mask + 1);
    }
  }

  void lock(const Word address, const bool locked) {
    const auto found = allocate(address);
    if (found == slots) { return; }
    sectors[found].locked = locked;
    sectors[found].age = ++clock;
  }

  void free() {
    for (auto &sector : sectors) { sector.locked = false; }
  }

  void flush(const bool unlocked_only) {
    clock = slots;
    for (std::size_t index = 0; index < slots; ++index) {
      auto &sector = sectors[index];
      sector.age = slots - index;
      if (!unlocked_only || !sector.locked) {
        sector.words.clear();
        sector.tag = word_mask / width;
        sector.locked = false;
      }
    }
  }
};
} // namespace dsp56300::test::cache_model
