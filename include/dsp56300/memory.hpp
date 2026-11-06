// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace dsp56300 {
using Word = std::uint32_t;
inline constexpr Word word_mask = 0x00ffffff;
inline constexpr std::size_t address_words = std::size_t{1} << 24;
enum class Space : std::uint8_t { program, x, y };

struct CacheRegion {
  Word begin = address_words, end = address_words;
  unsigned sector_words = 128;
};

struct Buffers {
  std::span<Word> program, x, y;
  CacheRegion cache{};
};

// Explicit ordinary storage grant. Empty known means all words are initialized;
// otherwise each byte is zero for unknown or nonzero for a known 24-bit word.
// Grants never include peripherals, side effects, wait states or unstable maps.
struct MemoryWindow {
  Word address = 0;
  std::span<Word> words;
  std::span<std::uint8_t> known;
  bool writable = false, aliases_program = false;
};

struct MemoryLayout {
  std::size_t program_words = 0, x_words = 0, y_words = 0;
};

// Borrowed memory; the consumer owns all backing storage and its lifetime.
class Memory {
public:
  [[nodiscard]] static constexpr Memory external(const CacheRegion cache = {}) noexcept {
    return Memory(Buffers{.program = {}, .x = {}, .y = {}, .cache = cache});
  }

  [[nodiscard]] static Memory separate(const Buffers buffers) {
    if (buffers.program.empty() || buffers.x.empty() || buffers.y.empty() ||
        buffers.program.size() > address_words || buffers.x.size() > address_words ||
        buffers.y.size() > address_words) {
      throw std::invalid_argument("DSP requires nonempty memory spans of at most 2^24 words");
    }
    return Memory(buffers);
  }

  // Explicit physical aliasing, including executable P aliases. Program identity
  // stays stable during execution; ordinary data is consumer-owned.
  [[nodiscard]] static Memory unified(const std::span<Word> words) {
    return separate({.program = words, .x = words, .y = words});
  }

  // One host allocation partitioned into disjoint Harvard spaces; tail may be unused.
  [[nodiscard]] static Memory partition(const std::span<Word> words, const MemoryLayout layout) {
    auto remaining = words.size();
    if (layout.program_words > remaining) {
      throw std::invalid_argument("DSP program partition exceeds backing storage");
    }
    remaining -= layout.program_words;
    if (layout.x_words > remaining) {
      throw std::invalid_argument("DSP X partition exceeds backing storage");
    }
    remaining -= layout.x_words;
    if (layout.y_words > remaining) {
      throw std::invalid_argument("DSP Y partition exceeds backing storage");
    }
    const auto after_program = words.subspan(layout.program_words);
    return separate({.program = words.first(layout.program_words),
                     .x = after_program.first(layout.x_words),
                     .y = after_program.subspan(layout.x_words, layout.y_words)});
  }

  [[nodiscard]] constexpr Buffers storage() const noexcept { return buffers_; }

private:
  explicit constexpr Memory(const Buffers buffers) noexcept : buffers_(buffers) {}

  Buffers buffers_;
};
}
