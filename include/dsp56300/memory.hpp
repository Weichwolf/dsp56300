// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"

#include <cstddef>
#include <span>
#include <stdexcept>

namespace dsp56300 {
struct MemoryLayout {
  std::size_t program_words = 0, x_words = 0, y_words = 0;
};

// Borrowed physical backing with explicit logical P/X/Y views. No hot dispatch.
class Memory {
public:
  [[nodiscard]] static constexpr Memory external() noexcept { return {{}, true}; }

  [[nodiscard]] static Memory separate(const Buffers buffers) {
    if (buffers.program.empty() || buffers.x.empty() || buffers.y.empty() ||
        buffers.program.size() > address_words || buffers.x.size() > address_words ||
        buffers.y.size() > address_words) {
      throw std::invalid_argument("DSP requires nonempty memory spans of at most 2^24 words");
    }
    return {buffers, false};
  }

  // Explicit physical aliasing, including executable P aliases. Mutate through Core.
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

  [[nodiscard]] constexpr bool externally_owned() const noexcept { return external_; }

private:
  constexpr Memory(const Buffers buffers, const bool external) noexcept
      : buffers_(buffers), external_(external) {}

  Buffers buffers_;
  bool external_;
};
}
