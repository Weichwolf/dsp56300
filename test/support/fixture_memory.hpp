// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
using FixtureMemory = std::array<std::vector<Word>, 3>;

inline void patch(FixtureMemory &memory, const std::vector<MemoryBlock> &blocks) {
  for (const auto &block : blocks) {
    auto &space = memory.at(static_cast<std::size_t>(block.space));
    if (block.address > space.size() || block.words.size() > space.size() - block.address) {
      throw std::runtime_error("Fixture block outside memory span");
    }
    std::ranges::copy(block.words, space.begin() + block.address);
  }
}

inline FixtureMemory initial_memory(const BinaryCase &item) {
  FixtureMemory memory;
  for (std::size_t space = 0; space < memory.size(); ++space) {
    memory[space].resize(item.memory_words[space]);
  }
  patch(memory, item.before);
  return memory;
}
} // namespace dsp56300::test
