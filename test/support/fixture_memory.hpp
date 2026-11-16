// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::test {
using FixtureSource = std::pair<Word, std::vector<Word>>;
using FixtureCatalogue = std::map<FixtureSource, std::uint64_t>;
using FixtureMemory = std::array<std::vector<Word>, 3>;

inline FixtureSource executable_source(const BinaryCase &item) {
  const auto found = std::ranges::find_if(item.before, [&](const MemoryBlock &block) {
    return block.space == Space::program && block.address == item.initial.pc;
  });
  if (found == item.before.end()) {
    throw std::runtime_error("Fixture must declare its executable entry stream");
  }
  return {found->address, found->words};
}

inline FixtureCatalogue fixture_catalogue(const std::span<const BinaryCase> cases) {
  FixtureCatalogue catalogue;
  for (const auto &item : cases) {
    catalogue.try_emplace(executable_source(item), catalogue.size());
  }
  return catalogue;
}

inline NativeImage compile_fixture(const FixtureCatalogue &catalogue) {
  std::vector<CodeSpan> spans;
  spans.reserve(catalogue.size());
  std::vector<CodeVariant> variants;
  variants.reserve(catalogue.size());
  for (const auto &[code, id] : catalogue) {
    spans.push_back({.address = code.first, .words = code.second});
    variants.push_back({.id = id,
                        .code = std::span(&spans.back(), 1),
                        .entries = std::span(&code.first, 1),
                        .failed_fetches = {}});
  }
  return Core::compile(
      {.variants = variants},
      {.memory = {}, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

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
