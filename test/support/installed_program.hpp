// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
// Compile the authored instruction stream against the actual borrowed P/X/Y map.
inline NativeImage
compile_program(const std::span<const Word> words, const Word entry, const Buffers memory) {
  const std::array spans{CodeSpan{.address = entry, .words = words}};
  const std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{.space = Space::program, .first = 0, .size = memory.program.size()},
      RegionLayout{.space = Space::x, .first = 0, .size = memory.x.size()},
      RegionLayout{.space = Space::y, .first = 0, .size = memory.y.size()}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  return Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

// Every declared stream is a real program variant with its own entry address.
inline NativeImage compile_programs(const std::span<const CodeSpan> programs,
                                    const std::span<const Word> entries = {},
                                    const std::span<const CodeTransition> transitions = {}) {
  if (!entries.empty() && entries.size() != programs.size()) {
    throw std::invalid_argument("Every authored program requires one declared entry");
  }
  std::vector<CodeVariant> variants;
  variants.reserve(programs.size());
  for (std::size_t index = 0; index < programs.size(); ++index) {
    variants.push_back({.id = index,
                        .code = programs.subspan(index, 1),
                        .entries = entries.empty() ? std::span(&programs[index].address, 1)
                                                   : entries.subspan(index, 1),
                        .failed_fetches = {}});
  }
  return Core::compile({.variants = variants},
                       {.memory = {},
                        .interrupts = {},
                        .transitions = transitions,
                        .callbacks = {},
                        .bindings = {}});
}

inline NativeImage compile_operations(const std::span<const Word> operations, const Word entry) {
  std::vector<CodeSpan> programs;
  programs.reserve(operations.size());
  for (std::size_t index = 0; index < operations.size(); ++index) {
    programs.push_back({.address = entry, .words = operations.subspan(index, 1)});
  }
  return compile_programs(programs);
}

}
