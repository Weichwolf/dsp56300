// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/llvm/image.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace dsp56300::detail {
class LlvmCompiler;
}

namespace dsp56300 {
struct NativeImage::Impl {
  ~Impl();

  struct Variant {
    std::uint64_t id = 0;
    std::map<Word, Word> words;
    std::map<Word, detail::Instruction> instructions;
    std::vector<RegionLayout> memory;
    bool fixed_memory = false;
    detail::ImageEntry entry = nullptr;
  };

  [[nodiscard]] const Variant &variant(std::uint64_t id) const;
  [[nodiscard]] detail::ImageEntry vector_entry(Word address, std::array<Word, 2> words) const;
  void prepare_vectors(const Variant &variant,
                       const detail::ImagePlan &foreground,
                       const std::string &name);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  std::vector<Variant> variants;
  std::set<std::pair<Word, unsigned>> interrupts;
  std::map<std::tuple<Word, Word, Word>, detail::ImageEntry> vectors;
  PreparationReport report;
};
}
