// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
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
struct ImageContracts {
  struct Variant {
    std::uint64_t id = 0;
    std::map<Word, Word> words;
    std::map<Word, Instruction> instructions;
    std::map<Word, std::uint64_t> transitions;
    std::set<Word> failed_fetches;
    std::vector<RegionLayout> memory;
    bool fixed_memory = false;
    ImageEntry entry = nullptr;
    std::map<Word, std::uint64_t> prefixes;
  };

  [[nodiscard]] std::map<std::tuple<Word, Word, Word>, ImagePlan>
  plan_vectors(const std::vector<ImagePlan> &plans) const;
  std::vector<Variant> variants;
  std::set<std::pair<Word, unsigned>> interrupts;
  std::map<std::tuple<Word, Word, Word>, ImagePlan> vector_plans;
  PreparationReport report;
};
}

namespace dsp56300 {
struct SemanticImage::Impl : detail::ImageContracts {
  std::string identity;
  std::vector<std::byte> bitcode;
  std::vector<std::string> names, abis;
  std::vector<SemanticExport> exports;
  std::vector<std::string> variant_names;
  std::map<std::tuple<Word, Word, Word>, std::string> vector_symbols;
};

struct NativeImage::Impl : detail::ImageContracts {
  [[nodiscard]] const Variant &variant(std::uint64_t id) const;
  [[nodiscard]] detail::ImageEntry vector_entry(Word address, std::array<Word, 2> words) const;
  std::shared_ptr<const void> owner;
  std::map<std::tuple<Word, Word, Word>, detail::ImageEntry> vectors;
};
}
