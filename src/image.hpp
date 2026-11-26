// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/fixed_pass.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace dsp56300::detail {
struct ImageContracts {
  struct Variant {
    struct ProgramRun {
      Word first = 0;
      std::vector<Word> words;
    };

    std::uint64_t id = 0;
    std::vector<ProgramRun> program;
    void append_word(std::pair<Word, Word> word);
    [[nodiscard]] std::optional<Word> source_word(Word address) const noexcept;
    std::map<Word, Instruction> instructions;
    std::map<Word, std::vector<std::uint64_t>> transitions;
    std::set<Word> failed_fetches;
    std::vector<RegionLayout> memory;
    bool fixed_memory = false;
    ImageEntry entry = nullptr;
    std::map<Word, std::uint64_t> prefixes;
    std::map<Word, PassEffects> fixed_passes;
  };

  [[nodiscard]] std::map<std::tuple<Word, Word, Word>, ImagePlan>
  plan_vectors(const std::vector<ImagePlan> &plans) const;
  std::vector<Variant> variants;
  std::set<std::pair<Word, unsigned>> interrupts;
  std::map<std::tuple<Word, Word, Word>, ImagePlan> vector_plans;
  PreparationReport report;
  CompileOptions::MemoryMode memory_mode = CompileOptions::MemoryMode::exact;
};
}

namespace dsp56300 {
struct SemanticImage::Impl : detail::ImageContracts {
  void serialize(std::string_view input_identity);
  [[nodiscard]] static std::shared_ptr<Impl> deserialize(std::span<const std::byte> artifact,
                                                         std::string_view input_identity);
  std::string identity;
  std::vector<std::byte> bitcode, artifact;
  std::vector<std::string> names, abis;
  std::vector<SemanticExport> exports;
  std::vector<std::string> variant_names;
  std::map<std::tuple<Word, Word, Word>, std::string> vector_symbols;
};

struct NativeImage::Impl : detail::ImageContracts {
  void bind_sites(NativeSymbols::FaultMap map);
  [[nodiscard]] const Variant &variant(std::uint64_t id) const;
  [[nodiscard]] detail::ImageEntry vector_entry(Word address, std::array<Word, 2> words) const;
  std::shared_ptr<const void> owner;
  std::vector<NativeSymbols::TextRange> text_ranges;
  std::vector<NativeSymbols::Site> sites;
  std::map<std::tuple<Word, Word, Word>, detail::ImageEntry> vectors;
};
}
