// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
enum class WordReuseMode : std::uint8_t { automatic, vectors, disabled };

struct ProductOperands {
  std::array<unsigned, 2> words{};
  bool valid = false, square = false, scaled = false;
};

struct WordReuseStep {
  ProductOperands operands;
  Word writes = 0;
  bool barrier = true;
};

[[nodiscard]] ProductOperands product_operands(const Instruction &instruction);
[[nodiscard]] Word written_words(const Instruction &instruction);
[[nodiscard]] std::vector<Word> plan_word_reuse(std::span<const WordReuseStep> steps);
[[nodiscard]] std::vector<Word> plan_gpr_word_reuse(std::span<const WordReuseStep> steps);
} // namespace dsp56300::detail
