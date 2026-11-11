// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
enum class SemanticFlow : std::uint8_t { fault, owner, sequential, branch };

// Compiler-facing values, with architectural state retained by Core.
struct SemanticResult {
  Word pc;
  SemanticFlow flow;
};

inline constexpr std::size_t semantic_abi_fields = 9;
extern "C" const std::array<Word, semantic_abi_fields> dsp56300_semantic_abi;
} // namespace dsp56300::detail
