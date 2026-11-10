// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <cstdint>

namespace dsp56300::detail {
enum class SemanticFlow : std::uint8_t { fault, owner, sequential, branch };

// Compiler-facing values, with architectural state retained by Core.
struct SemanticResult {
  Word pc;
  SemanticFlow flow;
};
} // namespace dsp56300::detail
