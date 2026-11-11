// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <map>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct ImagePlan {
  // Sorted, unique instruction starts; descriptors own their complete source values.
  std::map<Word, Instruction> instructions;
  std::vector<Word> entries;
  [[nodiscard]] const Instruction *find(Word address) const noexcept;
};

// Decode every declared stream without running a Core or fetching other memory.
// Overlapping streams must agree on bytes; alternate starts require an explicit span.
// Calls do not make their saved return addresses into executable-code obligations.
[[nodiscard]] ImagePlan plan_image(std::span<const CodeSpan> code, std::span<const Word> entries);
}
