// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <array>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct Successors {
  std::array<Word, 2> direct{};
  unsigned count = 0;
  // Register/stack/vector targets and implicit DO/REP continuation stay open.
  bool indirect = false;
};

struct PreparationGraph {
  std::vector<BlockInstruction> instructions;
  std::vector<Word> unresolved;
  Fault failure;
};

// Explicit encoding-derived edges, without sampling live register/data values.
[[nodiscard]] Successors successors(const BlockInstruction &operation);
[[nodiscard]] PreparationGraph preparation_graph(std::span<const Word> program,
                                                 const JitRuntime &runtime,
                                                 std::span<const Word> established_entries);
}
