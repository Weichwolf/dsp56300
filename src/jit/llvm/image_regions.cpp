// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/image_regions.hpp"

#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/image_plan.hpp"
#include "jit/preparation.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
auto body(const ImagePlan &plan, const Word first, const Word last) {
  std::vector<BlockInstruction> result;
  if (first > last) { return result; }
  auto pc = first;
  while (pc <= last) {
    const auto *instruction = plan.find(pc);
    if (instruction == nullptr || instruction->words - 1U > last - pc) {
      return std::vector<BlockInstruction>{};
    }
    const auto kind = instruction->operation;
    if (kind != Operation::nop && kind != Operation::parallel &&
        kind != Operation::arithmetic_shift_immediate) {
      return std::vector<BlockInstruction>{};
    }
    result.push_back({.instruction = *instruction, .pc = pc, .loop_address = last});
    pc += instruction->words;
  }
  return result;
}
}

std::vector<std::vector<BlockInstruction>> image_regions(const ImagePlan &plan) {
  constexpr std::array loops{Operation::loop_immediate,
                             Operation::loop_register,
                             Operation::loop_short,
                             Operation::loop_effective,
                             Operation::loop_forever};
  std::set<std::pair<Word, Word>> seen;
  std::vector<std::vector<BlockInstruction>> result;
  for (const auto &[pc, instruction] : plan.instructions) {
    if (std::ranges::find(loops, instruction.operation) == loops.end()) { continue; }
    const auto flow = successors({.instruction = instruction, .pc = pc});
    const auto first = flow.direct[0];
    const auto last = (flow.direct[1] - 1) & word_mask;
    if (!seen.emplace(first, last).second) { continue; }
    auto operations = body(plan, first, last);
    if (!operations.empty()) { result.push_back(std::move(operations)); }
  }
  return result;
}
}
