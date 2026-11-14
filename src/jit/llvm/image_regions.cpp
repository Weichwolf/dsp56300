// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/image_regions.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/region_plan.hpp"
#include "jit/preparation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <set>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
bool ordinary(const Operation kind) {
  return kind == Operation::nop || kind == Operation::parallel || kind == Operation::bit_memory ||
         kind == Operation::arithmetic_shift_immediate;
}

bool terminal(const Operation kind) {
  return kind == Operation::jump_short || kind == Operation::jump_effective ||
         kind == Operation::conditional_jump || kind == Operation::relative_branch;
}

auto body(const ImagePlan &plan, const Word first, const Word last) {
  std::vector<BlockInstruction> result;
  if (first > last) { return result; }
  auto pc = first;
  while (pc <= last) {
    const auto *instruction = plan.find(pc);
    if (instruction == nullptr || instruction->words - 1U > last - pc) {
      return std::vector<BlockInstruction>{};
    }
    if (!ordinary(instruction->operation)) { return std::vector<BlockInstruction>{}; }
    result.push_back({.instruction = *instruction, .pc = pc, .loop_address = last});
    pc += instruction->words;
  }
  return result;
}

void ordinary_regions(const ImagePlan &plan, std::vector<std::vector<BlockInstruction>> &result) {
  const auto units = plan.units();
  const std::set heads(units.begin(), units.end());
  const std::set entries(plan.entries.begin(), plan.entries.end());
  std::vector<BlockInstruction> current;
  const auto finish = [&] {
    if (current.size() > 1) {
      for (std::size_t index = 1; index + 1 < current.size(); ++index) {
        if (!entries.contains(current[index].pc)) { continue; }
        const auto suffix = std::span(current).subspan(index);
        result.emplace_back(suffix.begin(), suffix.end());
      }
      result.push_back(std::move(current));
    }
    current.clear();
  };
  for (const auto &[pc, instruction] : plan.instructions) {
    if (!current.empty() &&
        (heads.contains(pc) || pc != current.back().pc + current.back().instruction.words)) {
      finish();
    }
    if (!ordinary(instruction.operation)) {
      if (!current.empty() && terminal(instruction.operation)) {
        current.push_back({.instruction = instruction, .pc = pc});
      }
      finish();
    } else {
      current.push_back({.instruction = instruction, .pc = pc});
    }
  }
  finish();
}

void prefixes(std::span<const BlockInstruction> operations,
              const std::span<const RegionLayout> memory,
              std::vector<std::vector<BlockInstruction>> &result) {
  while (operations.size() > 1) {
    auto length = operations.size();
    while (length > 1 && !plan_region(operations.first(length), memory)) { --length; }
    if (length > 1) {
      const auto selected = operations.first(length);
      result.emplace_back(selected.begin(), selected.end());
    }
    operations = operations.subspan(length);
  }
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
  ordinary_regions(plan, result);
  return result;
}

std::vector<std::vector<BlockInstruction>>
image_regions(const ImagePlan &plan, const std::span<const RegionLayout> memory) {
  std::vector<std::vector<BlockInstruction>> result;
  std::set<Word> covered;
  for (auto operations : image_regions(plan)) {
    if (at_loop_end(operations.back())) {
      if (plan_region(operations, memory)) {
        for (const auto &operation : operations) { covered.insert(operation.pc); }
        result.push_back(std::move(operations));
      }
      continue;
    }
    std::size_t first = 0;
    for (std::size_t index = 0; index <= operations.size(); ++index) {
      if (index != operations.size() && !covered.contains(operations[index].pc)) { continue; }
      prefixes(std::span(operations).subspan(first, index - first), memory, result);
      first = index + 1;
    }
  }
  std::map<std::pair<Word, Word>, std::vector<BlockInstruction>> selected;
  for (auto &operations : result) {
    auto &longest = selected[{operations.front().pc, operations.back().loop_address}];
    if (longest.size() < operations.size()) { longest = std::move(operations); }
  }
  result.clear();
  for (auto &[key, operations] : selected) {
    static_cast<void>(key);
    result.push_back(std::move(operations));
  }
  return result;
}
}
