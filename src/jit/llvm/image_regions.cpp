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
         kind == Operation::arithmetic_shift_immediate ||
         kind == Operation::logical_shift_immediate || kind == Operation::immediate_alu ||
         kind == Operation::move_control_register || kind == Operation::product_mixed;
}

bool terminal(const Operation kind) {
  return kind == Operation::jump_short || kind == Operation::jump_effective ||
         kind == Operation::conditional_jump || kind == Operation::relative_branch ||
         kind == Operation::call_short || kind == Operation::call_effective ||
         kind == Operation::return_subroutine || kind == Operation::bit_jump ||
         kind == Operation::bit_branch || loop_setup(kind);
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
    if (!ordinary(instruction->operation) && instruction->operation != Operation::loop_immediate) {
      return std::vector<BlockInstruction>{};
    }
    result.push_back({.instruction = *instruction, .pc = pc, .loop_address = last});
    pc += instruction->words;
  }
  return result;
}

void ordinary_regions(const ImagePlan &plan,
                      std::vector<std::vector<BlockInstruction>> &result,
                      const bool complete_children = false) {
  const auto units = plan.units();
  const std::set heads(units.begin(), units.end());
  std::set entries(plan.entries.begin(), plan.entries.end());
  if (complete_children) { entries.insert(heads.begin(), heads.end()); }
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
    if (!current.empty() && ((!complete_children && heads.contains(pc)) ||
                             pc != current.back().pc + current.back().instruction.words)) {
      finish();
    }
    const auto immediate_child =
        complete_children && instruction.operation == Operation::loop_immediate;
    if (!ordinary(instruction.operation) && !immediate_child) {
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
              std::vector<std::vector<BlockInstruction>> &result,
              const bool allow_partial_exit = false) {
  while (operations.size() > 1) {
    auto length = operations.size();
    while (length > 1 && !plan_region(operations.first(length), memory, allow_partial_exit)) {
      --length;
    }
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
    if (!operations.empty()) {
      if (instruction.operation == Operation::loop_immediate) {
        auto complete = operations;
        complete.insert(complete.begin(), {.instruction = instruction, .pc = pc});
        result.push_back(std::move(complete));
      }
      result.push_back(std::move(operations));
    }
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

std::vector<std::vector<BlockInstruction>>
image_composed_regions(const ImagePlan &plan, const std::span<const RegionLayout> memory) {
  std::vector<std::vector<BlockInstruction>> sequences;
  std::vector<std::vector<BlockInstruction>> candidates;
  // Keep the original callable entries and compile a region across complete DO bodies.
  ordinary_regions(plan, sequences, true);
  for (const auto &sequence : sequences) {
    if (std::ranges::none_of(sequence, [](const auto &step) {
          return step.instruction.operation == Operation::loop_immediate;
        })) {
      continue;
    }
    prefixes(sequence, memory, candidates, true);
  }

  std::map<Word, std::vector<BlockInstruction>> selected;
  for (auto &operations : candidates) {
    const auto planned = plan_region(operations, memory, true);
    if (!planned || planned->repeats.empty()) { continue; }
    auto &longest = selected[operations.front().pc];
    if (longest.size() < operations.size()) { longest = std::move(operations); }
  }
  std::vector<std::vector<BlockInstruction>> result;
  for (auto &[head, operations] : selected) {
    static_cast<void>(head);
    result.push_back(std::move(operations));
  }
  return result;
}
}
