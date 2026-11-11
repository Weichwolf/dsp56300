// SPDX-License-Identifier: GPL-3.0-only
#include "jit/prefixes.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
#include "jit/preparation.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr auto infinite = std::numeric_limits<std::uint64_t>::max();

struct Edge {
  Word from;
  unsigned cycles;
};

using Entry = std::pair<std::uint64_t, Word>;
using Queue = std::priority_queue<Entry, std::vector<Entry>, std::greater<>>;

std::uint64_t repeat_prefix(const std::map<Word, Instruction> &source,
                            const std::span<const RegionLayout> memory,
                            const Cpu &cpu) {
  const auto found = source.find(cpu.repeat_pc);
  if (cpu.reg.pc != cpu.repeat_pc || found == source.end()) { return 0; }
  const auto &instruction = found->second;
  if (instruction.words != 1 || !repeatable(instruction.operation) ||
      (cpu.repeat_loaded && instruction.opcode != cpu.repeat_opcode) ||
      prefix_boundary(instruction, memory)) {
    return 0;
  }
  return std::uint64_t{cpu.repeat_remaining} * instruction.cycles;
}
}

std::map<Word, std::uint64_t> foreground_prefixes(const ImagePlan &plan,
                                                  const std::span<const RegionLayout> memory) {
  const auto edges = plan.edges();
  std::map<Word, std::uint64_t> distances;
  std::map<Word, std::vector<Edge>> incoming;
  for (const auto &[pc, instruction] : plan.instructions) {
    static_cast<void>(instruction);
    distances.emplace(pc, infinite);
  }
  Queue pending;
  for (const auto &[pc, instruction] : plan.instructions) {
    const auto &targets = edges.at(pc);
    if (prefix_boundary(instruction, memory) || targets.empty() ||
        std::ranges::any_of(targets,
                            [&](const auto target) { return !distances.contains(target); })) {
      distances.at(pc) = 0;
      pending.emplace(0, pc);
      continue;
    }
    for (const auto target : targets) {
      incoming[target].push_back({.from = pc, .cycles = instruction.cycles});
    }
  }
  while (!pending.empty()) {
    const auto [cycles, pc] = pending.top();
    pending.pop();
    if (cycles != distances.at(pc)) { continue; }
    const auto found = incoming.find(pc);
    if (found == incoming.end()) { continue; }
    for (const auto edge : found->second) {
      if (cycles > infinite - edge.cycles) { continue; }
      const auto candidate = cycles + edge.cycles;
      auto &before = distances.at(edge.from);
      if (candidate < before) {
        before = candidate;
        pending.emplace(candidate, edge.from);
      }
    }
  }
  return distances;
}

std::uint64_t native_prefix(const std::map<Word, Instruction> &source,
                            const std::span<const RegionLayout> memory,
                            const std::map<Word, std::uint64_t> &prepared,
                            const Cpu &cpu) {
  if (cpu.processing != ExecutionState::running || cpu.debug_flags != 0) { return 0; }
  if (cpu.repeat_remaining != 0) { return repeat_prefix(source, memory, cpu); }
  if ((cpu.reg.sr & loop_flag) != 0) { return 0; }
  const auto found = prepared.find(cpu.reg.pc);
  return found == prepared.end() ? 0 : found->second;
}

std::uint64_t vector_entry_prefix(const ImagePlan &vector,
                                  const Registers &registers,
                                  const std::span<const RegionLayout> memory) {
  std::uint64_t elapsed = 0;
  for (const auto &[pc, instruction] : vector.instructions) {
    const auto flow = successors({.instruction = instruction, .pc = pc});
    if (flow.call) {
      return elapsed +
             (flow.indirect || registers.sc >= stack_entries - 1 ? 0U : instruction.cycles);
    }
    if (prefix_boundary(instruction, memory)) { return elapsed; }
    elapsed += instruction.cycles;
  }
  return elapsed;
}
}
