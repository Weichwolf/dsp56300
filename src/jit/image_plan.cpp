// SPDX-License-Identifier: GPL-3.0-only
#include "jit/image_plan.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/preparation.hpp"
#include "vector_instruction.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <llvm/ADT/StringExtras.h>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace dsp56300::detail {
namespace {
[[noreturn]] void reject(const Word address, const std::string_view message) {
  throw std::invalid_argument("DSP image P:$" +
                              llvm::utohexstr(address, true, address <= word_mask ? 6 : 0) + ": " +
                              std::string(message));
}

void validate_sources(const std::span<const CodeSpan> code) {
  std::map<Word, Word> words;
  for (const auto &span : code) {
    if (span.words.empty() || span.address >= address_words || span.words.size() > address_words) {
      reject(span.address, "empty or out-of-range executable stream");
    }
    for (std::size_t index = 0; index < span.words.size(); ++index) {
      const auto address = (span.address + static_cast<Word>(index)) & word_mask;
      const auto value = span.words[index];
      if (value > word_mask) { reject(address, "instruction source word exceeds 24 bits"); }
      const auto [found, added] = words.emplace(address, value);
      if (!added && found->second != value) {
        reject(address, "overlapping executable streams disagree on source words");
      }
    }
  }
}

std::optional<Word> encoded_target(const Word address, const Instruction &instruction) {
  constexpr std::array branches{Operation::jump_short,
                                Operation::jump_effective,
                                Operation::call_short,
                                Operation::call_effective,
                                Operation::conditional_jump,
                                Operation::relative_branch,
                                Operation::bit_jump,
                                Operation::bit_branch};
  if (std::ranges::find(branches, instruction.operation) == branches.end()) { return std::nullopt; }
  const BlockInstruction operation{.instruction = instruction, .pc = address};
  const auto flow = successors(operation);
  if (flow.indirect || flow.count == 0) { return std::nullopt; }
  return flow.direct[flow.count - 1];
}

void validate_failed_fetches(const CodeVariant &source,
                             const std::span<const Word> external_entries) {
  std::set<Word> targets;
  for (const auto address : source.failed_fetches) {
    if (address > word_mask || !targets.insert(address).second) {
      reject(address, "invalid or duplicate failed-fetch target");
    }
    if (std::ranges::find(external_entries, address) != external_entries.end()) {
      reject(address, "failed-fetch target is a code transition");
    }
    for (const auto &span : source.code) {
      const auto offset = (address + address_words - span.address) & word_mask;
      if (offset < span.words.size()) {
        reject(address, "failed-fetch target overlaps instruction source");
      }
    }
  }
}

void validate_targets(const ImagePlan &plan,
                      const std::span<const Word> external_entries,
                      const std::span<const Word> failed_fetches) {
  for (const auto entry : plan.entries) {
    if (plan.find(entry) == nullptr) { reject(entry, "entry is not a declared instruction start"); }
  }
  for (const auto &[address, instruction] : plan.instructions) {
    const auto target = encoded_target(address, instruction);
    if (target && plan.find(*target) == nullptr &&
        std::ranges::find(external_entries, *target) == external_entries.end() &&
        std::ranges::find(failed_fetches, *target) == failed_fetches.end()) {
      reject(address,
             "encoded target P:$" + llvm::utohexstr(*target, true, 6) +
                 " is not a declared instruction start");
    }
  }
}
}

const Instruction *ImagePlan::find(const Word address) const noexcept {
  const auto found = instructions.find(address);
  return found == instructions.end() ? nullptr : &found->second;
}

std::vector<Word> ImagePlan::units() const {
  if (instructions.empty()) { return {}; }
  std::set<Word> heads;
  heads.insert(instructions.begin()->first);
  std::optional<Word> linear;
  for (const auto &[address, instruction] : instructions) {
    if (!linear || address != *linear) { heads.insert(address); }
    const auto next = (address + instruction.words) & word_mask;
    linear = next;
    const auto flow = successors({.instruction = instruction, .pc = address});
    if (flow.count == 1 && flow.direct[0] == next && !flow.indirect && !flow.call) { continue; }
    if (find(next) != nullptr) { heads.insert(next); }
    for (const auto target : std::span(flow.direct).first(flow.count)) {
      if (find(target) != nullptr) { heads.insert(target); }
    }
  }
  return {heads.begin(), heads.end()};
}

std::map<Word, std::vector<Word>> ImagePlan::edges() const {
  constexpr std::array loops{Operation::loop_immediate,
                             Operation::loop_register,
                             Operation::loop_short,
                             Operation::loop_effective,
                             Operation::loop_forever};
  std::map<Word, std::set<Word>> endings;
  for (const auto &[pc, instruction] : instructions) {
    if (std::ranges::find(loops, instruction.operation) == loops.end()) { continue; }
    const auto flow = successors({.instruction = instruction, .pc = pc});
    endings[(flow.direct[1] - 1) & word_mask].insert(flow.direct[0]);
  }
  std::map<Word, std::vector<Word>> result;
  for (const auto &[pc, instruction] : instructions) {
    const auto flow = successors({.instruction = instruction, .pc = pc});
    std::set<Word> targets(flow.direct.begin(), flow.direct.begin() + flow.count);
    if (instruction.words == 1 && repeatable(instruction.operation)) { targets.insert(pc); }
    const auto end = endings.find((pc + instruction.words - 1) & word_mask);
    if (end != endings.end()) { targets.insert(end->second.begin(), end->second.end()); }
    result.emplace(pc, std::vector<Word>(targets.begin(), targets.end()));
  }
  return result;
}

ImagePlan plan_image(const CodeVariant &source, const std::span<const Word> external_entries) {
  const auto code = source.code;
  const auto entries = source.entries;
  if (code.empty() || entries.empty()) {
    throw std::invalid_argument("DSP image requires executable streams and established entries");
  }
  validate_sources(code);
  validate_failed_fetches(source, external_entries);
  ImagePlan result;
  for (const auto &span : code) {
    std::size_t index = 0;
    while (index < span.words.size()) {
      const auto pc = (span.address + static_cast<Word>(index)) & word_mask;
      auto instruction = decode(span.words[index]);
      if (instruction.operation == Operation::unsupported) {
        reject(pc, "unsupported instruction encoding");
      }
      if (instruction.words > span.words.size() - index) {
        reject(pc, "instruction extension is outside its declared stream");
      }
      if (instruction.words == 2) { instruction.extension = span.words[index + 1]; }
      result.instructions.emplace(pc, instruction);
      index += instruction.words;
    }
  }
  result.entries.assign(entries.begin(), entries.end());
  std::ranges::sort(result.entries);
  const auto duplicates = std::ranges::unique(result.entries);
  result.entries.erase(duplicates.begin(), duplicates.end());
  validate_targets(result, external_entries, source.failed_fetches);
  return result;
}

ImagePlan plan_image(const std::span<const CodeSpan> code, const std::span<const Word> entries) {
  return plan_image(CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}},
                    {});
}

ImagePlan plan_vector(const Word address,
                      const std::array<Word, 2> words,
                      const ImagePlan &foreground,
                      const std::span<const Word> external_entries) {
  if (address > word_mask || (address & 1U) != 0) {
    reject(address, "vector address must be even and fit 24 bits");
  }
  ImagePlan result;
  for (unsigned slot = 0; slot < words.size();) {
    const auto pc = address + slot;
    if (words[slot] > word_mask) { reject(pc, "vector word exceeds 24 bits"); }
    auto instruction = decode(words[slot]);
    const bool call = instruction.operation == Operation::call_short ||
                      instruction.operation == Operation::call_effective;
    if (!call && !permitted_vector_instruction(instruction)) {
      reject(pc, "operation is not permitted in a fast vector");
    }
    if (instruction.words == 2) {
      if (slot != 0) { reject(pc, "second vector slot requires a missing extension"); }
      if (words[1] > word_mask) { reject(pc + 1, "vector extension exceeds 24 bits"); }
      instruction.extension = words[1];
    }
    const auto target = encoded_target(pc, instruction);
    if (target && foreground.find(*target) == nullptr &&
        std::ranges::find(external_entries, *target) == external_entries.end()) {
      reject(pc, "vector call target is not a declared ordinary instruction");
    }
    result.instructions.emplace(pc, instruction);
    slot += instruction.words;
    // A call or undefined opcode replaces the vector before its next slot executes.
    if (call || instruction.operation == Operation::undefined) { break; }
  }
  result.entries.push_back(address);
  return result;
}
}
