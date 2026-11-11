// SPDX-License-Identifier: GPL-3.0-only
#include "jit/image_plan.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/preparation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <llvm/ADT/StringExtras.h>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

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

void validate_targets(const ImagePlan &plan) {
  for (const auto entry : plan.entries) {
    if (plan.find(entry) == nullptr) { reject(entry, "entry is not a declared instruction start"); }
  }
  for (const auto &[address, instruction] : plan.instructions) {
    const auto target = encoded_target(address, instruction);
    if (target && plan.find(*target) == nullptr) {
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

ImagePlan plan_image(const std::span<const CodeSpan> code, const std::span<const Word> entries) {
  if (code.empty() || entries.empty()) {
    throw std::invalid_argument("DSP image requires executable streams and established entries");
  }
  validate_sources(code);
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
  validate_targets(result);
  return result;
}
}
