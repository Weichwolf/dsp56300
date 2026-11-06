// SPDX-License-Identifier: GPL-3.0-only
#include "word_reuse.hpp"

#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "move.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr Word product_bit = 0x80, all_words = 15;
constexpr unsigned maximum_score = 4;

Word register_word(const unsigned reg) {
  return reg >= x0_register && reg <= y1_register ? Word{1} << (reg - x0_register) : 0;
}
}

ProductOperands product_operands(const Instruction &instruction) {
  if (instruction.operation != Operation::parallel || (instruction.opcode & product_bit) == 0) {
    return {};
  }
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  auto pair = pairs.at((instruction.opcode >> 4) & 7);
  if ((pair[0] & 1U) != 0 && (pair[1] & 1U) == 0) { std::swap(pair[0], pair[1]); }
  const bool square = pair[0] == pair[1];
  return {.words = pair, .valid = true, .square = square, .scaled = !square && (pair[0] & 1U) == 0};
}

Word written_words(const Instruction &instruction) {
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & upper_move_mask) == no_move_group || pure_address_update(instruction) ||
      plan_address_immediate(instruction).valid()) {
    return 0;
  }
  const auto reg = plan_register_move(instruction);
  if (reg.valid()) { return register_word(reg.destination); }
  const auto plan = plan_moves(instruction);
  if (plan.count == 0) { return all_words; }
  Word result = 0;
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    if (move.load) { result |= register_word(move.reg); }
  }
  return result;
}

std::vector<Word> plan_word_reuse(const std::span<const WordReuseStep> steps) {
  std::vector<Word> result(steps.size());
  std::array<unsigned, 4> savings{};
  for (auto index = steps.size(); index != 0; --index) {
    const auto &step = steps[index - 1];
    if (step.barrier) {
      savings.fill(0);
      continue;
    }
    for (unsigned word = 0; word < savings.size(); ++word) {
      if ((step.writes & (Word{1} << word)) != 0) { savings[word] = 0; }
    }
    if (!step.operands.valid) { continue; }
    const auto count = step.operands.square ? 1U : 2U;
    for (unsigned operand = 0; operand < count; ++operand) {
      const auto word = step.operands.words.at(operand);
      const bool scaled = operand == 0 && step.operands.scaled;
      const unsigned cost = scaled ? 2U : 1U;
      if (savings.at(word) > cost) { result[index - 1] |= Word{1} << word; }
      savings.at(word) = std::min(maximum_score, savings.at(word) + (scaled ? 1U : 2U));
    }
  }
  return result;
}

namespace {
Word select_gpr_word(const std::span<const WordReuseStep> steps) {
  std::array<std::size_t, 4> savings{};
  std::array<unsigned, 4> costs{};
  for (const auto &step : steps) {
    const auto &inputs = step.operands;
    if (!inputs.valid) { continue; }
    const auto count = inputs.square ? 1U : 2U;
    for (unsigned operand = 0; operand < count; ++operand) {
      const auto word = inputs.words.at(operand);
      const bool extra_shift =
          !inputs.square && inputs.scaled && (inputs.words.at(1 - operand) & 1U) != 0;
      if (costs.at(word) == 0) {
        costs.at(word) = extra_shift ? 2U : 1U;
      } else {
        savings.at(word) += inputs.square || extra_shift ? 2U : 3U;
      }
    }
  }
  std::size_t best = 0;
  Word result = 0;
  for (unsigned word = 0; word < savings.size(); ++word) {
    if (savings[word] > costs[word] && savings[word] - costs[word] > best) {
      best = savings[word] - costs[word];
      result = Word{1} << word;
    }
  }
  return result;
}
}

std::vector<Word> plan_gpr_word_reuse(const std::span<const WordReuseStep> steps) {
  std::vector<Word> result(steps.size());
  std::size_t begin = 0;
  while (begin < steps.size()) {
    if (steps[begin].barrier || steps[begin].writes != 0) {
      ++begin;
      continue;
    }
    auto end = begin + 1;
    while (end < steps.size() && !steps[end].barrier && steps[end].writes == 0) { ++end; }
    const auto chosen = select_gpr_word(steps.subspan(begin, end - begin));
    for (auto index = begin; index < end; ++index) { result[index] = chosen; }
    begin = end;
  }
  return result;
}
} // namespace dsp56300::detail
