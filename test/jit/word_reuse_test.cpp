// SPDX-License-Identifier: GPL-3.0-only
// Independent forward generation and cost witnesses for backward reuse planning.
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "support.hpp"
#include "word_reuse.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr unsigned alphabet = 9, maximum_length = 5;
constexpr Word all_words = 15;

detail::WordReuseStep action(const unsigned number) {
  switch (number) {
    case 0: return {.operands = {.words = {0, 2}, .valid = true, .scaled = true}, .barrier = false};
    case 1: return {.operands = {.words = {3, 1}, .valid = true}, .barrier = false};
    case 2: return {.operands = {.words = {0, 0}, .valid = true, .square = true}, .barrier = false};
    case 3: return {.operands = {.words = {2, 2}, .valid = true, .square = true}, .barrier = false};
    case 4: return {.operands = {}, .writes = 1, .barrier = false};
    case 5: return {.operands = {}, .writes = 8, .barrier = false};
    case 6:
      return {.operands = {.words = {0, 2}, .valid = true, .scaled = true},
              .writes = 1,
              .barrier = false};
    case 7: return {.operands = {}, .writes = all_words, .barrier = true};
    case 8: return {.operands = {}, .barrier = false};
    default: throw std::runtime_error("Invalid declared reuse action");
  }
}

unsigned read_saving(const detail::WordReuseStep &step, const unsigned word) {
  if (!step.operands.valid) { return 0; }
  if (step.operands.square) { return step.operands.words[0] == word ? 2U : 0U; }
  if (step.operands.words[1] == word) { return 2; }
  if (step.operands.words[0] != word) { return 0; }
  return step.operands.scaled ? 1U : 2U;
}

unsigned future_saving(const std::span<const detail::WordReuseStep> suffix, const unsigned word) {
  unsigned result = 0;
  for (const auto &step : suffix) {
    if (step.barrier) { break; }
    result += read_saving(step, word);
    if ((step.writes & (Word{1} << word)) != 0) { break; }
  }
  return result;
}

struct CacheState {
  std::array<unsigned, 4> generation{};
  std::array<unsigned, 4> cached{};
  Word valid = 0;
};

void step_witness(const detail::WordReuseStep &step,
                  const Word population,
                  CacheState &state,
                  const std::span<const detail::WordReuseStep> suffix) {
  if (step.barrier) {
    require(population == 0, "No reuse population crosses a possible helper/observer");
    state.valid = 0;
  }
  for (unsigned word = 0; word < state.generation.size(); ++word) {
    const auto bit = Word{1} << word;
    const auto saving = step.barrier ? 0U : read_saving(step, word);
    const unsigned cost = saving == 1 ? 2U : 1U;
    const auto future = (step.writes & bit) == 0 ? future_saving(suffix, word) : 0;
    const bool populate = saving != 0 && future > cost;
    require(((population & bit) != 0) == populate,
            "Every population wins its independent forward instruction-cost comparison");
    if (saving != 0 && (state.valid & bit) != 0) {
      require(state.cached[word] == state.generation[word],
              "Every reused read names the exact old word before this instruction's writes");
    }
    if (saving != 0 && (state.valid & bit) == 0 && populate) {
      state.cached[word] = state.generation[word];
      state.valid |= bit;
    }
    if ((step.writes & bit) != 0) { ++state.generation[word]; }
  }
  state.valid &= ~step.writes;
  if (step.barrier) { state.valid = 0; }
}

void trace(const std::span<const detail::WordReuseStep> steps) {
  const auto plan = detail::plan_word_reuse(steps);
  require(plan.size() == steps.size(), "A reuse decision belongs to each original instruction");
  CacheState state;
  for (std::size_t index = 0; index < steps.size(); ++index) {
    step_witness(steps[index], plan[index], state, steps.subspan(index + 1));
  }
}

unsigned
virtual_read_cost(const detail::ProductOperands &inputs, const unsigned word, const bool cached) {
  constexpr unsigned square_cost = 4;
  constexpr unsigned first_square = 5;
  constexpr unsigned reused_square = 2;
  constexpr unsigned pair_cost = 6;
  constexpr unsigned operand_cost = 3;
  constexpr unsigned first_operand = 4;
  const bool selected = inputs.words[0] == word || inputs.words[1] == word;
  if (inputs.square) {
    if (!selected) { return square_cost; }
    return cached ? reused_square : first_square;
  }
  if (!selected) { return pair_cost + (inputs.scaled ? 0U : 1U); }
  const auto other = inputs.words[0] == word ? inputs.words[1] : inputs.words[0];
  return operand_cost + ((other & 1U) != 0 ? 1U : 0U) + (cached ? 0U : first_operand);
}

std::size_t virtual_cost(const std::span<const detail::WordReuseStep> steps, const unsigned word) {
  std::size_t result = 0;
  bool cached = false;
  for (const auto &step : steps) {
    const auto &inputs = step.operands;
    if (!inputs.valid) { continue; }
    result += virtual_read_cost(inputs, word, cached);
    cached = cached || inputs.words[0] == word || inputs.words[1] == word;
  }
  return result;
}

void gpr_trace(const std::span<const detail::WordReuseStep> steps) {
  const auto plan = detail::plan_gpr_word_reuse(steps);
  require(plan.size() == steps.size(), "GPR choices retain one decision per original instruction");
  std::size_t begin = 0;
  while (begin < steps.size()) {
    if (steps[begin].barrier || steps[begin].writes != 0) {
      require(plan[begin] == 0, "GPR word facts cannot cross possible scratch-register writes");
      ++begin;
      continue;
    }
    auto end = begin + 1;
    while (end < steps.size() && !steps[end].barrier && steps[end].writes == 0) { ++end; }
    const auto segment = steps.subspan(begin, end - begin);
    constexpr unsigned no_word = 4;
    auto best_cost = virtual_cost(segment, no_word);
    Word best = 0;
    for (unsigned word = 0; word < no_word; ++word) {
      const auto cost = virtual_cost(segment, word);
      if (cost < best_cost) {
        best_cost = cost;
        best = Word{1} << word;
      }
    }
    for (auto index = begin; index < end; ++index) {
      require(plan[index] == best,
              "Selected GPR word minimizes an independently enumerated instruction schedule");
    }
    begin = end;
  }
}

void traces() {
  unsigned cases = 1;
  for (unsigned length = 0; length <= maximum_length; ++length) {
    for (unsigned number = 0; number < cases; ++number) {
      std::vector<detail::WordReuseStep> steps;
      auto digits = number;
      for (unsigned index = 0; index < length; ++index) {
        steps.push_back(action(digits % alphabet));
        digits /= alphabet;
      }
      trace(steps);
      gpr_trace(steps);
    }
    cases *= alphabet;
  }
}

void word_writes() {
  constexpr std::array<unsigned, 6> registers{4, 5, 6, 7, 14, 15};
  constexpr unsigned source_shift = 13;
  constexpr unsigned destination_shift = 8;
  constexpr Word single_load = 0x40d800;
  constexpr Word single_store = 0x405800;
  constexpr unsigned high_selector_shift = 17;
  for (const auto source : registers) {
    for (const auto destination : registers) {
      const Word opcode =
          parallel_none | (source << source_shift) | (destination << destination_shift);
      const auto expected = destination < 8 ? Word{1} << (destination - 4) : 0;
      require(detail::written_words(detail::decode(opcode)) == expected,
              "Every independent full data-register move invalidates only its written word");
    }
    const auto selector = ((source & 7U) << 16) | ((source & 24U) << high_selector_shift);
    const auto expected = source < 8 ? Word{1} << (source - 4) : 0;
    for (const Word space : {0U, 0x080000U}) {
      require(detail::written_words(detail::decode(single_load | selector | space)) == expected,
              "Each X/Y memory load preserves the other register half");
      require(detail::written_words(detail::decode(single_store | selector | space)) == 0,
              "Stores do not overwrite source data-register words");
    }
  }
  constexpr Word unknown_move = 0x200100;
  require(detail::written_words(detail::decode(unknown_move)) == all_words,
          "An unknown move class cannot preserve optimistic word facts");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::word_writes();
    dsp56300::test::traces();
    std::cout << dsp56300::test::checks << " independent word-reuse planning checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
