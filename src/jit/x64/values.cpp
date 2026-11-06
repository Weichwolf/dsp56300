// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "native_moves.hpp"
#include "word_reuse.hpp"

#include <array>
#include <bit>
#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr std::array slots{x64::Xmm::x8, x64::Xmm::x9, x64::Xmm::x10, x64::Xmm::x11};
constexpr unsigned shift_left = 4, shift_arithmetic = 7;
constexpr unsigned low_word_shift = 40, high_word_shift = 16, product_shift = 9;
constexpr unsigned vector_valid_shift = 4;
}

void BlockEmitter::invalidate_word_reuse(const WordReuseStep &step) {
  word_valid_ &= ~(step.writes | (step.writes << vector_valid_shift));
  if (step.barrier) { word_valid_ = 0; }
}

std::vector<WordReuseStep>
BlockEmitter::word_effects(const std::span<const BlockInstruction> operations) const {
  std::vector<WordReuseStep> result;
  result.reserve(operations.size());
  for (const auto &operation : operations) {
    result.push_back({.operands = product_operands(operation.instruction),
                      .writes = written_words(operation.instruction),
                      .barrier = word_mode_ == WordReuseMode::disabled || !lowerable(operation) ||
                                 observable_moves(operation) ||
                                 pure_address_update(operation.instruction) ||
                                 is_loop_setup(operation)});
  }
  return result;
}

void BlockEmitter::product_word(const x64::Reg destination,
                                const BlockInstruction &operation,
                                const unsigned selector,
                                const bool scaled) {
  const auto bit = Word{1} << selector;
  const auto slot = slots.at(selector);
  if ((word_valid_ & (bit << vector_valid_shift)) != 0) {
    code_.packed_to(destination, slot);
    if (scaled) { code_.shift(shift_left, destination, product_shift); }
    return;
  }
  const bool populate = (word_populate_ & bit) != 0;
  value(destination, selector < 2 ? 2U : 3U, has_moves(operation));
  code_.shift(shift_left, destination, (selector & 1U) == 0 ? low_word_shift : high_word_shift);
  code_.shift(
      shift_arithmetic, destination, low_word_shift - (scaled && !populate ? product_shift : 0U));
  if (populate) {
    code_.packed_from<64>(slot, destination);
    word_valid_ |= bit << vector_valid_shift;
    reused_words_ = true;
    if (scaled) { code_.shift(shift_left, destination, product_shift); }
  }
}

bool BlockEmitter::gpr_product(const BlockInstruction &operation) {
  if (word_selected_ == 0) { return false; }
  const auto inputs = product_operands(operation.instruction);
  const auto word = static_cast<unsigned>(std::countr_zero(word_selected_));
  if (inputs.words[0] != word && inputs.words[1] != word) { return false; }
  using x64::Reg;
  const bool valid = (word_valid_ & word_selected_) != 0;
  if (inputs.square) {
    if (valid) {
      code_.mov({.destination = Reg::ax, .source = Reg::cx});
    } else {
      product_word(Reg::ax, operation, word, false);
      code_.mov({.destination = Reg::cx, .source = Reg::ax});
    }
  } else {
    const auto other = inputs.words[0] == word ? inputs.words[1] : inputs.words[0];
    const bool scaled = (other & 1U) == 0;
    product_word(Reg::ax, operation, other, scaled);
    if (!valid) {
      product_word(Reg::dx, operation, word, false);
      code_.mov({.destination = Reg::cx, .source = Reg::dx});
    }
    code_.multiply({.destination = Reg::ax, .source = Reg::cx});
    if (!scaled) { code_.shift(shift_left, Reg::ax, product_shift); }
  }
  if (inputs.square) {
    code_.multiply({.destination = Reg::ax, .source = Reg::cx});
    code_.shift(shift_left, Reg::ax, product_shift);
  }
  word_valid_ |= word_selected_;
  reused_words_ = true;
  return true;
}
} // namespace dsp56300::detail
