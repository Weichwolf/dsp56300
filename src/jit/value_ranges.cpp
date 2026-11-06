// SPDX-License-Identifier: GPL-3.0-only
#include "value_ranges.hpp"

#include "architecture.hpp"
#include "dataflow.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "move.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::detail {
namespace {
constexpr Word alu_mask = 0xf7, product_bit = 0x80;
constexpr Word clear = 0x13, rounded = 0x11, absolute = 0x26, negate = 0x36, asr = 0x22;
constexpr unsigned product_width = 49, canonical_width = 50;

struct Result {
  // For a nonzero width n, the positive bound is 2^(n-1) - 2^headroom.
  unsigned width = accumulator_bits, zero_bits = 0, headroom = 0;
  bool raw_overflow = false, rounding = false, compare = false, transfer = false;
  bool zero_addend = false;
};

unsigned rounding_bits(const Word mode) {
  return word_bits + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
}

unsigned stored_zero_bits(const Word mode, const Result result, const NativeValueProof &proof) {
  if (proof.saturation != SaturationProof::none) { return 0; }
  return result.rounding ? std::max(result.zero_bits, rounding_bits(mode)) : result.zero_bits;
}

unsigned stored_headroom(const Word mode, const Result result, const NativeValueProof &proof) {
  if (result.raw_overflow || proof.saturation != SaturationProof::none) { return 0; }
  const auto bits = rounding_bits(mode);
  if (!result.rounding || result.zero_bits >= bits) { return result.headroom; }
  if (proof.width != result.width || result.width <= bits || result.headroom < bits) { return 0; }
  return std::max(bits, result.headroom - 1);
}

unsigned
sum_headroom(const Result left, const Result right, const bool subtract, const bool carry) {
  if (carry) { return 0; }
  if (right.width == 0) { return left.headroom; }
  if (left.width == 0 && !subtract) { return right.headroom; }
  if (left.width != right.width) { return 0; }
  return subtract ? left.headroom : std::min(left.headroom, right.headroom) + 1;
}

unsigned
sum_width(const unsigned left, const unsigned right, const bool subtract, const bool carry) {
  if (carry) { return std::max(2U, std::max(left, right) + 1); }
  if (right == 0) { return left; }
  if (left == 0) { return right + (subtract ? 1U : 0U); }
  return std::max(left, right) + 1;
}

unsigned constant_width(const Word word) {
  const auto value = static_cast<std::int64_t>(signed_word(word)) * (std::int64_t{1} << word_bits);
  if (value == 0) { return 0; }
  const auto magnitude = static_cast<std::uint64_t>(value < 0 ? -(value + 1) : value);
  return static_cast<unsigned>(std::bit_width(magnitude)) + 1;
}

Result product_result(const Word code, const Result old) {
  const bool accumulate = (code & 2U) != 0;
  const auto width = accumulate ? std::max(product_width, old.width + 1) : product_width;
  constexpr unsigned product_headroom = double_word_bits - 1;
  return {.width = width,
          .zero_bits = accumulate ? std::min(1U, old.zero_bits) : 1U,
          .headroom = !accumulate || old.width == 0 ? product_headroom : 0U,
          .raw_overflow = width > accumulator_bits,
          .rounding = (code & 1U) != 0,
          .zero_addend = accumulate && old.width == 0};
}

std::optional<Result> parallel_result(const BlockInstruction &operation,
                                      const std::array<unsigned, 2> old,
                                      const std::array<unsigned, 2> zeros,
                                      const std::array<unsigned, 2> headroom) {
  const auto &instruction = operation.instruction;
  const auto code = instruction.opcode;
  const auto form = code & alu_mask;
  const auto destination = (code & 8U) != 0 ? 1U : 0U;
  const auto source = (code >> 4) & 7U;
  const unsigned operand_width = source < 2 ? old[1 - destination] : double_word_bits;
  if ((code & product_bit) != 0) {
    return product_result(code, {.width = old[destination], .zero_bits = zeros[destination]});
  }
  if (form == clear) { return Result{.width = 0, .zero_bits = accumulator_bits}; }
  if (form == rounded) {
    return Result{.width = old[destination],
                  .zero_bits = zeros[destination],
                  .headroom = headroom[destination],
                  .rounding = true};
  }
  if ((form == absolute || form == negate)) {
    const auto width = old[destination] == 0 ? 0U : old[destination] + 1;
    return Result{
        .width = width, .zero_bits = zeros[destination], .raw_overflow = width > accumulator_bits};
  }
  if (form == asr) {
    return Result{.width = old[destination] == 0 ? 0U : std::max(1U, old[destination] - 1),
                  .zero_bits = zeros[destination] == 0 ? 0U : zeros[destination] - 1,
                  .headroom = headroom[destination] == 0 ? 0U : headroom[destination] - 1};
  }
  if ((code & 7U) == 1 && (source == 0 || source >= 4)) {
    return Result{.width = operand_width,
                  .zero_bits = source < 2 ? zeros[1 - destination] : word_bits,
                  .headroom = source < 2 ? headroom[1 - destination] : word_bits,
                  .transfer = true};
  }
  return std::nullopt;
}

std::optional<Result> arithmetic(const BlockInstruction &operation,
                                 const std::array<unsigned, 2> old,
                                 const std::array<unsigned, 2> zeros,
                                 const std::array<unsigned, 2> headroom) {
  const auto &instruction = operation.instruction;
  const auto code = instruction.opcode;
  const bool immediate = instruction.operation == Operation::immediate_alu;
  if (!immediate) {
    const auto result = parallel_result(operation, old, zeros, headroom);
    if (result) { return result; }
  }
  const auto destination = (code & 8U) != 0 ? 1U : 0U;
  const auto source = (code >> 4) & 7U;
  const auto kind = code & 7U;
  const unsigned operand_width = source < 2 ? old[1 - destination] : double_word_bits;
  const bool with_carry = !immediate && (source == 2 || source == 3) && (kind == 1 || kind == 5);
  const bool compare = kind == 5 && !with_carry;
  const bool sum = immediate ? kind == 0 || kind == 4 || compare
                             : (source >= 1 && (kind == 0 || kind == 4)) || with_carry ||
                                   (compare && (source == 0 || source >= 4));
  if (!sum) { return std::nullopt; }
  const auto constant = instruction.words == 2 ? instruction.extension : (code >> 8) & six_bit_mask;
  const auto right = immediate ? constant_width(constant) : operand_width;
  const auto width = sum_width(old[destination], right, kind == 4 || kind == 5, with_carry);
  unsigned operand_zeros = 0;
  unsigned operand_headroom = 0;
  if (immediate || source >= 4) {
    operand_zeros = word_bits;
    operand_headroom = word_bits;
  } else if (source < 2) {
    operand_zeros = zeros[1 - destination];
    operand_headroom = headroom[1 - destination];
  }
  return Result{.width = width,
                .zero_bits = with_carry ? 0U : std::min(zeros[destination], operand_zeros),
                .headroom =
                    sum_headroom({.width = old[destination], .headroom = headroom[destination]},
                                 {.width = right, .headroom = operand_headroom},
                                 kind == 4 || kind == 5,
                                 with_carry),
                .raw_overflow = width > accumulator_bits,
                .compare = compare};
}

NativeValueProof finish(const BlockInstruction &operation, const Result result) {
  NativeValueProof proof{.width = std::min(result.width, accumulator_bits),
                         .rounding_zero_bits = result.zero_bits,
                         .raw_overflow = result.raw_overflow,
                         .rounding_overflow = false,
                         .zero_addend = result.zero_addend,
                         .saturation = SaturationProof::none};
  const auto bits = rounding_bits(operation.mode);
  const bool may_grow = result.raw_overflow || result.headroom < bits || proof.width <= bits;
  if (result.rounding && proof.width != 0 && result.zero_bits < bits && may_grow) {
    proof.rounding_overflow = proof.width == accumulator_bits;
    proof.width = std::min(accumulator_bits, std::max(proof.width, bits + 1) + 1);
  }
  if ((operation.mode & arithmetic_saturation) != 0 && !result.transfer &&
      proof.width > double_word_bits) {
    proof.saturation =
        proof.width <= canonical_width ? SaturationProof::canonical : SaturationProof::general;
    if (proof.saturation == SaturationProof::canonical) { proof.width = double_word_bits; }
  }
  return proof;
}
} // namespace

void NativeValueRanges::moves(const Instruction &instruction) {
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & upper_move_mask) == no_move_group) {
    return;
  }
  const auto effects = analyze_dataflow(instruction);
  for (unsigned index = 0; index < zeros_.size(); ++index) {
    if ((effects.move_writes & (Word{1} << index)) != 0) {
      zeros_[index] = 0;
      headroom_[index] = 0;
    }
  }
  const auto reg = plan_register_move(instruction);
  if (reg.valid()) {
    if (reg.destination >= a_register) {
      widths_.at(reg.destination - a_register) = double_word_bits;
    }
    return;
  }
  const auto plan = plan_moves(instruction);
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    if (move.load && move.reg >= a0_register && move.reg <= b1_register) {
      widths_.at(move.reg & 1U) = accumulator_bits;
    } else if (move.load && move.reg >= a_register) {
      widths_.at(move.reg - a_register) = double_word_bits;
    }
  }
}

NativeValueProof NativeValueRanges::advance(const BlockInstruction &operation,
                                            const bool native,
                                            const bool observer) {
  NativeValueProof unknown;
  if ((operation.mode & arithmetic_saturation) == 0) { unknown.saturation = SaturationProof::none; }
  if (!native) {
    reset();
    return unknown;
  }
  if (observer) { reset(); }
  const auto old = widths_;
  const auto old_zeros = zeros_;
  const auto old_headroom = headroom_;
  unknown.transfer_widths = old;
  moves(operation.instruction);
  if (operation.instruction.operation == Operation::nop ||
      (operation.instruction.operation == Operation::parallel &&
       (operation.instruction.opcode & byte_mask) == 0)) {
    if (observer) { reset(); }
    return unknown;
  }
  if (operation.instruction.operation != Operation::parallel &&
      operation.instruction.operation != Operation::immediate_alu) {
    reset();
    return unknown;
  }
  const auto result = arithmetic(operation, old, old_zeros, old_headroom);
  if (!result) {
    reset();
    return unknown;
  }
  auto proof = finish(operation, *result);
  proof.transfer_widths = old;
  proof.input_zero_bits = old_zeros[(operation.instruction.opcode & 8U) != 0 ? 1U : 0U];
  if (!result->compare) {
    const auto index = (operation.instruction.opcode & 8U) != 0 ? 1U : 0U;
    widths_.at(index) = proof.width;
    zeros_.at(index) = stored_zero_bits(operation.mode, *result, proof);
    headroom_.at(index) = stored_headroom(operation.mode, *result, proof);
  }
  if (observer) { reset(); }
  return proof;
}
} // namespace dsp56300::detail
