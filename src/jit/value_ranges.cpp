// SPDX-License-Identifier: GPL-3.0-only
#include "value_ranges.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "native_moves.hpp"

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
  unsigned width = accumulator_bits;
  bool raw_overflow = false, rounding = false, compare = false, transfer = false;
  bool zero_addend = false;
};

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

std::optional<Result> parallel_result(const NativeOp &operation,
                                      const std::array<unsigned, 2> old) {
  const auto &instruction = operation.instruction;
  const auto code = instruction.opcode;
  const auto form = code & alu_mask;
  const auto destination = (code & 8U) != 0 ? 1U : 0U;
  const auto source = (code >> 4) & 7U;
  const unsigned operand_width = source < 2 ? old[1 - destination] : double_word_bits;
  if ((code & product_bit) != 0) {
    const auto width =
        (code & 2U) != 0 ? std::max(product_width, old[destination] + 1) : product_width;
    return Result{.width = width,
                  .raw_overflow = width > accumulator_bits,
                  .rounding = (code & 1U) != 0,
                  .zero_addend = (code & 2U) != 0 && old[destination] == 0};
  }
  if (form == clear) { return Result{.width = 0}; }
  if (form == rounded) { return Result{.width = old[destination], .rounding = true}; }
  if ((form == absolute || form == negate)) {
    const auto width = old[destination] == 0 ? 0U : old[destination] + 1;
    return Result{.width = width, .raw_overflow = width > accumulator_bits};
  }
  if (form == asr) {
    return Result{.width = old[destination] == 0 ? 0U : std::max(1U, old[destination] - 1)};
  }
  if ((code & 7U) == 1 && (source == 0 || source >= 4)) {
    return Result{.width = operand_width, .transfer = true};
  }
  return std::nullopt;
}

std::optional<Result> arithmetic(const NativeOp &operation, const std::array<unsigned, 2> old) {
  const auto &instruction = operation.instruction;
  const auto code = instruction.opcode;
  const bool immediate = instruction.operation == Operation::immediate_alu;
  if (!immediate) {
    const auto result = parallel_result(operation, old);
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
  return Result{.width = width, .raw_overflow = width > accumulator_bits, .compare = compare};
}

NativeValueProof finish(const NativeOp &operation, const Result result) {
  NativeValueProof proof{.width = std::min(result.width, accumulator_bits),
                         .raw_overflow = result.raw_overflow,
                         .rounding_overflow = false,
                         .zero_addend = result.zero_addend,
                         .saturation = SaturationProof::none};
  if (result.rounding && proof.width != 0) {
    proof.rounding_overflow = proof.width == accumulator_bits;
    const auto bits = word_bits + ((operation.mode & scale_down) != 0 ? 1U : 0U) -
                      ((operation.mode & scale_up) != 0 ? 1U : 0U);
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
  const auto reg = plan_register_move(instruction);
  if (reg.valid()) {
    if (reg.destination >= a_register) {
      widths_.at(reg.destination - a_register) = double_word_bits;
    }
    return;
  }
  const auto plan = plan_moves(instruction);
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    if (move.load && move.reg >= a_register) {
      widths_.at(move.reg - a_register) = double_word_bits;
    }
  }
}

NativeValueProof
NativeValueRanges::advance(const NativeOp &operation, const bool native, const bool observer) {
  NativeValueProof unknown;
  if ((operation.mode & arithmetic_saturation) == 0) { unknown.saturation = SaturationProof::none; }
  if (!native) {
    reset();
    return unknown;
  }
  if (observer) { reset(); }
  const auto old = widths_;
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
  const auto result = arithmetic(operation, old);
  if (!result) {
    reset();
    return unknown;
  }
  const auto proof = finish(operation, *result);
  if (!result->compare) {
    widths_.at((operation.instruction.opcode & 8U) != 0 ? 1U : 0U) = proof.width;
  }
  if (observer) { reset(); }
  return proof;
}
} // namespace dsp56300::detail
