// SPDX-License-Identifier: GPL-3.0-only
#include "dataflow.hpp"

#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "native_moves.hpp"

#include <array>
#include <span>

namespace dsp56300::detail {
namespace {
constexpr Word all_banks = 15;
constexpr Word accumulator_b_bit = 8, multiply_bit = 0x80, accumulate_bit = 2;
constexpr Word clear_code = 0x13, round_code = 0x11, absolute_code = 0x26, negate_code = 0x36;
constexpr Word operation_mask = 0xf7;

Word register_bank(const unsigned reg) {
  if (reg == a_register) { return bank_bit(ValueBank::a); }
  if (reg == b_register) { return bank_bit(ValueBank::b); }
  return bank_bit(reg <= x1_register ? ValueBank::x : ValueBank::y);
}

Word input_bank(const unsigned selector, const bool destination_b) {
  if (selector < 2) { return bank_bit(destination_b ? ValueBank::a : ValueBank::b); }
  const bool x = selector == 2 || selector == 4 || selector == 6;
  return bank_bit(x ? ValueBank::x : ValueBank::y);
}

Word alu_inputs(const Word opcode) {
  const auto alu = opcode & byte_mask;
  const auto kind = alu & operation_mask;
  if (alu == 0 || kind == clear_code) { return 0; }
  const bool destination_b = (alu & accumulator_b_bit) != 0;
  const auto destination = bank_bit(destination_b ? ValueBank::b : ValueBank::a);
  if ((alu & multiply_bit) != 0) {
    constexpr Word x = bank_bit(ValueBank::x);
    constexpr Word y = bank_bit(ValueBank::y);
    constexpr std::array inputs{x, y, x, y, x | y, x | y, x | y, x | y};
    return inputs.at((alu >> 4) & 7) | ((alu & accumulate_bit) != 0 ? destination : 0);
  }
  if (kind == round_code || kind == absolute_code || kind == negate_code) { return destination; }
  const auto selector = (alu >> 4) & 7;
  // TFR has no old destination input; ADC forms with low bits 001 do.
  if ((alu & 7U) == 1 && (selector == 0 || selector >= 4)) {
    return input_bank(selector, destination_b);
  }
  const auto low = alu & 7U;
  const bool carry_form = (selector == 2 || selector == 3) && (low == 1 || low == 5);
  const bool sum = (selector >= 1 && (low == 0 || low == 4)) || carry_form ||
                   (low == 5 && (selector == 0 || selector >= 4));
  if (sum) { return destination | input_bank(selector, destination_b); }
  // Preserve every bank for unknown/future ALU forms until their effects are proven.
  return all_banks;
}
}

NativeDataflow analyze_dataflow(const Instruction &instruction) {
  NativeDataflow result;
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & upper_move_mask) == no_move_group) {
    return result;
  }
  const auto plan = plan_moves(instruction);
  if (plan.count == 0) {
    result.move_writes = all_banks;
  } else {
    for (const auto &move : std::span(plan.transfers.data(), plan.count)) {
      if (move.load) { result.move_writes |= register_bank(move.reg); }
    }
  }
  result.old_inputs = result.move_writes & alu_inputs(instruction.opcode);
  return result;
}
} // namespace dsp56300::detail
