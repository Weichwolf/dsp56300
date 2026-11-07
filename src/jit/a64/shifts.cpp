// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::shift_count(const BlockInstruction &operation, const bool arithmetic) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto kind = instruction.operation;
  const bool immediate =
      kind == Operation::arithmetic_shift_immediate || kind == Operation::logical_shift_immediate;
  constexpr Word arithmetic_mask = 63;
  constexpr Word logical_mask = 31;
  const auto count_mask = arithmetic ? arithmetic_mask : logical_mask;
  if (kind == Operation::parallel) {
    code_.constant(Reg::x1, 1);
  } else if (immediate) {
    code_.constant(Reg::x1, (opcode >> 1) & count_mask);
  } else {
    const auto selector = (opcode >> 1) & 7U;
    if (selector < 2) {
      code_.constant(Reg::x1, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x1);
      return;
    }
    constexpr std::array banks{0U, 1U, 2U, 3U, 2U, 3U};
    constexpr std::array positions{main_shift, main_shift, 0U, 0U, word_bits, word_bits};
    code_.load(
        Reg::x1, Reg::sp, banks.at(selector - 2) * sizeof(std::uint64_t), Access::doubleword);
    code_.extract(Reg::x1, Reg::x1, positions.at(selector - 2), word_bits, Width::x64);
    mask(Reg::x1, Reg::x1, count_mask);
  }
  Label admitted;
  code_.constant(Reg::x17, arithmetic ? accumulator_bits - 1 : word_bits);
  code_.compare(Reg::x1, Reg::x17);
  code_.branch_far(Condition::below_equal, admitted);
  fault(operation, FaultKind::invalid_instruction, Reg::x1);
  code_.bind(admitted);
}

void Emitter::arithmetic_shift(const Word mode, const ShiftOperation operation) {
  Label unchanged;
  code_.constant(Reg::x3, 0);
  code_.constant(Reg::x4, 0);
  code_.branch_zero_far(Reg::x1, unchanged);
  if (operation.left) {
    code_.constant(Reg::x17, host_sign_bit + 1);
    code_.alu(Alu::subtract, Reg::x5, Reg::x17, Reg::x1);
    code_.shift(Shift::right, Reg::x4, Reg::x0, Reg::x5, Width::x64);
    mask(Reg::x4, Reg::x4, carry);
    code_.constant(Reg::x17, host_sign_bit);
    code_.alu(Alu::subtract, Reg::x5, Reg::x17, Reg::x1);
    code_.shift(Shift::arithmetic, Reg::x6, Reg::x0, Reg::x5, Width::x64);
    code_.compare(Reg::x6, Reg::zr, Width::x64);
    code_.set(Reg::x7, Condition::not_equal);
    code_.constant(Reg::x17, ~std::uint64_t{0});
    code_.compare(Reg::x6, Reg::x17, Width::x64);
    code_.set(Reg::x6, Condition::not_equal);
    code_.alu(Alu::bit_and, Reg::x3, Reg::x6, Reg::x7);
    code_.shift(Shift::left, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  } else {
    code_.add(Reg::x5, Reg::x1, padding_bits - 1);
    code_.shift(Shift::right, Reg::x4, Reg::x0, Reg::x5, Width::x64);
    mask(Reg::x4, Reg::x4, carry);
    code_.shift(Shift::arithmetic, Reg::x0, Reg::x0, Reg::x1, Width::x64);
    mask(Reg::x0, Reg::x0, padding_mask);
  }
  code_.bind(unchanged);
  if (operation.saturate) { saturate(mode, false); }
  arithmetic_flags(mode, true);
  code_.move(operation.target, Reg::x0);
}

void Emitter::logical_shift(const ShiftOperation operation, const bool rotate) {
  Label unchanged;
  code_.move(Reg::x2, Reg::x0, Width::w32);
  code_.constant(Reg::x4, 0);
  code_.branch_zero_far(Reg::x1, unchanged);
  if (operation.left) {
    code_.constant(Reg::x17, word_bits);
    code_.alu(Alu::subtract, Reg::x5, Reg::x17, Reg::x1);
    code_.shift(Shift::right, Reg::x4, Reg::x0, Reg::x5);
    code_.shift(Shift::left, Reg::x2, Reg::x0, Reg::x1);
    mask(Reg::x2, Reg::x2, word_mask, Width::w32);
  } else {
    code_.constant(Reg::x17, 1);
    code_.alu(Alu::subtract, Reg::x5, Reg::x1, Reg::x17);
    code_.shift(Shift::right, Reg::x4, Reg::x0, Reg::x5);
    code_.shift(Shift::right, Reg::x2, Reg::x0, Reg::x1);
  }
  mask(Reg::x4, Reg::x4, carry, Width::w32);
  if (rotate) {
    mask(Reg::x5, status, carry, Width::w32);
    if (!operation.left) { code_.shift(Shift::left, Reg::x5, Reg::x5, word_bits - 1); }
    code_.alu(Alu::bit_or, Reg::x2, Reg::x2, Reg::x5);
  }
  code_.bind(unchanged);
  code_.move(Reg::x0, Reg::x2, Width::w32);
  logical_flags(Reg::x0);
  mask(status, status, word_mask ^ carry, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x4);
  constexpr auto middle = std::uint64_t{word_mask} << main_shift;
  mask(operation.target, operation.target, ~middle);
  code_.shift(Shift::left, Reg::x0, Reg::x0, main_shift, Width::x64);
  code_.alu(Alu::bit_or, operation.target, operation.target, Reg::x0, Width::x64);
}

void Emitter::shift(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const auto opcode = operation.instruction.opcode;
  const auto code = opcode & alu_mask;
  const bool parallel = kind == Operation::parallel;
  const bool arithmetic = parallel ? code == alu_asr || code == alu_asl
                                   : kind == Operation::arithmetic_shift_immediate ||
                                         kind == Operation::arithmetic_shift_register;
  const bool immediate =
      kind == Operation::arithmetic_shift_immediate || kind == Operation::logical_shift_immediate;
  const bool destination_b = (opcode & (parallel ? 8U : 1U)) != 0;
  bool source_b = destination_b;
  bool left = (opcode & (1U << 5)) == 0;
  if (parallel) {
    left = (opcode & (1U << 4)) != 0;
  } else if (immediate) {
    left = arithmetic ? (opcode & (1U << 8)) != 0 : (opcode & (1U << 6)) == 0;
  }
  if (arithmetic && !parallel) { source_b = (opcode & (1U << (immediate ? 7 : 4))) != 0; }
  shift_count(operation, arithmetic);
  code_.load(Reg::x0, Reg::sp, source_b ? sizeof(std::uint64_t) : 0, Access::doubleword);
  const ShiftOperation control{
      .target = destination_b ? accumulator_b : accumulator_a, .left = left, .saturate = parallel};
  if (arithmetic) {
    arithmetic_shift(operation.mode, control);
  } else {
    code_.extract(Reg::x0, Reg::x0, main_shift, word_bits, Width::x64);
    logical_shift(control, parallel && (opcode & 4U) != 0);
  }
}
}
