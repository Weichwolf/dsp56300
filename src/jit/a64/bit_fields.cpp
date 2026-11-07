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
bool Emitter::field_control(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.words == 2) {
    code_.constant(Reg::x0, instruction.extension);
    return true;
  }
  const auto selector = (instruction.opcode >> 1) & 7U;
  if (selector < 2) {
    code_.constant(Reg::x0, instruction.opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return false;
  }
  constexpr std::array controls{
      a1_register, b1_register, x0_register, y0_register, x1_register, y1_register};
  read_register(operation, controls.at(selector - 2));
  return true;
}

void Emitter::field_bounds(const BlockInstruction &operation, const bool insert) {
  code_.extract(Reg::x1, Reg::x0, 0, 6);
  constexpr unsigned width_shift = 12;
  code_.extract(Reg::x2, Reg::x0, width_shift, 6);
  code_.alu(Alu::add, Reg::x3, Reg::x1, Reg::x2);
  code_.constant(Reg::x17, accumulator_bits);
  code_.compare(Reg::x3, Reg::x17);
  Label invalid;
  Label admitted;
  code_.branch_far(Condition::above, invalid);
  if (insert) {
    code_.constant(Reg::x17, word_bits);
    code_.compare(Reg::x2, Reg::x17);
    code_.branch_far(Condition::above, invalid);
  }
  code_.branch(admitted);
  code_.bind(invalid);
  if (insert) { code_.constant(Reg::x0, operation.instruction.opcode); }
  fault(operation, FaultKind::invalid_instruction, Reg::x0);
  code_.bind(admitted);
}

void Emitter::extract_field(const BlockInstruction &operation) {
  field_bounds(operation, false);
  const auto opcode = operation.instruction.opcode;
  const bool sign = (opcode & (1U << 7)) == 0;
  if (sign) {
    Label nonempty;
    code_.branch_zero_far(Reg::x2, nonempty, Width::w32, true);
    fault(operation, FaultKind::unsupported_mode, Reg::x0);
    code_.bind(nonempty);
  }
  const auto source = (opcode & 16U) != 0 ? accumulator_b : accumulator_a;
  code_.shift(Shift::right, Reg::x0, source, padding_bits, Width::x64);
  code_.shift(Shift::right, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.constant(Reg::x4, 1);
  code_.shift(Shift::left, Reg::x4, Reg::x4, Reg::x2, Width::x64);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x4, Reg::x4, Reg::x17, Width::x64);
  code_.alu(Alu::bit_and, Reg::x0, Reg::x0, Reg::x4, Width::x64);
  if (sign) {
    code_.constant(Reg::x17, host_sign_bit + 1);
    code_.alu(Alu::subtract, Reg::x5, Reg::x17, Reg::x2);
    code_.shift(Shift::left, Reg::x0, Reg::x0, Reg::x5, Width::x64);
    code_.shift(Shift::arithmetic, Reg::x0, Reg::x0, Reg::x5, Width::x64);
  }
  code_.shift(Shift::left, Reg::x0, Reg::x0, padding_bits, Width::x64);
  code_.move((opcode & 1U) != 0 ? accumulator_b : accumulator_a, Reg::x0);
  code_.constant(Reg::x3, 0);
  code_.constant(Reg::x4, 0);
  arithmetic_flags(operation.mode, true);
}

void Emitter::insert_field(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto selector = (opcode >> 4) & 7U;
  if (selector < 2) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  field_bounds(operation, true);
  constexpr std::array sources{
      a0_register, b0_register, x0_register, y0_register, x1_register, y1_register};
  read_register(operation, sources.at(selector - 2));
  code_.constant(Reg::x3, 1);
  code_.shift(Shift::left, Reg::x3, Reg::x3, Reg::x2, Width::x64);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x3, Reg::x3, Reg::x17, Width::x64);
  code_.alu(Alu::bit_and, Reg::x0, Reg::x0, Reg::x3, Width::x64);
  code_.shift(Shift::left, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.shift(Shift::left, Reg::x3, Reg::x3, Reg::x1, Width::x64);
  const auto target = (opcode & 1U) != 0 ? accumulator_b : accumulator_a;
  code_.shift(Shift::right, Reg::x4, target, padding_bits, Width::x64);
  code_.alu(Alu::bit_clear, Reg::x4, Reg::x4, Reg::x3, Width::x64);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x4, Width::x64);
  code_.shift(Shift::left, Reg::x0, Reg::x0, padding_bits, Width::x64);
  code_.move(target, Reg::x0);
  code_.constant(Reg::x3, 0);
  code_.constant(Reg::x4, 0);
  arithmetic_flags(operation.mode, true);
}

void Emitter::merge_field(const BlockInstruction &operation) {
  const auto target = (operation.instruction.opcode & 1U) != 0 ? accumulator_b : accumulator_a;
  constexpr unsigned half_bits = 12;
  constexpr Word half_mask = 0xfff;
  mask(Reg::x0, Reg::x0, half_mask, Width::w32);
  code_.shift(Shift::left, Reg::x0, Reg::x0, half_bits);
  code_.extract(Reg::x1, target, main_shift, half_bits, Width::x64);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x1);
  mask(target, target, ~(std::uint64_t{word_mask} << main_shift));
  code_.shift(Shift::left, Reg::x1, Reg::x0, main_shift, Width::x64);
  code_.alu(Alu::bit_or, target, target, Reg::x1, Width::x64);
  logical_flags(Reg::x0);
}

void Emitter::count_leading_bits(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto source = (opcode & 2U) != 0 ? accumulator_b : accumulator_a;
  code_.shift(Shift::arithmetic, Reg::x1, source, host_sign_bit, Width::x64);
  code_.alu(Alu::bit_xor, Reg::x1, Reg::x1, source, Width::x64);
  code_.count_leading_zeroes(Reg::x1, Reg::x1, Width::x64);
  constexpr unsigned exponent_bias = 9;
  code_.constant(Reg::x0, exponent_bias);
  code_.alu(Alu::subtract, Reg::x0, Reg::x0, Reg::x1);
  code_.compare(source, Reg::zr, Width::x64);
  code_.select(Reg::x0, Reg::zr, Reg::x0, Condition::equal);
  code_.shift(Shift::left,
              (opcode & 1U) != 0 ? accumulator_b : accumulator_a,
              Reg::x0,
              main_shift,
              Width::x64);
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  logical_flags(Reg::x0);
}

bool Emitter::bit_field(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const bool leading = kind == Operation::count_leading_bits;
  if (!leading && kind != Operation::bit_field_extract && kind != Operation::bit_field_insert &&
      kind != Operation::bit_field_merge) {
    return false;
  }
  if (!numeric_mode(operation, true)) { return true; }
  if (leading) {
    count_leading_bits(operation);
  } else if (field_control(operation)) {
    if (kind == Operation::bit_field_extract) {
      extract_field(operation);
    } else if (kind == Operation::bit_field_insert) {
      insert_field(operation);
    } else {
      merge_field(operation);
    }
  }
  return true;
}
}
