// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

namespace dsp56300::detail::a64 {
void Emitter::magnitude(const Reg value, const Reg temporary) {
  code_.alu(Alu::subtract, temporary, Reg::zr, value, Width::x64);
  code_.compare(value, Reg::zr, Width::x64);
  code_.select(value, temporary, value, Condition::negative, Width::x64);
}

void Emitter::maximum(const Word opcode) {
  read_old(Reg::x0, 0);
  read_old(Reg::x1, 1);
  if (opcode == alu_maximum_magnitude) {
    magnitude(Reg::x0, Reg::x9);
    magnitude(Reg::x1, Reg::x9);
  }
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  const auto selected = opcode == alu_maximum ? Condition::greater_equal : Condition::carry;
  const auto rejected = opcode == alu_maximum ? Condition::less : Condition::no_carry;
  read_old(Reg::x0, 0);
  code_.select(accumulator_b, Reg::x0, accumulator_b, selected, Width::x64);
  code_.set(Reg::x1, rejected);
  mask(status, status, word_mask ^ carry, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x1);
}

void Emitter::compare_magnitude(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const bool destination_b = (opcode & 8U) != 0;
  read_old(Reg::x0, destination_b ? 1U : 0U);
  source(Reg::x1, (opcode >> 4) & 7U, destination_b);
  magnitude(Reg::x0, Reg::x9);
  magnitude(Reg::x1, Reg::x9);
  code_.alu(Alu::subtract_flags, Reg::x2, Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x4, Condition::no_carry);
  code_.set(Reg::x3, Condition::above);
  code_.extract(Reg::x5, Reg::x2, host_sign_bit, 1, Width::x64);
  code_.alu(Alu::bit_and, Reg::x3, Reg::x3, Reg::x5);
  code_.move(Reg::x0, Reg::x2);
  complete(operation, true, true);
}

void Emitter::shifted_arithmetic(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const bool destination_b = (opcode & 8U) != 0;
  const bool left = (opcode & 16U) != 0;
  const bool subtract = (opcode & 4U) != 0;
  read_old(Reg::x0, destination_b ? 1U : 0U);
  read_old(Reg::x1, destination_b ? 0U : 1U);
  code_.constant(Reg::x3, 0);
  if (left) {
    code_.shift(Shift::left, Reg::x2, Reg::x0, 1, Width::x64);
    code_.alu(Alu::bit_xor, Reg::x3, Reg::x2, Reg::x0, Width::x64);
    code_.extract(Reg::x3, Reg::x3, host_sign_bit, 1, Width::x64);
    code_.move(Reg::x0, Reg::x2);
  } else {
    code_.shift(Shift::arithmetic, Reg::x0, Reg::x0, 1, Width::x64);
    mask(Reg::x0, Reg::x0, padding_mask);
  }
  code_.alu(subtract ? Alu::subtract_flags : Alu::add_flags, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x4, subtract ? Condition::no_carry : Condition::carry);
  code_.set(Reg::x5, Condition::overflow);
  code_.alu(Alu::bit_or, Reg::x3, Reg::x3, Reg::x5);
  complete(operation, false, true);
}
}
