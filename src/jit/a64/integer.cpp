// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstdint>

namespace dsp56300::detail::a64 {
bool Emitter::integer(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const bool normalization = kind == Operation::normalize_iteration;
  const bool recognized = normalization || kind == Operation::normalize_fast ||
                          kind == Operation::divide_iteration || kind == Operation::increment ||
                          kind == Operation::decrement || kind == Operation::compare_unsigned;
  if (!recognized) { return false; }
  const auto ignored = arithmetic_saturation | (normalization ? compatibility_mode : 0U);
  if (!numeric_mode(operation, ignored)) { return true; }
  if (kind == Operation::increment || kind == Operation::decrement) {
    unit(operation);
  } else if (kind == Operation::compare_unsigned) {
    compare_unsigned(operation);
  } else if (kind == Operation::divide_iteration) {
    divide_iteration(operation);
  } else if (normalization) {
    normalize_iteration(operation);
  } else {
    normalize_fast(operation);
  }
  return true;
}

void Emitter::unit(const BlockInstruction &operation) {
  const auto target = (operation.instruction.opcode & 1U) != 0 ? accumulator_b : accumulator_a;
  const bool subtract = operation.instruction.operation == Operation::decrement;
  code_.move(Reg::x0, target);
  code_.constant(Reg::x1, std::uint64_t{1} << padding_bits);
  code_.alu(subtract ? Alu::subtract_flags : Alu::add_flags, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x3, Condition::overflow);
  code_.set(Reg::x4, subtract ? Condition::no_carry : Condition::carry);
  saturate(operation.mode, false);
  arithmetic_flags(operation.mode, true);
  code_.move(target, Reg::x0);
}

void Emitter::compare_unsigned(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto selector = (opcode >> 1) & 7U;
  if (selector >= 1 && selector <= 3) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  const bool destination_b = (opcode & 1U) != 0;
  code_.extract(Reg::x0,
                destination_b ? accumulator_b : accumulator_a,
                padding_bits,
                double_word_bits,
                Width::x64);
  if (selector == 0) {
    code_.extract(Reg::x1,
                  destination_b ? accumulator_a : accumulator_b,
                  padding_bits,
                  double_word_bits,
                  Width::x64);
  } else {
    code_.extract(Reg::x1,
                  (selector & 1U) == 0 ? bank_x : bank_y,
                  selector < 6 ? 0U : word_bits,
                  word_bits,
                  Width::x64);
    code_.shift(Shift::left, Reg::x1, Reg::x1, word_bits, Width::x64);
  }
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x2, Condition::equal);
  code_.set(Reg::x3, Condition::no_carry);
  code_.shift(Shift::left, Reg::x2, Reg::x2, 2);
  code_.constant(Reg::x17, carry | negative);
  code_.multiply(Reg::x3, Reg::x3, Reg::x17);
  mask(status, status, word_mask ^ (carry | negative | zero | overflow), Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x2);
  code_.alu(Alu::bit_or, status, status, Reg::x3);
}
}
