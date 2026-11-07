// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
namespace {
bool addition(const Instruction &instruction) {
  const auto opcode = instruction.opcode;
  const auto kind = opcode & 7U;
  if (instruction.operation == Operation::immediate_alu) {
    return kind == 0 || kind == 4 || kind == 5;
  }
  const auto selector = (opcode >> 4) & 7U;
  const bool sum = selector >= 1 && (kind == 0 || kind == 4);
  const bool carry_sum = (selector == 2 || selector == 3) && (kind == 1 || kind == 5);
  const bool compare = kind == 5 && (selector == 0 || selector >= 4);
  return sum || carry_sum || compare;
}

bool logical(const Instruction &instruction) {
  const auto opcode = instruction.opcode;
  const auto kind = opcode & 7U;
  if ((opcode & alu_mask) == alu_complement) { return true; }
  return (kind == 2 || kind == 3 || kind == 6) &&
         (instruction.operation == Operation::immediate_alu || ((opcode >> 4) & 7U) >= 4);
}
}

void Emitter::source(const Reg value, const unsigned selector, const bool destination_b) {
  unsigned index = destination_b ? 0U : 1U;
  if (selector >= 2) { index = selector < 4 ? selector : (selector & 1U) + 2; }
  code_.load(value, Reg::sp, index * sizeof(std::uint64_t), Access::doubleword);
  if (selector >= 2 && selector < 4) {
    code_.extract(value, value, 0, double_word_bits, Width::x64, true);
    code_.shift(Shift::left, value, value, padding_bits, Width::x64);
  } else if (selector >= 4) {
    code_.extract(value, value, selector < 6 ? 0U : word_bits, word_bits, Width::x64, true);
    code_.shift(Shift::left, value, value, main_shift, Width::x64);
  }
}

void Emitter::arithmetic(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto kind = opcode & 7U;
  const auto selector = (opcode >> 4) & 7U;
  const bool immediate = instruction.operation == Operation::immediate_alu;
  const bool with_carry =
      !immediate && (selector == 2 || selector == 3) && (kind == 1 || kind == 5);
  const bool subtract = kind == 4 || kind == 5;
  const bool compare = kind == 5 && !with_carry;
  const bool destination_b = (opcode & 8U) != 0;
  code_.load(Reg::x0, Reg::sp, destination_b ? sizeof(std::uint64_t) : 0, Access::doubleword);
  if (immediate) {
    const auto value =
        instruction.words == 2 ? instruction.extension : (opcode >> 8) & six_bit_mask;
    code_.constant(Reg::x1, aligned_word(value));
  } else {
    source(Reg::x1, selector, destination_b);
  }
  const auto function = subtract ? Alu::subtract_flags : Alu::add_flags;
  const auto carried = subtract ? Condition::no_carry : Condition::carry;
  code_.alu(function, Reg::x2, Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x4, carried);
  if (with_carry) {
    code_.extract(Reg::x5, status, 0, 1, Width::x64);
    code_.shift(Shift::left, Reg::x5, Reg::x5, padding_bits, Width::x64);
    code_.alu(function, Reg::x2, Reg::x2, Reg::x5, Width::x64);
    code_.set(Reg::x5, carried);
    code_.alu(Alu::bit_or, Reg::x4, Reg::x4, Reg::x5);
    code_.alu(Alu::bit_xor, Reg::x5, subtract ? Reg::x0 : Reg::x2, Reg::x1, Width::x64);
    code_.alu(Alu::bit_xor, Reg::x6, Reg::x0, Reg::x2, Width::x64);
    code_.alu(Alu::bit_and, Reg::x3, Reg::x5, Reg::x6, Width::x64);
    code_.extract(Reg::x3, Reg::x3, host_sign_bit, 1, Width::x64);
  } else {
    code_.set(Reg::x3, Condition::overflow);
  }
  code_.move(Reg::x0, Reg::x2);
  complete(operation, compare, true);
}

void Emitter::product_word(const Reg value, const unsigned selector) {
  const auto bank = selector < 2 ? 2U : 3U;
  code_.load(value, Reg::sp, bank * sizeof(std::uint64_t), Access::doubleword);
  code_.extract(value, value, (selector & 1U) == 0 ? 0U : word_bits, word_bits, Width::x64, true);
}

void Emitter::multiply(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const auto pair = pairs.at((opcode >> 4) & 7U);
  product_word(Reg::x0, pair[0]);
  product_word(Reg::x1, pair[1]);
  code_.multiply(Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.shift(Shift::left, Reg::x0, Reg::x0, padding_bits + 1, Width::x64);
  if ((opcode & 4U) != 0) { code_.alu(Alu::subtract, Reg::x0, Reg::zr, Reg::x0, Width::x64); }
  if ((opcode & 2U) != 0) {
    code_.load(
        Reg::x1, Reg::sp, (opcode & 8U) != 0 ? sizeof(std::uint64_t) : 0, Access::doubleword);
    code_.alu(Alu::add_flags, Reg::x0, Reg::x0, Reg::x1, Width::x64);
    code_.set(Reg::x3, Condition::overflow);
  } else {
    code_.constant(Reg::x3, 0);
  }
  complete(operation, false, false, (opcode & 1U) != 0);
}

void Emitter::operation(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::nop) { return; }
  if ((operation.mode & (compatibility_mode | multiply_mode | arithmetic_16)) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down)) {
    throw std::invalid_argument("Unsupported DSP numeric mode requires its native fault exit");
  }
  const bool immediate = instruction.operation == Operation::immediate_alu;
  if (!immediate && (instruction.operation != Operation::parallel ||
                     (instruction.opcode & upper_move_mask) != no_move_group)) {
    throw std::invalid_argument("A64 DSP operation lowering is incomplete");
  }
  const auto opcode = instruction.opcode & byte_mask;
  if (!immediate && (opcode & 0x80U) != 0) {
    multiply(operation);
  } else if (logical(instruction)) {
    logic(operation);
  } else if (addition(instruction)) {
    arithmetic(operation);
  } else {
    unary(operation);
  }
}
}
