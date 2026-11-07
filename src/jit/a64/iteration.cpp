// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::divide_iteration(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr std::array sources{0U, 2U, 1U, 3U};
  product_word(Reg::x1, sources.at((opcode >> 4) & 3U));
  const auto target = destination(opcode);
  code_.extract(Reg::x5, target, host_sign_bit, 1, Width::x64);
  code_.extract(Reg::x2, Reg::x1, word_bits - 1, 1, Width::x64);
  code_.alu(Alu::bit_xor, Reg::x5, Reg::x5, Reg::x2);
  code_.shift(Shift::left, Reg::x6, target, 1, Width::x64);
  code_.alu(Alu::bit_xor, Reg::x3, target, Reg::x6, Width::x64);
  code_.extract(Reg::x3, Reg::x3, host_sign_bit, 1, Width::x64);
  mask(Reg::x2, status, carry, Width::w32);
  code_.shift(Shift::left, Reg::x2, Reg::x2, padding_bits, Width::x64);
  code_.alu(Alu::bit_or, Reg::x6, Reg::x6, Reg::x2, Width::x64);
  code_.shift(Shift::left, Reg::x1, Reg::x1, main_shift, Width::x64);
  code_.alu(Alu::add, Reg::x0, Reg::x6, Reg::x1, Width::x64);
  code_.alu(Alu::subtract, Reg::x2, Reg::x6, Reg::x1, Width::x64);
  code_.compare(Reg::x5, Reg::zr);
  code_.select(Reg::x0, Reg::x0, Reg::x2, Condition::not_equal, Width::x64);
  code_.compare(Reg::x0, Reg::zr, Width::x64);
  code_.set(Reg::x4, Condition::positive);
  saturate(operation.mode, false);
  mask(status, status, word_mask ^ (carry | overflow), Width::w32);
  code_.constant(Reg::x17, overflow | limit);
  code_.multiply(Reg::x3, Reg::x3, Reg::x17);
  code_.alu(Alu::bit_or, status, status, Reg::x3);
  code_.alu(Alu::bit_or, status, status, Reg::x4);
  code_.move(target, Reg::x0);
}

void Emitter::normalize_step(const BlockInstruction &operation, const int count) {
  const auto opcode = operation.instruction.opcode;
  code_.move(Reg::x0, destination(opcode));
  code_.constant(Reg::x1, count == 0 ? 0U : 1U);
  arithmetic_shift(operation.mode,
                   {.target = destination(opcode), .left = count < 0, .saturate = true});
  if (count == 0) { return; }
  const auto offset = offsetof(Registers, r) + (((opcode >> 8) & 7U) * sizeof(Word));
  code_.load(Reg::x0, cpu, offset);
  code_.constant(Reg::x17, static_cast<Word>(count));
  code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x17);
  constexpr Word compatibility_mask = 0xffff;
  mask(Reg::x0,
       Reg::x0,
       (operation.mode & compatibility_mode) != 0 ? compatibility_mask : word_mask,
       Width::w32);
  code_.store(Reg::x0, cpu, offset);
}

void Emitter::normalize_iteration(const BlockInstruction &operation) {
  constexpr auto saved_status = 4 * sizeof(std::uint64_t);
  code_.store(status, Reg::sp, saved_status);
  Label right;
  Label unchanged;
  Label complete;
  mask(Reg::x0, status, extension, Width::w32);
  code_.branch_zero_far(Reg::x0, right, Width::w32, true);
  mask(Reg::x0, status, unnormalized | zero, Width::w32);
  code_.constant(Reg::x17, unnormalized);
  code_.compare(Reg::x0, Reg::x17);
  code_.branch_far(Condition::not_equal, unchanged);
  normalize_step(operation, -1);
  code_.branch(complete);
  code_.bind(right);
  normalize_step(operation, 1);
  code_.branch(complete);
  code_.bind(unchanged);
  normalize_step(operation, 0);
  code_.bind(complete);
  code_.load(Reg::x0, Reg::sp, saved_status);
  mask(Reg::x0, Reg::x0, carry, Width::w32);
  mask(status, status, word_mask ^ carry, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x0);
}

void Emitter::normalize_fast(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto selector = (opcode >> 1) & 7U;
  if (selector < 2) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  constexpr std::array sources{
      a1_register, b1_register, x0_register, y0_register, x1_register, y1_register};
  read_register(operation, sources.at(selector - 2));
  code_.extract(Reg::x1, Reg::x0, 0, word_bits, Width::x64, true);
  code_.add(Reg::x2, Reg::x1, accumulator_bits - 1, Width::x64);
  constexpr Word count_range = (2 * accumulator_bits) - 1;
  code_.constant(Reg::x17, count_range);
  code_.compare(Reg::x2, Reg::x17, Width::x64);
  Label admitted;
  code_.branch_far(Condition::below_equal, admitted);
  code_.constant(Reg::x0, opcode);
  fault(operation, FaultKind::unsupported_mode, Reg::x0);
  code_.bind(admitted);
  const auto target = (opcode & 1U) != 0 ? accumulator_b : accumulator_a;
  code_.move(Reg::x0, target);
  constexpr auto saved_status = 4 * sizeof(std::uint64_t);
  code_.store(status, Reg::sp, saved_status);
  code_.constant(Reg::x17, accumulator_bits);
  code_.compare(Reg::x1, Reg::x17, Width::x64);
  Label ordinary;
  Label right;
  Label complete;
  code_.branch_far(Condition::not_equal, ordinary);
  code_.shift(Shift::arithmetic, Reg::x0, Reg::x0, host_sign_bit, Width::x64);
  mask(Reg::x0, Reg::x0, padding_mask);
  code_.constant(Reg::x3, 0);
  arithmetic_flags(operation.mode, false);
  code_.move(target, Reg::x0);
  code_.branch(complete);
  code_.bind(ordinary);
  code_.compare(Reg::x1, Reg::zr, Width::x64);
  code_.branch_far(Condition::positive, right);
  code_.alu(Alu::subtract, Reg::x1, Reg::zr, Reg::x1, Width::x64);
  arithmetic_shift(operation.mode, {.target = target, .left = true});
  code_.branch(complete);
  code_.bind(right);
  arithmetic_shift(operation.mode, {.target = target});
  code_.bind(complete);
  code_.load(Reg::x0, Reg::sp, saved_status);
  mask(Reg::x0, Reg::x0, carry, Width::w32);
  mask(status, status, word_mask ^ carry, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x0);
}
}
