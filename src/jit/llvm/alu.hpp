// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/llvm/builder.hpp"

#include <array>
#include <llvm-c/Types.h>

namespace dsp56300::detail::llvm19 {
using Banks = std::array<LLVMValueRef, 4>;

[[nodiscard]] constexpr bool conditional_alu(const Word opcode) {
  constexpr Word mask = 0xffe000;
  constexpr Word group = 0x202000;
  return (opcode & mask) == group;
}

[[nodiscard]] constexpr bool supported_product(const Operation kind) {
  return kind == Operation::product_mixed || kind == Operation::product_power;
}

[[nodiscard]] LLVMValueRef condition(Builder &ir, LLVMValueRef status, unsigned selector);
// Flags consumed by data or predicates; sticky retention is handled separately.
[[nodiscard]] Word flag_inputs(Word opcode);
[[nodiscard]] bool supported_alu(Word opcode);
void lower_alu(
    Builder &ir, Word opcode, Word mode, const Banks &before, Banks &after, LLVMValueRef &status);
void lower_arithmetic_shift(
    Builder &ir, Word opcode, Word mode, Banks &banks, LLVMValueRef &status);
void lower_logical_shift(Builder &ir, Word opcode, Banks &banks, LLVMValueRef &status);
void lower_product(
    Builder &ir, const Instruction &instruction, Word mode, Banks &banks, LLVMValueRef &status);
void lower_immediate_alu(
    Builder &ir, const Instruction &instruction, Word mode, Banks &banks, LLVMValueRef &status);
[[nodiscard]] LLVMValueRef
transfer_word(Builder &ir, LLVMValueRef value, Word mode, LLVMValueRef &status);
}
