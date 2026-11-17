// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "jit/llvm/builder.hpp"

#include <array>
#include <llvm-c/Types.h>

namespace dsp56300::detail::llvm19 {
using Banks = std::array<LLVMValueRef, 4>;

struct DeferredMac {
  LLVMValueRef value = nullptr, overflowed = nullptr, limited = nullptr;
};

[[nodiscard]] constexpr bool conditional_alu(const Word opcode) {
  constexpr Word mask = 0xffe000;
  constexpr Word group = 0x202000;
  return (opcode & mask) == group;
}

[[nodiscard]] LLVMValueRef condition(Builder &ir, LLVMValueRef status, unsigned selector);
[[nodiscard]] bool supported_alu(Word opcode);
void lower_alu(Builder &ir,
               Word opcode,
               Word mode,
               const Banks &before,
               Banks &after,
               LLVMValueRef &status,
               DeferredMac *deferred = nullptr);
void finish_flags(Builder &ir, Word mode, const DeferredMac &flags, LLVMValueRef &status);
void lower_arithmetic_shift(
    Builder &ir, Word opcode, Word mode, Banks &banks, LLVMValueRef &status);
[[nodiscard]] LLVMValueRef
transfer_word(Builder &ir, LLVMValueRef value, Word mode, LLVMValueRef &status);
}
