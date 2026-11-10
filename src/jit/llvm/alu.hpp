// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "jit/llvm/builder.hpp"

#include <array>
#include <llvm-c/Types.h>

namespace dsp56300::detail::llvm19 {
using Banks = std::array<LLVMValueRef, 4>;

[[nodiscard]] bool supported_alu(Word opcode);
void lower_alu(
    Builder &ir, Word opcode, Word mode, const Banks &before, Banks &after, LLVMValueRef &status);
[[nodiscard]] LLVMValueRef
transfer_word(Builder &ir, LLVMValueRef value, Word mode, LLVMValueRef &status);
}
