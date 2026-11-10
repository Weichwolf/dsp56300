// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <cstdint>

namespace dsp56300::detail::llvm19 {
class Builder {
public:
  Builder(LLVMContextRef context, LLVMBasicBlockRef block)
      : context_(context), builder_(LLVMCreateBuilderInContext(context)) {
    LLVMPositionBuilderAtEnd(builder_, block);
  }

  ~Builder() { LLVMDisposeBuilder(builder_); }

  Builder(const Builder &) = delete;
  Builder &operator=(const Builder &) = delete;

  [[nodiscard]] LLVMBuilderRef get() const { return builder_; }

  [[nodiscard]] LLVMTypeRef getInt8Ty() const { return LLVMInt8TypeInContext(context_); }

  [[nodiscard]] LLVMValueRef getInt8(std::uint64_t value) const {
    return LLVMConstInt(getInt8Ty(), value, 0);
  }

  [[nodiscard]] LLVMTypeRef getInt32Ty() const { return LLVMInt32TypeInContext(context_); }

  [[nodiscard]] LLVMValueRef getInt32(std::uint64_t value) const {
    return LLVMConstInt(getInt32Ty(), value, 0);
  }

  [[nodiscard]] LLVMTypeRef getInt64Ty() const { return LLVMInt64TypeInContext(context_); }

  [[nodiscard]] LLVMValueRef getInt64(std::uint64_t value) const {
    return LLVMConstInt(getInt64Ty(), value, 0);
  }

  LLVMValueRef CreateAnd(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildAnd(builder_, a, b, "");
  }

  LLVMValueRef CreateOr(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildOr(builder_, a, b, "");
  }

  LLVMValueRef CreateXor(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildXor(builder_, a, b, "");
  }

  LLVMValueRef CreateShl(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildShl(builder_, a, b, "");
  }

  LLVMValueRef CreateLShr(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildLShr(builder_, a, b, "");
  }

  LLVMValueRef CreateAdd(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildAdd(builder_, a, b, "");
  }

  LLVMValueRef CreateSub(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildSub(builder_, a, b, "");
  }

  LLVMValueRef CreateICmpEQ(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildICmp(builder_, LLVMIntEQ, a, b, "");
  }

  LLVMValueRef CreateICmpNE(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildICmp(builder_, LLVMIntNE, a, b, "");
  }

  LLVMValueRef CreateICmpULT(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildICmp(builder_, LLVMIntULT, a, b, "");
  }

  LLVMValueRef CreateICmpUGT(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildICmp(builder_, LLVMIntUGT, a, b, "");
  }

  [[nodiscard]] LLVMTypeRef getIntNTy(unsigned bits) const {
    return LLVMIntTypeInContext(context_, bits);
  }

  [[nodiscard]] LLVMTypeRef getPtrTy() const { return LLVMPointerTypeInContext(context_, 0); }

  LLVMValueRef CreateMul(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildMul(builder_, a, b, "");
  }

  LLVMValueRef CreateUDiv(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildUDiv(builder_, a, b, "");
  }

  LLVMValueRef CreateSExt(LLVMValueRef a, LLVMTypeRef t) const {
    return LLVMBuildSExt(builder_, a, t, "");
  }

  LLVMValueRef CreateICmpUGE(LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildICmp(builder_, LLVMIntUGE, a, b, "");
  }

  void SetInsertPoint(LLVMBasicBlockRef block) const { LLVMPositionBuilderAtEnd(builder_, block); }

  LLVMValueRef CreateNot(LLVMValueRef a) const { return LLVMBuildNot(builder_, a, ""); }

  LLVMValueRef CreateSelect(LLVMValueRef c, LLVMValueRef a, LLVMValueRef b) const {
    return LLVMBuildSelect(builder_, c, a, b, "");
  }

  LLVMValueRef CreateZExt(LLVMValueRef a, LLVMTypeRef t) const {
    return LLVMBuildZExt(builder_, a, t, "");
  }

  LLVMValueRef CreateTrunc(LLVMValueRef a, LLVMTypeRef t) const {
    return LLVMBuildTrunc(builder_, a, t, "");
  }

  LLVMValueRef CreateGEP(LLVMTypeRef t, LLVMValueRef p, LLVMValueRef i) const {
    return LLVMBuildGEP2(builder_, t, p, &i, 1, "");
  }

  LLVMValueRef CreateLoad(LLVMTypeRef t, LLVMValueRef p) const {
    return LLVMBuildLoad2(builder_, t, p, "");
  }

  void CreateStore(LLVMValueRef v, LLVMValueRef p) const { LLVMBuildStore(builder_, v, p); }

private:
  LLVMContextRef context_;
  LLVMBuilderRef builder_;
};

}
