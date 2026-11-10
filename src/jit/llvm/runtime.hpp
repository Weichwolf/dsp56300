// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <llvm-c/Orc.h>
#include <llvm-c/Types.h>
#include <memory>
#include <string>

namespace dsp56300::detail {
// Cold compiler ownership only; generated code uses the concrete native ABI.
class LlvmModule {
public:
  explicit LlvmModule(std::string name);
  ~LlvmModule();
  LlvmModule(const LlvmModule &) = delete;
  LlvmModule &operator=(const LlvmModule &) = delete;
  [[nodiscard]] LLVMContextRef context() const noexcept;

  [[nodiscard]] LLVMModuleRef code() const noexcept { return module_; }

  [[nodiscard]] const std::string &name() const noexcept { return name_; }

  [[nodiscard]] LLVMOrcThreadSafeModuleRef release() noexcept;

private:
  std::string name_;
  LLVMOrcThreadSafeContextRef context_ = nullptr;
  LLVMModuleRef module_ = nullptr;
};

class LlvmCompiler {
public:
  LlvmCompiler();
  ~LlvmCompiler();
  LlvmCompiler(const LlvmCompiler &) = delete;
  LlvmCompiler &operator=(const LlvmCompiler &) = delete;
  // Verify exact semantic IR, optimize with the native target, materialize now.
  // The single exported entry has the module's name. Code lives with this owner.
  [[nodiscard]] const void *install(LlvmModule &module);
  // Cold materialization accounting; native execution does not update it.
  [[nodiscard]] std::uint64_t native_bytes() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
