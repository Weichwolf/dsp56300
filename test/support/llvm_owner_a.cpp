// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/native.hpp"
#include "jit/llvm/runtime.hpp"
#include "llvm_owner.hpp"

#include <memory>
#include <mutex>

namespace dsp56300::test {
std::once_flag *llvm_flag_a() noexcept {
  return &weichwolf::llvm19::native_target_once;
}

std::unique_ptr<detail::LlvmCompiler> llvm_owner_a() {
  weichwolf::llvm19::initialize_native();
  return std::make_unique<detail::LlvmCompiler>();
}
}
