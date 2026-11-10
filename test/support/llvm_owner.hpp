// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/llvm/runtime.hpp"

#include <memory>
#include <mutex>

namespace dsp56300::test {
[[nodiscard]] std::once_flag *llvm_flag_a() noexcept;
[[nodiscard]] std::once_flag *llvm_flag_b() noexcept;
[[nodiscard]] std::unique_ptr<detail::LlvmCompiler> llvm_owner_a();
[[nodiscard]] std::unique_ptr<detail::LlvmCompiler> llvm_owner_b();
}
