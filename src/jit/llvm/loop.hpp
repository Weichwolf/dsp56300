// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/llvm/runtime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300::detail {
[[nodiscard]] bool memory_loop_candidate(std::span<const BlockInstruction> operations,
                                         const JitRuntime &runtime);

struct PreparedLoop {
  const void *entry = nullptr;
  std::uint64_t native_bytes = 0;
};

// Ordinary RAM transfers in natural DO bodies; other effects retain native ISA lowering.
[[nodiscard]] std::optional<PreparedLoop>
compile_memory_loop(std::unique_ptr<LlvmCompiler> &compiler,
                    std::span<const BlockInstruction> operations,
                    const JitRuntime &runtime,
                    const void *fallback);
}
