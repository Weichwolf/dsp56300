// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/llvm/runtime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300::detail {
[[nodiscard]] bool region_candidate(std::span<const BlockInstruction> operations,
                                    const JitRuntime &runtime,
                                    bool complete_loop = true);

struct PreparedRegion {
  const void *entry = nullptr;
  std::uint64_t native_bytes = 0;
};

// Complete arithmetic/RAM regions with non-call control; other effects retain ISA lowering.
[[nodiscard]] std::optional<PreparedRegion>
compile_region(std::unique_ptr<LlvmCompiler> &compiler,
               std::span<const BlockInstruction> operations,
               const JitRuntime &runtime,
               const void *fallback);
}
