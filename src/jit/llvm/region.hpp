// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/llvm/runtime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace dsp56300::detail {
// A complete non-call control region with no captured interior/final DO boundary.
[[nodiscard]] bool region_independent(std::span<const BlockInstruction> operations);
[[nodiscard]] bool region_candidate(std::span<const BlockInstruction> operations,
                                    const JitRuntime &runtime,
                                    bool complete_loop = true);

struct PreparedRegion {
  const void *entry = nullptr;
  std::uint64_t native_bytes = 0;
};

// Linkable guarded region with dynamic owner storage and no ISA fallback or
// embedded instance addresses. A guard miss returns zero without CPU effects.
[[nodiscard]] bool lift_region(LlvmModule &module,
                               std::span<const BlockInstruction> operations,
                               const JitRuntime &runtime);
[[nodiscard]] bool lift_region(LlvmModule &module,
                               std::span<const BlockInstruction> operations,
                               std::span<const RegionLayout> memory,
                               std::string_view entry = {});

// Read-only uint64_t(const Cpu *, const NativeContext *) query: complete base
// periods or zero. Relevant state/storage changes invalidate the certificate.
[[nodiscard]] bool lift_region_admission(LlvmModule &module,
                                         std::span<const BlockInstruction> operations,
                                         std::span<const RegionLayout> memory,
                                         std::string_view entry = {});

// Complete arithmetic/RAM regions with non-call control; other effects retain ISA lowering.
[[nodiscard]] std::optional<PreparedRegion>
compile_region(std::unique_ptr<LlvmCompiler> &compiler,
               std::span<const BlockInstruction> operations,
               const JitRuntime &runtime,
               const void *fallback);
}
