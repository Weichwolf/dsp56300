// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/llvm/runtime.hpp"

#include <span>
#include <string_view>

namespace dsp56300::detail {
[[nodiscard]] bool region_independent(std::span<const BlockInstruction> operations);
[[nodiscard]] bool region_candidate(std::span<const BlockInstruction> operations,
                                    const JitRuntime &runtime,
                                    bool complete_loop = true);

// Linkable guarded region with dynamic owner storage and no ISA fallback or
// embedded instance addresses. A guard miss returns zero without CPU effects.
// An admitted image entry has already selected the instruction site and numeric mode.
[[nodiscard]] bool lift_region(LlvmModule &module,
                               std::span<const BlockInstruction> operations,
                               const JitRuntime &runtime);
[[nodiscard]] bool lift_region(LlvmModule &module,
                               std::span<const BlockInstruction> operations,
                               std::span<const RegionLayout> memory,
                               std::string_view entry = {},
                               Word *properties = nullptr,
                               std::uint64_t variant = 0,
                               bool direct = false,
                               bool linear_only = false,
                               std::vector<Word> *resume = nullptr,
                               bool admitted_site = false);

// Read-only uint64_t(const Cpu *, const NativeContext *) query: complete base
// periods or zero. Relevant state/storage changes invalidate the certificate.
[[nodiscard]] bool lift_region_admission(LlvmModule &module,
                                         std::span<const BlockInstruction> operations,
                                         std::span<const RegionLayout> memory,
                                         std::string_view entry = {});

}
