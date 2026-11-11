// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/runtime.hpp"

#include <cstdint>
#include <span>

namespace dsp56300::detail {
enum class ImageExit : std::uint8_t { owner, fault, budget, deadline, unprepared };
// Result: low 32 bits are retired instructions; high 32 bits contain ImageExit.
using ImageEntry = std::uint64_t (*)(void *, Word, std::uint32_t, std::uint64_t) noexcept;

// Link the exact semantic/binding bitcode first. Lift only explicitly declared code.
// Mutable instance state remains in the owner argument, never in exported objects.
void lift_image(LlvmModule &module, std::span<const CodeSpan> code, std::span<const Word> entries);
// Remove ordinary ISA calls before the shared native O3/export/install pipeline.
void specialize_image(LlvmModule &module);
}
