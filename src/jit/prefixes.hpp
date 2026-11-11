// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"

#include <cstdint>
#include <map>
#include <span>

namespace dsp56300::detail {
struct Cpu;

[[nodiscard]] bool prefix_boundary(const Instruction &instruction,
                                   std::span<const RegionLayout> memory);
[[nodiscard]] std::map<Word, std::uint64_t>
foreground_prefixes(const ImagePlan &plan, std::span<const RegionLayout> memory);
// Pending vectors use their own captured fetch context.
[[nodiscard]] std::uint64_t native_prefix(const std::map<Word, Instruction> &source,
                                          std::span<const RegionLayout> memory,
                                          const std::map<Word, std::uint64_t> &prepared,
                                          const Cpu &cpu);
[[nodiscard]] std::uint64_t vector_entry_prefix(const ImagePlan &vector,
                                                const Registers &registers,
                                                std::span<const RegionLayout> memory);
}
