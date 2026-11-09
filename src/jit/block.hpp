// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
inline constexpr unsigned native_page_shift = 8;
inline constexpr std::size_t native_page_entries = 1U << native_page_shift;
using NativePage = std::array<const void *, native_page_entries>;

struct NativeIndex {
  const void *const *entries = nullptr;

  [[nodiscard]] const void *const *data() const noexcept { return entries; }
};

using NativeCatalog = NativeIndex;

[[nodiscard]] std::vector<BlockInstruction> parse_block(std::span<const Word> program,
                                                        const Registers &state,
                                                        const JitRuntime &runtime,
                                                        bool callback_boundaries = false);
[[nodiscard]] std::vector<std::uint8_t>
emit_block(std::span<const BlockInstruction> operations,
           const JitRuntime &runtime,
           const std::array<Word, agu_registers> &modifiers);
[[nodiscard]] std::vector<std::uint8_t>
emit_repeat(const BlockInstruction &operation,
            const JitRuntime &runtime,
            const std::array<Word, agu_registers> &modifiers);
#if defined(__aarch64__)
[[nodiscard]] std::vector<std::uint8_t> emit_vector(const BlockInstruction &operation);
#endif
[[nodiscard]] std::vector<std::uint8_t>
admit_cycles(std::span<const std::uint8_t> body, std::span<const BlockInstruction> operations = {});
[[nodiscard]] std::uint32_t
enter_block(const void *entry, Cpu &cpu, NativeContext &context) noexcept;
[[nodiscard]] std::uint32_t enter_blocks(const void *entry,
                                         Cpu &cpu,
                                         NativeContext &context,
                                         std::uint32_t budget,
                                         const NativeIndex &index,
                                         const NativeCatalog *catalog = nullptr) noexcept;
}
