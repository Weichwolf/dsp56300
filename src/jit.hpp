// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "translation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace dsp56300::detail {
struct BlockInstruction {
  Instruction instruction;
  Word pc = 0, mode = 0, loop_address = inactive_loop;
};

constexpr bool at_loop_end(const BlockInstruction &operation) noexcept {
  return operation.loop_address == ((operation.pc + operation.instruction.words - 1) & word_mask);
}

using JitHelper = std::uint32_t (*)(void *, const BlockInstruction *) noexcept;

struct JitMemory {
  Word *data = nullptr;
  std::size_t words = 0;
  bool aliases_program = false;
};

struct JitRuntime {
  void *owner = nullptr;
  std::array<std::uint64_t, 4> *latched = nullptr;
  JitHelper execute = nullptr, moves = nullptr;
  std::optional<Word> (*program)(void *, Word) noexcept = nullptr;
  std::array<JitMemory, 2> memory;
  std::array<std::span<const MemoryWindow>, 2> windows{};
  std::array<Word *, 2> writes{};
  // Conservative for standalone low-level contexts without a declared chip map.
  bool cacheable_program = true;
  bool admitted_memory = false;
};

struct NativeContext {
  JitRuntime runtime;
  std::uint32_t budget = 0;
  std::uint64_t cycle_deadline = std::numeric_limits<std::uint64_t>::max();
};

} // namespace dsp56300::detail
