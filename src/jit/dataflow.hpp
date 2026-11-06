// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <cstdint>

namespace dsp56300::detail {
// Bit coordinates match the native A/B/X/Y value slots, not ISA register IDs.
enum class ValueBank : std::uint8_t { a, b, x, y };

constexpr Word bank_bit(const ValueBank bank) noexcept {
  return Word{1} << static_cast<unsigned>(bank);
}

struct NativeDataflow {
  Word old_inputs = 0, move_writes = 0;
};

// Unknown move classes conservatively clobber every bank. These effects describe
// parallel old-value dependencies, not flag liveness or cross-iteration freedom.
[[nodiscard]] NativeDataflow analyze_dataflow(const Instruction &instruction);
} // namespace dsp56300::detail
