// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "program_cache.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace dsp56300::detail {
enum class VectorPhase : std::uint8_t { none, pending, first, second };

struct VectorState {
  Word resume = 0, address = 0;
  unsigned level = 0;
  VectorPhase phase = VectorPhase::none;
};

// CPU continuation values only; external memory, I/O and host code stay separate.
struct ExecutionState {
  Cpu cpu;
  std::array<std::uint64_t, 4> latched{};
  Word current_pc = 0;
  unsigned current_words = 1;
  Instruction repeated;
  Fault fault;
  VectorState vector;
  Word debug_vector = 0;
  std::optional<ProgramCache> cache;
};
} // namespace dsp56300::detail
