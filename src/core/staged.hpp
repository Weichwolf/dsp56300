// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "instruction.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace dsp56300::detail {
struct InstructionPause {};

struct AcceptedEffect {
  InstructionEffect effect;
  Word response = 0;
};

struct StagedState {
  static constexpr std::size_t capacity = 64;
  ExecutionState base;
  ExecutionState candidate;
  std::size_t used = 0, cursor = 0;
  std::optional<InstructionEffect> pending;
  Registers computed;
  std::array<AcceptedEffect, capacity> records{};
  Instruction decoded;
  InstructionWords input;
  InstructionPublication publication = InstructionPublication::retirement;
  bool active = false, computing = false, complete = false;

  [[nodiscard]] Word sample(InstructionEffect effect);
};
} // namespace dsp56300::detail
