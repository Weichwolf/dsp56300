// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "hidden_retirement.hpp"
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

struct PlannedEffect {
  InstructionEffect effect;
  Registers computed;
};

struct StagedState {
  static constexpr std::size_t capacity = 64;
  static constexpr std::size_t plan_capacity = 8;
  ExecutionState base;
  ExecutionState candidate;
  std::size_t used = 0, cursor = 0;
  std::optional<InstructionEffect> pending;
  std::optional<InstructionEffect> following;
  Registers following_computed;
  std::array<PlannedEffect, plan_capacity> plan{};
  std::size_t planned = 0, plan_index = 0;
  Registers computed;
  AluWriteback alu_result;
  UnitWriteback unit_result;
  PcuWriteback pcu_result;
  HiddenRetirement hidden_result;
  bool pcu_source = false;
  bool control_transfer = false;
  bool unit_active = false;
  std::array<AcceptedEffect, capacity> records{};
  Instruction decoded;
  InstructionWords input;
  std::optional<Word> latched_opcode;
  InstructionPublication publication = InstructionPublication::retirement;
  bool active = false, computing = false, complete = false, fetch_words = false;
  bool planning = false, planning_disabled = false;

  [[nodiscard]] Word sample(InstructionEffect effect, const Registers &registers);
  [[nodiscard]] bool capacity_replay() const noexcept;
};
} // namespace dsp56300::detail
