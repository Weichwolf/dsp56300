// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region_plan.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/move.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
bool permitted(const MemoryWindow &window, const bool writing) {
  return !window.words.empty() && window.address < io_begin &&
         window.words.size() <= io_begin - window.address &&
         (window.known.empty() || window.known.size() == window.words.size()) &&
         (!writing || (window.writable && !window.aliases_program));
}

namespace {
auto declared_windows(const JitRuntime &runtime) {
  std::array<std::vector<MemoryWindow>, 2> result;
  for (std::size_t bank = 0; bank < result.size(); ++bank) {
    if (!runtime.windows[bank].empty()) {
      result[bank].assign(runtime.windows[bank].begin(), runtime.windows[bank].end());
      continue;
    }
    const auto &memory = runtime.memory[bank];
    if (memory.data != nullptr && memory.words != 0) {
      result[bank].push_back({.words = {memory.data, std::min(memory.words, std::size_t{io_begin})},
                              .known = {},
                              .writable = true,
                              .aliases_program = memory.aliases_program});
    }
  }
  return result;
}

bool stable_knowledge(const RegionPlan &plan) {
  std::vector<std::span<const std::byte>> masks;
  for (const auto &bank : plan.windows) {
    for (const auto &window : bank) {
      if (!window.known.empty()) { masks.push_back(std::as_bytes(window.known)); }
    }
  }
  for (std::size_t bank = 0; bank < plan.windows.size(); ++bank) {
    if (!std::ranges::any_of(plan.extents[bank],
                             [](const auto &extent) { return extent.writing; })) {
      continue;
    }
    for (const auto &window : plan.windows[bank]) {
      if (!permitted(window, true)) { continue; }
      const auto data = std::bit_cast<std::uintptr_t>(window.words.data());
      for (const auto mask : masks) {
        const auto known = std::bit_cast<std::uintptr_t>(mask.data());
        if (data <= known ? known - data < window.words.size_bytes() : data - known < mask.size()) {
          return false;
        }
      }
    }
  }
  return true;
}

bool plain_register(const unsigned reg) {
  return (reg >= x0_register && reg <= y1_register) || reg == a_register || reg == b_register ||
         (reg >= r_register_base && reg < m_register_base);
}

bool destination(RegionPlan &plan, const unsigned reg) {
  if (!plain_register(reg)) { return false; }
  if (reg >= n_register_base) {
    plan.changed_n[reg - n_register_base] = true;
  } else if (reg >= r_register_base) {
    plan.changed_r[reg - r_register_base] = true;
  }
  if (reg >= r_register_base) { plan.batch = false; }
  return true;
}

bool append_move(RegionPlan &plan, const NativeMove &move, const bool advance) {
  const auto form = move.address;
  const auto mode = form.mode;
  plan.agu[form.index] = plan.agu[form.index] || mode != 4;
  if (mode == 6 || plan.changed_r[form.index] ||
      ((mode == 0 || mode == 1 || mode == 5) && plan.changed_n[form.index])) {
    return false;
  }
  if (mode == 0 || mode == 1 || mode == 5) { plan.batch = false; }
  const auto bank = move.space == Space::x ? 0U : 1U;
  if (!std::ranges::any_of(plan.windows[bank],
                           [&move](const auto &window) { return permitted(window, !move.load); })) {
    return false;
  }
  auto &delta = plan.deltas[form.index];
  if (mode == 7 && advance) { --delta; }
  auto &extent = plan.extents[bank][form.index];
  if (!extent.accessed) {
    extent.minimum = extent.maximum = delta;
  } else {
    extent.minimum = std::min(extent.minimum, delta);
    extent.maximum = std::max(extent.maximum, delta);
  }
  extent.accessed = true;
  extent.writing = extent.writing || !move.load;
  if (mode == 3 && advance) { ++delta; }
  if (mode == 2 && advance) { --delta; }
  return true;
}

bool append_moves(RegionPlan &plan, const NativeMoves &moves) {
  if (moves.long_pair) {
    const auto index = moves.transfers[0].address.index;
    const auto prior = plan.deltas[index];
    if (!append_move(plan, moves.transfers[0], true)) { return false; }
    plan.deltas[index] = prior;
    return append_move(plan, moves.transfers[1], true);
  }
  Word updated = 0;
  for (const auto &move : std::span(moves.transfers).first(moves.count)) {
    if (!plain_register(move.reg)) { return false; }
    const auto bit = 1U << move.address.index;
    const bool advance = (updated & bit) == 0;
    if (!advance) { return false; }
    if (!append_move(plan, move, advance)) { return false; }
    updated |= bit;
  }
  for (const auto &move : std::span(moves.transfers).first(moves.count)) {
    if (move.load && !destination(plan, move.reg)) { return false; }
  }
  return true;
}

RegionControl control(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const auto opcode = operation.instruction.opcode;
  constexpr Word short_address_mask = 0xfff;
  const bool conditional = kind == Operation::conditional_jump;
  if (!conditional && kind != Operation::jump_short && kind != Operation::jump_effective) {
    return {};
  }
  if (conditional && (opcode & (1U << 16)) != 0) { return {}; }
  const bool short_form = conditional ? (opcode & (1U << 18)) != 0 : kind == Operation::jump_short;
  const auto mode = (opcode >> 11) & 7U;
  const auto index = (opcode >> 8) & 7U;
  if (!short_form && mode != 4 && (mode != 6 || index != 0)) { return {}; }
  return {.target = short_form ? opcode & short_address_mask : operation.instruction.extension,
          .selector = conditional ? (opcode >> (short_form ? 12U : 0U)) & 15U : 16U,
          .reg = !short_form && mode == 4 ? index : static_cast<unsigned>(agu_registers),
          .valid = true};
}

bool append(RegionPlan &plan, const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::arithmetic_shift_immediate) {
    constexpr Word count_mask = 63;
    if (((instruction.opcode >> 1) & count_mask) >= accumulator_bits) { return false; }
    plan.steps.push_back({});
    return true;
  }
  if ((instruction.operation != Operation::parallel && instruction.operation != Operation::nop) ||
      !llvm19::supported_alu(instruction.opcode)) {
    return false;
  }
  const auto moves = plan_moves(instruction);
  const auto registers = plan_register_move(instruction);
  if (!append_moves(plan, moves)) { return false; }
  if (registers.valid() &&
      (!plain_register(registers.source) || !destination(plan, registers.destination))) {
    return false;
  }
  if (instruction.operation == Operation::parallel && moves.count == 0 && !registers.valid() &&
      (instruction.opcode & upper_move_mask) != no_move_group) {
    return false;
  }
  plan.steps.push_back({.memory = moves, .registers = registers});
  return true;
}
}

std::optional<RegionPlan> plan_region(const std::span<const BlockInstruction> operations,
                                      const JitRuntime &runtime) {
  if (operations.empty()) { return {}; }
  constexpr Word supported_modes = rounding_mode | scale_down | scale_up;
  const auto mode = operations.front().mode;
  if ((mode & ~supported_modes) != 0 ||
      (mode & (scale_down | scale_up)) == (scale_down | scale_up)) {
    return {};
  }
  RegionPlan plan{
      .operations = operations, .steps = {}, .windows = declared_windows(runtime), .control = {}};
  plan.control = control(operations.back());
  plan.batch = at_loop_end(operations.back()) && !plan.control.valid;
  auto pc = operations.front().pc;
  for (const auto &operation : operations) {
    if (operation.pc != pc || operation.mode != mode ||
        operation.loop_address != operations.front().loop_address) {
      return {};
    }
    pc = (pc + operation.instruction.words) & word_mask;
    if (&operation == &operations.back() && plan.control.valid) {
      plan.steps.push_back({});
    } else if (!append(plan, operation)) {
      return {};
    }
    plan.cost += operation.instruction.cycles;
    ++plan.count;
  }
  if (!plan.batch && !std::ranges::any_of(plan.extents, [](const auto &bank) {
        return std::ranges::any_of(bank, [](const auto &extent) { return extent.accessed; });
      })) {
    return {};
  }
  if (!stable_knowledge(plan)) { return {}; }
  return plan;
}
}
