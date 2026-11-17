// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region_plan.hpp"

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/move.hpp"
#include "jit/preparation.hpp"
#include "jit/translation.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::detail {
bool permitted(const RegionWindow &window, const bool writing) {
  return window.size != 0 && window.address < io_begin &&
         window.size <= io_begin - window.address &&
         (!writing || (window.writable && !window.aliases_program));
}

namespace {
auto declared_windows(const JitRuntime &runtime) {
  std::array<std::vector<RegionWindow>, 2> result;
  for (std::size_t bank = 0; bank < result.size(); ++bank) {
    if (!runtime.windows[bank].empty()) {
      for (const auto &window : runtime.windows[bank]) {
        result[bank].push_back({.address = window.address,
                                .size = window.words.size(),
                                .tracked = !window.known.empty(),
                                .writable = window.writable,
                                .aliases_program = window.aliases_program,
                                .data = window.words.data(),
                                .known = window.known.data()});
      }
      continue;
    }
    const auto &memory = runtime.memory[bank];
    if (memory.data != nullptr && memory.words != 0) {
      result[bank].push_back({.size = std::min(memory.words, std::size_t{io_begin}),
                              .aliases_program = memory.aliases_program,
                              .data = memory.data});
    }
  }
  return result;
}

bool stable_knowledge(const RegionPlan &plan) {
  std::vector<std::span<const std::byte>> masks;
  for (const auto &bank : plan.windows) {
    for (const auto &window : bank) {
      if (window.tracked) { masks.push_back(std::as_bytes(std::span(window.known, window.size))); }
    }
  }
  for (std::size_t bank = 0; bank < plan.windows.size(); ++bank) {
    if (!std::ranges::any_of(plan.extents[bank],
                             [](const auto &extent) { return extent.writing; })) {
      continue;
    }
    for (const auto &window : plan.windows[bank]) {
      if (!permitted(window, true)) { continue; }
      const auto data = std::bit_cast<std::uintptr_t>(window.data);
      for (const auto mask : masks) {
        const auto known = std::bit_cast<std::uintptr_t>(mask.data());
        if (data <= known ? known - data < window.size * sizeof(Word)
                          : data - known < mask.size()) {
          return false;
        }
      }
    }
  }
  return true;
}

bool plain_register(const unsigned reg) {
  return (reg >= x0_register && reg <= b_register) ||
         (reg >= r_register_base && reg < m_register_base);
}

bool destination(RegionPlan &plan, const unsigned reg) {
  if (!plain_register(reg)) { return false; }
  if (reg >= n_register_base) {
    plan.changed_n[reg - n_register_base] = true;
  } else if (reg >= r_register_base) {
    const auto index = reg - r_register_base;
    plan.definitions[index] = plan.steps.size();
    plan.guardable[index] =
        std::ranges::none_of(plan.absolute,
                             [](const auto &bank) {
                               return std::ranges::any_of(
                                   bank, [](const auto &entry) { return entry.second; });
                             }) &&
        std::ranges::none_of(plan.extents, [index](const auto &bank) {
          return bank[index].accessed ||
                 std::ranges::any_of(bank, [](const auto &extent) { return extent.writing; });
        });
  }
  if (reg >= r_register_base) { plan.batch = false; }
  return true;
}

bool append_absolute(RegionPlan &plan, const NativeMove &move) {
  const auto form = move.address;
  if (move.immediate()) { return move.load; }
  if (form.index != 0) { return false; }
  const auto bank = move.space == Space::x ? 0U : 1U;
  const auto at = form.extra & word_mask;
  if (!std::ranges::any_of(plan.windows[bank], [&](const auto &window) {
        return permitted(window, !move.load) && at >= window.address &&
               at - window.address < window.size;
      })) {
    return false;
  }
  auto &writing = plan.absolute[bank][at];
  writing = writing || !move.load;
  return true;
}

bool append_move(RegionPlan &plan, const NativeMove &move, const bool advance) {
  const auto form = move.address;
  const auto mode = form.mode;
  if (mode == 6) { return append_absolute(plan, move); }
  plan.agu[form.index] = plan.agu[form.index] || mode != 4;
  const auto definition = plan.definitions[form.index];
  if ((definition && !plan.guardable[form.index]) ||
      ((mode == 0 || mode == 1 || mode == 5) && plan.changed_n[form.index])) {
    return false;
  }
  if (definition && !plan.deferred[form.index]) {
    plan.deferred[form.index] = true;
    plan.steps[*definition].guards |= 1U << form.index;
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
    const auto bit = move.address.mode == 6 ? 0U : 1U << move.address.index;
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

RegionControl relative_control(const BlockInstruction &operation) {
  const auto flow = successors(operation);
  if (flow.call || flow.indirect || flow.count == 0) { return {}; }
  const auto opcode = operation.instruction.opcode;
  const bool short_form = (opcode & 0xff0000U) == 0x050000U;
  return {.target = flow.direct[flow.count - 1],
          .selector = flow.count == 1 ? 16U : (opcode >> (short_form ? 12U : 0U)) & 15U,
          .reg = static_cast<unsigned>(agu_registers),
          .valid = true};
}

RegionControl control(const BlockInstruction &operation) {
  if (operation.instruction.operation == Operation::relative_branch) {
    return relative_control(operation);
  }
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

bool append_bit(RegionPlan &plan, const Instruction &instruction) {
  const auto operand = bit_operand(instruction);
  Word at = operand.selector;
  if (operand.location == BitLocation::effective && operand.selector == (6U << 3)) {
    at = instruction.extension & word_mask;
  } else if (operand.location != BitLocation::short_address) {
    return false;
  }
  if ((instruction.opcode & five_bit_mask) >= word_bits) { return false; }
  const auto action = bit_action(instruction.opcode);
  const NativeMove move{.address = {.mode = 6, .extra = at},
                        .space = operand.space,
                        .load = action == BitAction::test};
  if (!append_absolute(plan, move)) { return false; }
  plan.computation = true;
  plan.steps.push_back({.memory = {.transfers = {move}, .count = 1}, .registers = {}});
  return true;
}

bool append(RegionPlan &plan, const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::bit_memory) { return append_bit(plan, instruction); }
  if (instruction.operation == Operation::arithmetic_shift_immediate) {
    plan.computation = true;
    constexpr Word count_mask = 63;
    if (((instruction.opcode >> 1) & count_mask) >= accumulator_bits) { return false; }
    plan.steps.push_back({});
    return true;
  }
  if ((instruction.operation != Operation::parallel && instruction.operation != Operation::nop) ||
      !llvm19::supported_alu(instruction.opcode)) {
    return false;
  }
  const auto immediate = plan_address_immediate(instruction);
  if (immediate.valid()) {
    if (!destination(plan, immediate.destination)) { return false; }
    plan.steps.push_back({.memory = {}, .registers = {}, .immediate = immediate});
    return true;
  }
  const auto moves = plan_moves(instruction);
  plan.computation = plan.computation || moves.long_pair || (instruction.opcode & byte_mask) != 0;
  const auto registers = plan_register_move(instruction, true);
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

struct TreeContext {
  Word weight = 1;
  unsigned depth = 0;
};

struct RepeatAnalysis {
  decltype(RegionPlan::extents) extents;
  decltype(RegionPlan::deltas) origins;
  decltype(RegionPlan::definitions) definitions;
};

bool repeat_extents(RegionPlan &plan,
                    const RepeatAnalysis &prior,
                    const Word repetitions,
                    RegionPlan::Repeat &repeat) {
  for (unsigned reg = 0; reg < agu_registers; ++reg) {
    std::int64_t travel = 0;
    if (__builtin_mul_overflow(plan.deltas[reg] - prior.origins[reg], repetitions - 1, &travel) ||
        __builtin_add_overflow(plan.deltas[reg], travel, &plan.deltas[reg])) {
      return false;
    }
    repeat.deltas[reg] = plan.deltas[reg] - prior.origins[reg];
    for (unsigned bank = 0; bank < 2; ++bank) {
      auto &extent = plan.extents[bank][reg];
      if (extent.accessed) {
        auto &edge = travel < 0 ? extent.minimum : extent.maximum;
        if (__builtin_add_overflow(edge, travel, &edge)) { return false; }
        repeat.extents[bank][reg] = {.minimum = extent.minimum - prior.origins[reg],
                                     .maximum = extent.maximum - prior.origins[reg],
                                     .accessed = true,
                                     .writing = extent.writing};
      }
      const auto &earlier = prior.extents[bank][reg];
      if (!earlier.accessed) { continue; }
      if (!extent.accessed) {
        extent = earlier;
      } else {
        extent.minimum = std::min(extent.minimum, earlier.minimum);
        extent.maximum = std::max(extent.maximum, earlier.maximum);
        extent.writing = extent.writing || earlier.writing;
      }
    }
  }
  return true;
}

bool append_tree(RegionPlan &plan, std::size_t first, std::size_t last, TreeContext context = {});

std::optional<std::size_t> append_repeat(RegionPlan &plan,
                                         const std::size_t index,
                                         const std::size_t last,
                                         const TreeContext context) {
  const auto &operation = plan.operations[index];
  const auto flow = successors(operation);
  const auto repetitions = ((operation.instruction.opcode >> 8) & byte_mask) |
                           ((operation.instruction.opcode & 15U) << 8);
  const auto weight = std::uint64_t{context.weight} * repetitions;
  if (repetitions == 0 || flow.count != 2 || index + 1 >= last ||
      flow.direct[0] != plan.operations[index + 1].pc ||
      weight > std::numeric_limits<Word>::max()) {
    return {};
  }
  const auto remaining = plan.operations.subspan(index + 1, last - index - 1);
  const auto end = std::ranges::find(remaining, flow.direct[1], &BlockInstruction::pc);
  if (end == remaining.end() || end->pc != flow.direct[1]) { return {}; }
  const auto after = static_cast<std::size_t>(end - plan.operations.begin());
  plan.steps.push_back(
      {.memory = {}, .registers = {}, .repeat_end = after, .repetitions = repetitions});
  const RepeatAnalysis prior{
      .extents = plan.extents, .origins = plan.deltas, .definitions = plan.definitions};
  plan.extents = {};
  const auto batch = plan.batch;
  plan.batch = true;
  RegionPlan::Repeat repeat;
  if (!append_tree(plan,
                   index + 1,
                   after,
                   {.weight = static_cast<Word>(weight), .depth = context.depth + 1}) ||
      !plan.batch || plan.definitions != prior.definitions ||
      !repeat_extents(plan, prior, repetitions, repeat)) {
    return {};
  }
  plan.batch = batch;
  plan.repeats.emplace(index, repeat);
  return after;
}

bool append_tree(RegionPlan &plan,
                 const std::size_t first,
                 const std::size_t last,
                 const TreeContext context) {
  if (context.depth >= stack_entries / 2) { return false; }
  plan.depth = std::max(plan.depth, context.depth);
  for (auto index = first; index < last; ++index) {
    const auto &operation = plan.operations[index];
    const auto count = std::uint64_t{plan.count} + context.weight;
    const auto cost =
        std::uint64_t{plan.cost} + (std::uint64_t{context.weight} * operation.instruction.cycles);
    if (count > std::numeric_limits<Word>::max() || cost > std::numeric_limits<Word>::max()) {
      return false;
    }
    plan.count = static_cast<Word>(count);
    plan.cost = static_cast<Word>(cost);
    if (operation.instruction.operation == Operation::loop_immediate) {
      const auto after = append_repeat(plan, index, last, context);
      if (!after) { return false; }
      index = *after - 1;
    } else if (index + 1 == plan.operations.size() && plan.control.valid) {
      plan.steps.push_back({});
    } else if (!append(plan, operation)) {
      return false;
    }
  }
  return true;
}

std::optional<RegionPlan> make_plan(RegionPlan plan) {
  const auto operations = plan.operations;
  if (operations.empty()) { return {}; }
  if (operations.front().instruction.operation == Operation::loop_immediate &&
      operations.front().loop_address == inactive_loop) {
    const auto &first = operations.front();
    const auto flow = successors(first);
    const auto count =
        ((first.instruction.opcode >> 8) & byte_mask) | ((first.instruction.opcode & 15U) << 8);
    if (operations.size() < 2 || count == 0 || flow.count != 2 ||
        flow.direct[0] != operations[1].pc ||
        flow.direct[1] != operations.back().pc + operations.back().instruction.words) {
      return {};
    }
    plan.operations = operations.subspan(1);
    auto body = make_plan(std::move(plan));
    if (!body || !body->batch || body->setup || body->depth + 1U >= stack_entries / 2 ||
        (std::uint64_t{body->count} * count) + 1 > std::numeric_limits<Word>::max() ||
        (std::uint64_t{body->cost} * count) + first.instruction.cycles >
            std::numeric_limits<Word>::max()) {
      return {};
    }
    body->setup = first;
    body->setup_count = count;
    return body;
  }
  constexpr Word supported_modes = rounding_mode | scale_down | scale_up;
  const auto mode = operations.front().mode;
  if ((mode & ~supported_modes) != 0 ||
      (mode & (scale_down | scale_up)) == (scale_down | scale_up)) {
    return {};
  }
  plan.control = control(operations.back());
  plan.batch = at_loop_end(operations.back()) && !plan.control.valid;
  auto pc = operations.front().pc;
  for (const auto &operation : operations) {
    if (operation.pc != pc || operation.mode != mode ||
        operation.loop_address != operations.front().loop_address) {
      return {};
    }
    pc = (pc + operation.instruction.words) & word_mask;
  }
  if (!append_tree(plan, 0, operations.size())) { return {}; }
  if (!plan.batch &&
      std::ranges::all_of(plan.absolute, [](const auto &bank) { return bank.empty(); }) &&
      !std::ranges::any_of(plan.extents, [](const auto &bank) {
        return std::ranges::any_of(bank, [](const auto &extent) { return extent.accessed; });
      })) {
    return {};
  }
  return plan;
}
}

std::optional<RegionPlan> plan_region(const std::span<const BlockInstruction> operations,
                                      const JitRuntime &runtime) {
  for (const auto windows : runtime.windows) {
    for (const auto &window : windows) {
      if (!window.known.empty() && window.known.size() != window.words.size()) { return {}; }
    }
  }
  auto plan = make_plan({.operations = operations,
                         .steps = {},
                         .repeats = {},
                         .windows = declared_windows(runtime),
                         .windowed = {!runtime.windows[0].empty(), !runtime.windows[1].empty()},
                         .control = {}});
  if (plan && !stable_knowledge(*plan)) { return {}; }
  return plan;
}

std::optional<RegionPlan> plan_region(const std::span<const BlockInstruction> operations,
                                      const std::span<const RegionLayout> memory) {
  const bool external = std::ranges::none_of(
      memory, [](const auto &region) { return region.space == Space::program; });
  RegionPlan plan{.operations = operations,
                  .steps = {},
                  .repeats = {},
                  .windows = {},
                  .windowed = {external, external},
                  .control = {}};
  for (const auto &region : memory) {
    if (region.space != Space::x && region.space != Space::y) { continue; }
    if (!external && (region.first != 0 || region.tracked || !region.writable)) { return {}; }
    plan.windows[region.space == Space::x ? 0 : 1].push_back(
        {.address = region.first,
         .size = external ? region.size : std::min(region.size, std::size_t{io_begin}),
         .tracked = region.tracked,
         .writable = region.writable,
         .aliases_program = region.aliases_program});
  }
  // Exported entries validate actual storage and mask aliases at admission.
  return make_plan(std::move(plan));
}
}
