// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/resident.hpp"
#include "move.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

namespace dsp56300::detail {
namespace {
constexpr Word minimum_transfers = 8;

void unit_extent(RegionAddress &address, const Word mode) {
  if (mode == 7) { --address.displacement; }
  if (!address.accessed) {
    address.minimum = address.maximum = address.displacement;
  } else {
    address.minimum = std::min(address.minimum, address.displacement);
    address.maximum = std::max(address.maximum, address.displacement);
  }
  if (mode == 3) {
    ++address.displacement;
  } else if (mode == 2) {
    --address.displacement;
  }
  if (mode == 2 || mode == 7) { address.updated = address.retreats = true; }
}

bool transfer(ResidentPlan &region,
              const NativeMove &move,
              const JitMemory &memory,
              const std::array<Word, agu_registers> &modifiers,
              const RegionAddressing addressing) {
  // An R/N load can replace a later cursor/stride inside this region. Keep
  // ordinary per-access guards until its loaded value has a region proof.
  if (move.load && move.reg >= r_register_base) { return false; }
  if (!move.load && memory.aliases_program) { return false; }
  if (move.immediate()) { return true; }
  if (memory.data == nullptr) { return false; }
  const auto bound = static_cast<Word>(std::min(memory.words, std::size_t{io_begin}));
  const auto &form = move.address;
  ++region.transfers;
  if (form.mode == 6) { return form.index == 0 && form.extra < bound; }
  if (form.mode != 1 && form.mode != 3 && form.mode != 4 &&
      (addressing != RegionAddressing::bidirectional || (form.mode != 2 && form.mode != 7))) {
    return false;
  }
  auto &address = region.addresses.at(form.index);
  unit_extent(address, form.mode);
  address.accessed = true;
  address.modifier = modifiers.at(form.index) & modifier_mask;
  address.bound = std::min(address.bound, bound);
  const auto limit = address.advances < bound ? bound - address.advances : 0U;
  address.linear_limit = std::min(address.linear_limit, limit);
  // Separate maxima conservatively bound every mixed unit/N access. A final
  // post-N update has no later access to prove and may wrap normally.
  address.stride_accesses = std::max(address.stride_accesses, address.stride_steps);
  if (form.mode == 1 || form.mode == 3) {
    address.updated = true;
    if (form.mode == 1) {
      ++address.stride_steps;
    } else {
      ++address.advances;
    }
  }
  return true;
}

bool moves(ResidentPlan &region,
           const Instruction &instruction,
           const std::array<JitMemory, 2> &memory,
           const std::array<Word, agu_registers> &modifiers,
           const RegionAddressing addressing) {
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & upper_move_mask) == no_move_group) {
    return true;
  }
  const auto registers = plan_register_move(instruction);
  if (registers.valid()) { return registers.destination < r_register_base; }
  const auto immediate = plan_address_immediate(instruction);
  if (immediate.valid()) {
    if (immediate.destination >= n_register_base) { return false; }
    auto &address = region.addresses.at(immediate.destination - r_register_base);
    if (address.accessed) { return false; }
    address.constant = immediate.value;
    return true;
  }
  const auto plan = plan_moves(instruction);
  if (plan.count == 0 || plan.long_pair) { return false; }
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    if (!transfer(region, move, memory.at(move.space == Space::x ? 0 : 1), modifiers, addressing)) {
      return false;
    }
  }
  return true;
}

bool address_supported(const RegionAddress &address) {
  if (!address.accessed) { return true; }
  if (address.retreats) {
    return address.modifier == modifier_mask && address.stride_steps == 0 && address.bound != 0;
  }
  if (address.stride_steps != 0 && address.modifier != modifier_mask) { return false; }
  if (!address.updated || address.modifier == modifier_mask) { return address.linear_limit != 0; }
  return address.modifier > 0 && address.modifier < multiple_wrap_bit &&
         address.modifier < address.bound;
}
}

JitRuntime resident_runtime(JitRuntime runtime) {
  for (std::size_t bank = 0; bank < runtime.memory.size(); ++bank) {
    if (runtime.windows[bank].empty()) { continue; }
    // Only the entry-guarded region receives this view; rejected entries retain sparse guards.
    const auto found = std::ranges::find_if(runtime.windows[bank], [](const auto &window) {
      return window.address == 0 && window.known.empty() && window.writable &&
             !window.aliases_program;
    });
    if (found == runtime.windows[bank].end()) { continue; }
    runtime.memory[bank] = {.data = found->words.data(), .words = found->words.size()};
    runtime.windows[bank] = {};
  }
  return runtime;
}

ResidentPlan resident_plan(const std::span<const BlockInstruction> operations,
                           const std::array<JitMemory, 2> &memory,
                           const std::array<Word, agu_registers> &modifiers,
                           const RegionAddressing addressing) {
  ResidentPlan region;
  if (operations.empty()) { return region; }
  auto pc = operations.front().pc;
  for (std::size_t index = 0; index < operations.size(); ++index) {
    const auto &operation = operations[index];
    const auto kind = operation.instruction.operation;
    if ((kind != Operation::parallel && kind != Operation::nop &&
         kind != Operation::immediate_alu) ||
        operation.pc != pc || operation.mode != operations.front().mode ||
        operation.loop_address != operations.front().loop_address ||
        (at_loop_end(operation) && index + 1 != operations.size()) ||
        !moves(region, operation.instruction, memory, modifiers, addressing)) {
      return {};
    }
    pc = (pc + operation.instruction.words) & word_mask;
  }
  region.loop = at_loop_end(operations.back());
  region.valid =
      (region.transfers >= minimum_transfers || (region.loop && region.transfers != 0)) &&
      std::ranges::all_of(region.addresses, address_supported);
  return region;
}
} // namespace dsp56300::detail
