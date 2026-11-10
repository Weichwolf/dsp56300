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
#include <bit>
#include <cstddef>
#include <span>

namespace dsp56300::detail {
namespace {
constexpr Word minimum_transfers = 8;

bool compound_addressing(const RegionAddressing addressing) {
  return addressing == RegionAddressing::compound || addressing == RegionAddressing::single_pass;
}

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
  if (move.load && move.reg >= r_register_base) {
    if (!compound_addressing(addressing) || move.reg >= n_register_base) { return false; }
  }
  if (!move.load && memory.aliases_program) { return false; }
  if (move.immediate()) { return true; }
  if (memory.data == nullptr) { return false; }
  const auto bound = static_cast<Word>(std::min(memory.words, std::size_t{io_begin}));
  const auto &form = move.address;
  ++region.transfers;
  if (form.mode == 6) { return form.index == 0 && form.extra < bound; }
  if (form.mode != 1 && form.mode != 3 && form.mode != 4 &&
      (addressing == RegionAddressing::forward || (form.mode != 2 && form.mode != 7))) {
    return false;
  }
  auto &address = region.addresses.at(form.index);
  if (address.replaced) { return false; }
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

bool mixed_transfer(ResidentPlan &region,
                    const Instruction &instruction,
                    const std::array<JitMemory, 2> &memory,
                    const std::array<Word, agu_registers> &modifiers,
                    const RegionAddressing addressing) {
  const auto opcode = instruction.opcode;
  const AddressForm form{.mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U};
  if (!compound_addressing(addressing) || form.mode == 6) { return false; }
  const bool y = (opcode & (1U << 14)) != 0;
  constexpr std::array xregs{x0_register, x1_register, a_register, b_register};
  constexpr std::array yregs{y0_register, y1_register, a_register, b_register};
  const NativeMove move{.address = form,
                        .reg = y ? yregs.at((opcode >> 16) & 3U) : xregs.at((opcode >> 18) & 3U),
                        .space = y ? Space::y : Space::x,
                        .load = (opcode & (1U << 15)) != 0};
  return transfer(region, move, memory[y ? 1U : 0U], modifiers, addressing);
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
  if ((instruction.opcode & pair_group_mask) == pair_group) {
    return mixed_transfer(region, instruction, memory, modifiers, addressing);
  }
  const auto plan = plan_moves(instruction);
  if (plan.count == 0) { return false; }
  if (plan.long_pair) {
    if (!compound_addressing(addressing)) { return false; }
    const auto index = plan.transfers[0].address.index;
    const auto original = region.addresses.at(index);
    if (!transfer(region, plan.transfers[0], memory[0], modifiers, addressing)) { return false; }
    const auto high = region.addresses.at(index);
    region.addresses.at(index) = original;
    if (!transfer(region, plan.transfers[1], memory[1], modifiers, addressing)) { return false; }
    auto &address = region.addresses.at(index);
    address.bound = std::min(address.bound, high.bound);
    address.linear_limit = std::min(address.linear_limit, high.linear_limit);
    return true;
  }
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    if (!transfer(region, move, memory.at(move.space == Space::x ? 0 : 1), modifiers, addressing)) {
      return false;
    }
    if (move.load && move.reg >= r_register_base) {
      region.addresses.at(move.reg - r_register_base).replaced = true;
    }
  }
  return true;
}

bool address_supported(const RegionAddress &address, const RegionAddressing addressing) {
  if (!address.accessed) { return true; }
  if (address.replaced && addressing != RegionAddressing::single_pass) { return false; }
  if ((addressing == RegionAddressing::masked_rings || compound_addressing(addressing)) &&
      address.updated && address.modifier != modifier_mask &&
      (address.modifier & multiple_wrap_bit) != 0) {
    const auto mask = address.modifier & multiple_wrap_mask;
    return (address.modifier & multiple_reserved_bit) == 0 && mask != 0 &&
           std::has_single_bit(mask + 1) && mask < address.bound;
  }
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
    const bool shift =
        compound_addressing(addressing) && kind == Operation::arithmetic_shift_immediate;
    if ((kind != Operation::parallel && kind != Operation::nop &&
         kind != Operation::immediate_alu && !shift) ||
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
      std::ranges::all_of(region.addresses, [addressing](const auto &address) {
        return address_supported(address, addressing);
      });
  return region;
}
} // namespace dsp56300::detail
