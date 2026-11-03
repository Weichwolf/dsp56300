// SPDX-License-Identifier: GPL-3.0-only
#include "instruction_boundary.hpp"

#include "fields.hpp"
#include "instruction.hpp"
#include "native_moves.hpp"

#include <algorithm>
#include <array>
#include <span>

namespace dsp56300::detail {
bool requires_owner(const Instruction &instruction) {
  if (instruction.operation == Operation::parallel) {
    if ((instruction.opcode & upper_move_mask) == no_move_group ||
        pure_address_update(instruction) || plan_address_immediate(instruction).valid() ||
        plan_register_move(instruction).valid()) {
      return false;
    }
    const auto moves = plan_moves(instruction);
    if (moves.count == 0) { return true; }
    return std::ranges::any_of(std::span(moves.transfers).first(moves.count),
                               [](const auto &move) { return !move.load || !move.immediate(); });
  }
  constexpr std::array pure{Operation::nop,
                            Operation::jump_short,
                            Operation::immediate_alu,
                            Operation::repeat_immediate,
                            Operation::loop_immediate,
                            Operation::arithmetic_shift_immediate,
                            Operation::arithmetic_shift_register,
                            Operation::logical_shift_immediate,
                            Operation::logical_shift_register,
                            Operation::divide_iteration,
                            Operation::normalize_iteration,
                            Operation::normalize_fast,
                            Operation::product_immediate,
                            Operation::product_power};
  return std::ranges::find(pure, instruction.operation) == pure.end();
}
}
