// SPDX-License-Identifier: GPL-3.0-only
#include "fields.hpp"
#include "instruction.hpp"
#include "move.hpp"

namespace dsp56300::detail {
namespace {
bool plain_register(const unsigned reg, const bool partial) {
  return (reg >= x0_register && reg <= y1_register) || reg == a_register || reg == b_register ||
         (partial && reg >= a0_register && reg <= b1_register) ||
         (reg >= r_register_base && reg < m_register_base);
}
}

NativeRegisterMove plan_register_move(const Instruction &instruction, const bool partial) {
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & register_group_mask) != no_move_group) {
    return {};
  }
  const auto opcode = instruction.opcode;
  const auto source = (opcode >> 13) & five_bit_mask;
  const auto destination = (opcode >> 8) & five_bit_mask;
  if (!plain_register(source, partial) || !plain_register(destination, partial)) { return {}; }
  // Duplicate accumulator destinations are prohibited. Keep such encodings
  // general rather than assigning an additional native ordering convention.
  if ((opcode & byte_mask) != 0 && destination >= a0_register && destination <= b_register &&
      ((opcode >> 3) & 1U) == (destination & 1U)) {
    return {};
  }
  return {.source = source, .destination = destination};
}

NativeRegisterMove plan_register_move(const Instruction &instruction) {
  return plan_register_move(instruction, false);
}
} // namespace dsp56300::detail
