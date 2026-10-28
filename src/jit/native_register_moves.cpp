// SPDX-License-Identifier: GPL-3.0-only
#include "fields.hpp"
#include "instruction.hpp"
#include "native_moves.hpp"

namespace dsp56300::detail {
namespace {
bool data_register(const unsigned reg) {
  return (reg >= x0_register && reg <= y1_register) || reg == a_register || reg == b_register;
}
}

NativeRegisterMove plan_register_move(const Instruction &instruction) {
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & register_group_mask) != no_move_group) {
    return {};
  }
  const auto opcode = instruction.opcode;
  const auto source = (opcode >> 13) & five_bit_mask;
  const auto destination = (opcode >> 8) & five_bit_mask;
  if (!data_register(source) || !data_register(destination)) { return {}; }
  // Duplicate accumulator destinations are prohibited. Keep such encodings
  // general rather than assigning an additional native ordering convention.
  if ((opcode & byte_mask) != 0 && destination >= a_register &&
      ((opcode >> 3) & 1U) == destination - a_register) {
    return {};
  }
  return {.source = source, .destination = destination};
}
} // namespace dsp56300::detail
