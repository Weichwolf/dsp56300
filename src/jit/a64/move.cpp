// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <stdexcept>

namespace dsp56300::detail::a64 {
void Emitter::register_move(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  if ((opcode & register_group_mask) == no_move_group) {
    const auto source = (opcode >> 13) & five_bit_mask;
    const auto target = (opcode >> 8) & five_bit_mask;
    if (source < x0_register) {
      code_.constant(Reg::x0, 0);
      if (target >= x0_register) { write_register(operation, target); }
      code_.constant(Reg::x0, source);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
      return;
    }
    read_register(operation, source);
    write_register(operation, target);
    return;
  }
  const auto target = (opcode >> 16) & five_bit_mask;
  auto value = (opcode >> 8) & byte_mask;
  const bool aligned = target == x0_register || target == x1_register || target == y0_register ||
                       target == y1_register || target == a_register || target == b_register;
  if (aligned) { value <<= 16; }
  code_.constant(Reg::x0, value);
  write_register(operation, target);
}

void Emitter::moves(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  if ((opcode & upper_move_mask) == no_move_group) { return; }
  if ((opcode & update_group_mask) == update_group) {
    effective(operation, {.mode = (opcode >> 11) & 3U, .index = (opcode >> 8) & 7U});
  } else if ((opcode & parallel_group_mask) == immediate_group) {
    register_move(operation);
  } else {
    throw std::invalid_argument("A64 DSP parallel memory transfer lowering is incomplete");
  }
}
}
