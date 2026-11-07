// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

namespace dsp56300::detail::a64 {
void Emitter::control_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto control = m_register_base + (opcode & five_bit_mask);
  if (!valid_register(control)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if (instruction.operation == Operation::move_control_immediate) {
    code_.constant(Reg::x0, (opcode >> 8) & byte_mask);
    write_register(operation, control);
    return;
  }
  const auto other = (opcode >> 8) & six_bit_mask;
  if (!valid_register(other)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if ((other == a_register || other == b_register) && !numeric_mode(operation, true)) { return; }
  const bool load = (opcode & (1U << 15)) != 0;
  read_register(operation, load ? other : control);
  write_register(operation, load ? control : other);
}

bool Emitter::transfer(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  if (kind == Operation::viterbi_shift) {
    viterbi_shift(operation);
    return true;
  }
  if (kind == Operation::move_control_effective || kind == Operation::move_control_short ||
      kind == Operation::move_program_effective || kind == Operation::move_program_short) {
    memory_transfer(operation);
    return true;
  }
  if (kind == Operation::move_displacement) {
    displacement_move(operation);
    return true;
  }
  if (kind == Operation::move_peripheral_register || kind == Operation::move_peripheral_memory ||
      kind == Operation::move_peripheral_program) {
    peripheral_move(operation);
    return true;
  }
  if (kind == Operation::move_control_register || kind == Operation::move_control_immediate) {
    control_move(operation);
    return true;
  }
  if (kind == Operation::load_relative_address || kind == Operation::load_updated_address) {
    address_calculation(operation);
    return true;
  }
  return false;
}
}
