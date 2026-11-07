// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"

namespace dsp56300::detail {
[[nodiscard]] inline bool permitted_vector_move(const Instruction &instruction) {
  const auto kind = instruction.operation;
  Word reg = 0;
  bool load = false;
  constexpr Word direction = Word{1} << 15;
  if (kind == Operation::move_program_effective || kind == Operation::move_program_short) {
    reg = instruction.opcode & six_bit_mask;
    load = (instruction.opcode & direction) != 0;
  } else if (kind == Operation::move_peripheral_register) {
    reg = (instruction.opcode >> 8) & six_bit_mask;
    load = (instruction.opcode & direction) == 0;
  } else {
    return false;
  }
  if (!load) { return reg != ssh_register; }
  return reg != la_register && reg != lc_register && reg != sp_register && reg != sc_register &&
         reg != ssh_register && reg != ssl_register;
}

[[nodiscard]] inline bool permitted_vector_instruction(const Instruction &instruction) {
  const auto kind = instruction.operation;
  return kind == Operation::parallel || kind == Operation::nop ||
         kind == Operation::move_peripheral_memory || kind == Operation::move_peripheral_program ||
         permitted_vector_move(instruction) || kind == Operation::reset_peripherals ||
         kind == Operation::debug || kind == Operation::conditional_debug;
}
}
