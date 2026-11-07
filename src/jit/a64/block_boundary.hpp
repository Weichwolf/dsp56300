// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "fields.hpp"
#include "instruction.hpp"

namespace dsp56300::detail::a64 {
constexpr bool control_boundary(const unsigned reg) {
  return reg == sr_register || reg == omr_register || reg == la_register;
}

constexpr bool block_boundary(const Instruction &instruction) {
  const auto kind = instruction.operation;
  const auto opcode = instruction.opcode;
  if (kind == Operation::reset_peripherals || kind == Operation::cache_flush ||
      kind == Operation::cache_flush_unlocked || kind == Operation::cache_free ||
      kind == Operation::cache_lock || kind == Operation::cache_unlock) {
    return true;
  }
  if (kind == Operation::and_control || kind == Operation::or_control) {
    return (opcode & 3U) != 1;
  }
  const auto control = m_register_base + (opcode & five_bit_mask);
  const bool load = (opcode & (1U << 15)) != 0;
  if (kind == Operation::move_control_immediate) { return control_boundary(control); }
  if (kind == Operation::move_control_register) {
    return control_boundary(load ? control : (opcode >> 8) & six_bit_mask);
  }
  if (kind == Operation::move_control_effective || kind == Operation::move_control_short) {
    return load && control_boundary(control);
  }
  if (kind == Operation::move_program_effective || kind == Operation::move_program_short) {
    return !load || control_boundary(opcode & six_bit_mask);
  }
  return kind == Operation::move_peripheral_register && !load &&
         control_boundary((opcode >> 8) & six_bit_mask);
}
}
