// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>

namespace dsp56300::detail::a64 {
void Emitter::mixed_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  const bool load = (opcode & (1U << 15)) != 0;
  if (form.mode == 6 && form.index == 4 && !load) {
    code_.constant(Reg::x0, form.index);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  const bool y = (opcode & (1U << 14)) != 0;
  const auto accumulator = (opcode & (1U << (y ? 19 : 17))) != 0 ? b_register : a_register;
  const auto second = y ? x0_register + ((opcode >> 18) & 1U) : y0_register + ((opcode >> 16) & 1U);
  constexpr std::array xregs{x0_register, x1_register, a_register, b_register};
  constexpr std::array yregs{y0_register, y1_register, a_register, b_register};
  const auto first = y ? yregs.at((opcode >> 16) & 3U) : xregs.at((opcode >> 18) & 3U);
  effective(operation, form);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  read_register(operation, accumulator);
  code_.store(Reg::x0, Reg::sp, Frame::secondary);
  if (load) {
    if (form.mode == 6 && form.index == 4) {
      code_.constant(Reg::x0, instruction.extension);
    } else {
      code_.load(Reg::x0, Reg::sp, Frame::address);
      memory(operation, {.space = y ? Space::y : Space::x});
    }
    write_register(operation, first);
  } else {
    read_register(operation, first);
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = y ? Space::y : Space::x, .write = true});
  }
  code_.load(Reg::x0, Reg::sp, Frame::secondary);
  write_register(operation, second);
}
}
