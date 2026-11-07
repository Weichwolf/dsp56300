// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::jump(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const auto opcode = operation.instruction.opcode;
  const bool conditional = kind == Operation::conditional_jump;
  const bool short_address = conditional
                                 ? (opcode & (1U << 18)) != 0
                                 : kind == Operation::jump_short || kind == Operation::call_short;
  const bool call = conditional
                        ? (opcode & (1U << 16)) != 0
                        : kind == Operation::call_short || kind == Operation::call_effective;
  flow_ = Flow::control;
  if (short_address) {
    constexpr Word address_mask = 0xfff;
    code_.constant(Reg::x0, opcode & address_mask);
  } else {
    const AddressForm form{.mode = (opcode >> 11) & 7U,
                           .index = (opcode >> 8) & 7U,
                           .extra = operation.instruction.extension};
    if (form.mode == 6 && form.index != 0) {
      code_.constant(Reg::x0, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    } else {
      effective(operation, form);
    }
  }
  Label complete;
  if (conditional) {
    condition(Reg::x7, (opcode >> (short_address ? 12U : 0U)) & 15U);
    code_.branch_zero_far(Reg::x7, complete);
  }
  code_.move(pc, Reg::x0, Width::w32);
  if (call) {
    code_.constant(Reg::x0, (operation.pc + operation.instruction.words) & word_mask);
    code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
    code_.alu(Alu::bit_or, Reg::x0, Reg::x0, status, Width::x64);
    push_frame(operation);
  }
  code_.bind(complete);
}

void Emitter::branch(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr Word short_group_mask = 0xff0000;
  constexpr Word short_group = 0x050000;
  const bool short_offset = (opcode & short_group_mask) == short_group;
  const bool unconditional = (opcode & (1U << (short_offset ? 11 : 7))) != 0;
  const bool call = (opcode & (1U << (short_offset ? 10 : 6))) == 0;
  flow_ = Flow::control;
  Label complete;
  if (!unconditional) {
    condition(Reg::x7, (opcode >> (short_offset ? 12U : 0U)) & 15U);
    code_.branch_zero_far(Reg::x7, complete);
  }
  if (short_offset) {
    constexpr Word upper = 0x1e0;
    constexpr Word lower = 0x1f;
    constexpr Word sign = 0x100;
    constexpr Word span_mask = 0x1ff;
    auto offset = ((opcode >> 1) & upper) | (opcode & lower);
    if ((offset & sign) != 0) { offset |= word_mask ^ span_mask; }
    code_.constant(Reg::x0, offset);
  } else if ((opcode & (1U << 11)) != 0) {
    code_.load(Reg::x0, cpu, offsetof(Registers, r) + (((opcode >> 8) & 7U) * sizeof(Word)));
  } else {
    code_.constant(Reg::x0, operation.instruction.extension);
  }
  code_.constant(Reg::x17, operation.pc);
  code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x17);
  mask(pc, Reg::x0, word_mask, Width::w32);
  if (call) {
    code_.constant(Reg::x0, (operation.pc + operation.instruction.words) & word_mask);
    code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
    code_.alu(Alu::bit_or, Reg::x0, Reg::x0, status, Width::x64);
    push_frame(operation);
  }
  code_.bind(complete);
}
}
