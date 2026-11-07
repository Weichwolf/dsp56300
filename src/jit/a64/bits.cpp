// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::modify_bit(const BitAction action, const unsigned bit) {
  code_.extract(Reg::x7, Reg::x0, bit, 1);
  code_.store(Reg::x7, Reg::sp, Frame::secondary);
  if (action == BitAction::test) { return; }
  code_.constant(Reg::x17, Word{1} << bit);
  Alu operation = Alu::bit_clear;
  if (action == BitAction::set) {
    operation = Alu::bit_or;
  } else if (action == BitAction::change) {
    operation = Alu::bit_xor;
  }
  code_.alu(operation, Reg::x0, Reg::x0, Reg::x17);
}

void Emitter::register_bit(const BlockInstruction &operation, const BitAccess access) {
  const auto number = access.operand.selector;
  if ((number == a_register || number == b_register) && !numeric_mode(operation, true)) { return; }
  const bool modify = access.action != BitAction::test;
  const bool high = number == ssh_register && modify;
  if (high) {
    stack_slot(false);
    code_.load(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    code_.shift(Shift::right, Reg::x0, Reg::x0, word_bits, Width::x64);
  } else {
    read_register(operation, number);
  }
  modify_bit(access.action, access.bit);
  if (modify) {
    if (high) {
      stack_slot(false);
      code_.load(Reg::x9, Reg::x11, offsetof(Registers, stack), Access::doubleword);
      mask(Reg::x9, Reg::x9, word_mask, Width::w32);
      code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
      code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x9, Width::x64);
      code_.store(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    } else {
      write_register(operation, number);
    }
  }
  code_.load(Reg::x0, Reg::sp, Frame::secondary);
}

void Emitter::bit_branch(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  flow_ = Flow::control;
  code_.constant(Reg::x17, (instruction.opcode >> 5) & 1U);
  code_.compare(Reg::x0, Reg::x17);
  Label complete;
  code_.branch_far(Condition::not_equal, complete);
  if (bit_call(instruction)) {
    code_.shift(Shift::left, Reg::x0, pc, word_bits, Width::x64);
    code_.alu(Alu::bit_or, Reg::x0, Reg::x0, status, Width::x64);
    push_frame(operation);
  }
  if (instruction.operation == Operation::bit_jump) {
    code_.constant(pc, instruction.extension);
  } else {
    code_.constant(pc, (operation.pc + instruction.extension) & word_mask);
  }
  code_.bind(complete);
}

bool Emitter::bits(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto kind = instruction.operation;
  const bool branching = kind == Operation::bit_jump || kind == Operation::bit_branch;
  if (!branching && kind != Operation::bit_register && kind != Operation::bit_memory) {
    return false;
  }
  const auto bit = instruction.opcode & five_bit_mask;
  if (bit >= word_bits) {
    code_.constant(Reg::x0, instruction.opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return true;
  }
  const BitAccess access{.operand = bit_operand(instruction),
                         .action = branching ? BitAction::test : bit_action(instruction.opcode),
                         .bit = bit};
  if (access.operand.location == BitLocation::reg) {
    register_bit(operation, access);
  } else {
    memory_bit(operation, access);
  }
  if (branching) {
    bit_branch(operation);
  } else if (access.operand.location != BitLocation::reg ||
             access.operand.selector != sr_register || access.action == BitAction::test) {
    mask(status, status, word_mask ^ carry, Width::w32);
    code_.alu(Alu::bit_or, status, status, Reg::x0);
  }
  return true;
}
}
