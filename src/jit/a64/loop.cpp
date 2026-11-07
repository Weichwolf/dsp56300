// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::loop_count(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const auto opcode = operation.instruction.opcode;
  if (kind == Operation::loop_immediate || kind == Operation::repeat_immediate) {
    code_.constant(Reg::x0, ((opcode >> 8) & byte_mask) | ((opcode & 15U) << 8));
    return;
  }
  if (!numeric_mode(operation, true)) { return; }
  if (kind == Operation::loop_register || kind == Operation::repeat_register) {
    const auto selector = (opcode >> 8) & six_bit_mask;
    if (kind == Operation::loop_register && selector == ssh_register) {
      code_.constant(Reg::x0, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    } else {
      read_register(operation, selector);
    }
    return;
  }
  if (kind == Operation::loop_effective || kind == Operation::repeat_effective) {
    const AddressForm form{.mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U};
    if (form.mode == 6) {
      code_.constant(Reg::x0, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
      return;
    }
    effective(operation, form);
  } else {
    code_.constant(Reg::x0, (opcode >> 8) & six_bit_mask);
  }
  memory(operation, {.space = (opcode & (1U << 6)) != 0 ? Space::y : Space::x});
}

void Emitter::loop_pop(const BlockInstruction &operation) {
  code_.constant(Reg::x0, 0);
  Label resumed;
  failure_ = &resumed;
  pop_frame(operation);
  failure_ = nullptr;
  code_.bind(resumed);
}

void Emitter::loop_restore(const BlockInstruction &operation) {
  loop_pop(operation);
  code_.store(Reg::x0, Reg::sp, Frame::secondary, Access::doubleword);
  loop_pop(operation);
  code_.shift(Shift::right, Reg::x1, Reg::x0, word_bits, Width::x64);
  code_.store(Reg::x1, cpu, offsetof(Registers, la));
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  code_.store(Reg::x0, cpu, offsetof(Registers, lc));
  constexpr Word flags = loop_flag | forever_flag;
  code_.load(Reg::x0, Reg::sp, Frame::secondary, Access::doubleword);
  mask(Reg::x0, Reg::x0, flags, Width::w32);
  mask(status, status, word_mask ^ flags, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x0);
  code_.load(
      Reg::x17, context, offsetof(NativeContext, failure) + offsetof(Fault, kind), Access::byte);
  code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
  context_boundary();
}

void Emitter::loop_setup(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if ((operation.mode & compatibility_mode) != 0) {
    fault(operation, FaultKind::unsupported_mode, status);
    return;
  }
  const bool forever = instruction.operation == Operation::loop_forever;
  const bool relative = forever ? (instruction.opcode & 1U) == 0 : (instruction.opcode & 16U) != 0;
  const auto last =
      (relative ? operation.pc + instruction.extension : instruction.extension) & word_mask;
  code_.load(Reg::x0, cpu, offsetof(Registers, la));
  code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
  code_.load(Reg::x1, cpu, offsetof(Registers, lc));
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  push_frame(operation);
  if (forever) {
    code_.load(Reg::x0, cpu, offsetof(Registers, lc));
  } else {
    loop_count(operation);
  }
  Label nonzero;
  Label complete;
  if (!forever) {
    code_.branch_zero_far(Reg::x0, nonzero, Width::w32, true);
    pop_frame(operation);
    code_.constant(pc, (last + 1) & word_mask);
    code_.branch(complete);
    code_.bind(nonzero);
  }
  code_.store(Reg::x0, cpu, offsetof(Registers, lc));
  code_.shift(Shift::left, Reg::x0, pc, word_bits, Width::x64);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, status, Width::x64);
  push_frame(operation);
  code_.constant(Reg::x0, last);
  code_.store(Reg::x0, cpu, offsetof(Registers, la));
  mask(status, status, word_mask ^ forever_flag, Width::w32);
  code_.constant(Reg::x17, loop_flag | (forever ? forever_flag : 0U));
  code_.alu(Alu::bit_or, status, status, Reg::x17);
  boundary(NativeBoundary::context);
  code_.bind(complete);
}

bool Emitter::loop(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  if (is_repeat(kind)) {
    code_.load(Reg::x0, cpu, offsetof(Registers, lc));
    code_.store(Reg::x0, Reg::sp, Frame::other_value);
    loop_count(operation);
    code_.store(Reg::x0, cpu, offsetof(Registers, lc));
    constexpr Word zero_count = 65536;
    code_.constant(Reg::x1, zero_count);
    code_.compare(Reg::x0, Reg::zr);
    code_.select(Reg::x0, Reg::x1, Reg::x0, Condition::equal);
    code_.store(Reg::x0, cpu, offsetof(Cpu, repeat_remaining));
    code_.store(pc, cpu, offsetof(Cpu, repeat_pc));
    code_.load(Reg::x0, Reg::sp, Frame::other_value);
    code_.store(Reg::x0, cpu, offsetof(Cpu, repeat_lc));
    code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_loaded), Access::byte);
    boundary(NativeBoundary::owner);
    return true;
  }
  if (kind == Operation::end_loop || kind == Operation::break_loop) {
    flow_ = Flow::control;
    Label complete;
    if (kind == Operation::break_loop) {
      condition(Reg::x7, operation.instruction.opcode & 15U);
      code_.branch_zero_far(Reg::x7, complete);
      code_.load(Reg::x0, cpu, offsetof(Registers, la));
      code_.add(Reg::x0, Reg::x0, 1);
      mask(pc, Reg::x0, word_mask, Width::w32);
    }
    loop_restore(operation);
    code_.bind(complete);
    return true;
  }
  if (kind == Operation::loop_immediate || kind == Operation::loop_register ||
      kind == Operation::loop_short || kind == Operation::loop_effective ||
      kind == Operation::loop_forever) {
    loop_setup(operation);
    return true;
  }
  return false;
}

void Emitter::finish_loop(const BlockInstruction &operation) {
  Label complete;
  mask(Reg::x0, status, loop_flag, Width::w32);
  code_.branch_zero_far(Reg::x0, complete);
  code_.load(Reg::x1, cpu, offsetof(Registers, la));
  code_.constant(Reg::x17, (operation.pc + operation.instruction.words - 1) & word_mask);
  code_.compare(Reg::x1, Reg::x17);
  code_.branch_far(Condition::not_equal, complete);
  code_.add(Reg::x1, Reg::x1, 1);
  mask(Reg::x1, Reg::x1, word_mask, Width::w32);
  code_.compare(pc, Reg::x1);
  code_.branch_far(Condition::not_equal, complete);
  code_.load(Reg::x0, cpu, offsetof(Cpu, repeat_remaining));
  code_.branch_zero_far(Reg::x0, complete, Width::w32, true);
  Label supported;
  mask(Reg::x0, status, compatibility_mode, Width::w32);
  code_.branch_zero_far(Reg::x0, supported);
  fault(operation, FaultKind::unsupported_mode, status);
  code_.bind(supported);
  Label continuing;
  code_.load(Reg::x0, cpu, offsetof(Registers, lc));
  code_.constant(Reg::x17, 1);
  code_.compare(Reg::x0, Reg::x17);
  code_.branch_far(Condition::not_equal, continuing);
  mask(Reg::x1, status, forever_flag, Width::w32);
  code_.branch_zero_far(Reg::x1, continuing, Width::w32, true);
  loop_restore(operation);
  code_.branch(complete);
  code_.bind(continuing);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x0, Reg::x0, Reg::x17);
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  code_.store(Reg::x0, cpu, offsetof(Registers, lc));
  stack_slot(false);
  code_.load(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
  code_.shift(Shift::right, pc, Reg::x0, word_bits, Width::x64);
  code_.bind(complete);
}
}
