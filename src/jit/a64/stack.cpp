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
namespace {
constexpr unsigned pointer_mask = (stack_entries * 4) - 1;
constexpr unsigned frame_shift = 3;
}

void Emitter::push_frame(const BlockInstruction &operation) {
  Label available;
  code_.load(Reg::x9, cpu, offsetof(Registers, sc));
  code_.constant(Reg::x17, stack_entries - 1);
  code_.compare(Reg::x9, Reg::x17);
  code_.branch_far(Condition::no_carry, available);
  code_.load(Reg::x10, cpu, offsetof(Registers, sp));
  fault(operation, FaultKind::stack, Reg::x10);
  code_.bind(available);
  code_.load(Reg::x10, cpu, offsetof(Registers, sp));
  code_.add(Reg::x10, Reg::x10, 1);
  mask(Reg::x10, Reg::x10, pointer_mask, Width::w32);
  code_.store(Reg::x10, cpu, offsetof(Registers, sp));
  code_.add(Reg::x9, Reg::x9, 1);
  code_.store(Reg::x9, cpu, offsetof(Registers, sc));
  mask(Reg::x11, Reg::x10, stack_entries - 1);
  code_.shift(Shift::left, Reg::x11, Reg::x11, frame_shift, Width::x64);
  code_.alu(Alu::add, Reg::x11, cpu, Reg::x11, Width::x64);
  code_.store(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
}

void Emitter::pop_frame(const BlockInstruction &operation) {
  Label available;
  code_.load(Reg::x9, cpu, offsetof(Registers, sc));
  code_.branch_zero_far(Reg::x9, available, Width::w32, true);
  code_.load(Reg::x10, cpu, offsetof(Registers, sp));
  fault(operation, FaultKind::stack, Reg::x10);
  code_.bind(available);
  code_.load(Reg::x10, cpu, offsetof(Registers, sp));
  mask(Reg::x11, Reg::x10, stack_entries - 1);
  code_.shift(Shift::left, Reg::x11, Reg::x11, frame_shift, Width::x64);
  code_.alu(Alu::add, Reg::x11, cpu, Reg::x11, Width::x64);
  code_.load(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x10, Reg::x10, Reg::x17);
  code_.alu(Alu::subtract, Reg::x9, Reg::x9, Reg::x17);
  mask(Reg::x10, Reg::x10, pointer_mask, Width::w32);
  code_.store(Reg::x10, cpu, offsetof(Registers, sp));
  code_.store(Reg::x9, cpu, offsetof(Registers, sc));
}

void Emitter::return_(const BlockInstruction &operation) {
  flow_ = Flow::control;
  pop_frame(operation);
  code_.shift(Shift::right, Reg::x9, Reg::x0, word_bits, Width::x64);
  code_.move(pc, Reg::x9, Width::w32);
  if (operation.instruction.operation == Operation::return_interrupt) {
    mask(status, Reg::x0, word_mask, Width::w32);
    code_.constant(Reg::x17, 1);
    code_.store(Reg::x17, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
    boundary(NativeBoundary::owner);
  }
}
}
