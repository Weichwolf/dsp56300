// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "vector_instruction.hpp"
#include "vector_state.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
namespace {
constexpr auto vector_pointer = offsetof(NativeContext, events) + offsetof(NativeEvents, vector);

bool call(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  return kind == Operation::call_short || kind == Operation::call_effective;
}
}

bool Emitter::prepare_vector(const BlockInstruction &operation) {
  code_.load(Reg::x15, context, vector_pointer, Access::doubleword);
  code_.load(Reg::x0, Reg::x15, offsetof(VectorState, phase), Access::byte);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(VectorPhase::pending));
  code_.compare(Reg::x0, Reg::x17);
  Label admitted;
  code_.branch_far(Condition::not_equal, admitted);
  code_.load(pc, Reg::x15, offsetof(VectorState, resume));
  code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_completed), Access::byte);
  code_.constant(Reg::x0, static_cast<std::uint8_t>(VectorPhase::first));
  code_.store(Reg::x0, Reg::x15, offsetof(VectorState, phase), Access::byte);
  code_.bind(admitted);
  if (operation.instruction.words == 2) {
    Label first;
    code_.load(Reg::x0, Reg::x15, offsetof(VectorState, phase), Access::byte);
    code_.constant(Reg::x17, static_cast<std::uint8_t>(VectorPhase::second));
    code_.compare(Reg::x0, Reg::x17);
    code_.branch_far(Condition::not_equal, first);
    code_.constant(Reg::x0, operation.instruction.opcode);
    fault(operation, FaultKind::unsupported_instruction, Reg::x0);
    code_.bind(first);
  }
  if (!call(operation) && !permitted_vector_instruction(operation.instruction)) {
    code_.constant(Reg::x0, operation.instruction.opcode);
    fault(operation, FaultKind::unsupported_instruction, Reg::x0);
    return false;
  }
  return true;
}

void Emitter::call_vector(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::call_short) {
    constexpr Word short_mask = 0xfff;
    code_.constant(Reg::x0, instruction.opcode & short_mask);
  } else {
    const AddressForm form{.mode = (instruction.opcode >> 11) & 7U,
                           .index = (instruction.opcode >> 8) & 7U,
                           .extra = instruction.extension};
    if (form.mode == 6 && form.index != 0) {
      code_.constant(Reg::x0, instruction.opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    } else {
      effective(operation, form);
    }
  }
  code_.store(Reg::x0, Reg::sp, Frame::address);
  code_.load(Reg::x15, context, vector_pointer, Access::doubleword);
  code_.load(Reg::x0, Reg::x15, offsetof(VectorState, resume));
  code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, status, Width::x64);
  push_frame(operation);
  code_.load(Reg::x0, Reg::x15, offsetof(VectorState, level));
  code_.add(Reg::x1, Reg::x0, 1);
  constexpr unsigned maximum_level = 3;
  constexpr unsigned priority_shift = 8;
  code_.constant(Reg::x17, maximum_level);
  code_.compare(Reg::x0, Reg::x17);
  code_.select(Reg::x0, Reg::x0, Reg::x1, Condition::equal);
  code_.shift(Shift::left, Reg::x0, Reg::x0, priority_shift);
  constexpr Word priority_mask = 0x300;
  constexpr Word cleared =
      scale_up | scale_down | arithmetic_16 | loop_flag | forever_flag | priority_mask;
  mask(status, status, word_mask ^ cleared, Width::w32);
  code_.alu(Alu::bit_or, status, status, Reg::x0);
  code_.load(pc, Reg::sp, Frame::address);
  code_.store(Reg::zr, Reg::x15, offsetof(VectorState, phase), Access::byte);
}

void Emitter::vector_operation(const BlockInstruction &operation) {
  if (call(operation)) {
    call_vector(operation);
  } else {
    this->operation(operation);
  }
}

void Emitter::finish_vector(const BlockInstruction &operation) {
  if (!call(operation) && operation.instruction.operation != Operation::undefined) {
    code_.load(Reg::x15, context, vector_pointer, Access::doubleword);
    Label final;
    Label complete;
    if (operation.instruction.words == 1) {
      code_.load(Reg::x0, Reg::x15, offsetof(VectorState, phase), Access::byte);
      code_.constant(Reg::x17, static_cast<std::uint8_t>(VectorPhase::second));
      code_.compare(Reg::x0, Reg::x17);
      code_.branch_far(Condition::equal, final);
      code_.store(Reg::x17, Reg::x15, offsetof(VectorState, phase), Access::byte);
      code_.branch(complete);
    }
    code_.bind(final);
    code_.store(Reg::zr, Reg::x15, offsetof(VectorState, phase), Access::byte);
    code_.load(pc, Reg::x15, offsetof(VectorState, resume));
    code_.constant(Reg::x17, 1);
    code_.store(Reg::x17, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
    code_.bind(complete);
  }
  boundary(NativeBoundary::owner);
}
}
