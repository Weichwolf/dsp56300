// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "vector_state.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::debug_request() {
  code_.load(Reg::x0, cpu, offsetof(Cpu, debug_flags), Access::byte);
  mask(Reg::x1, Reg::x0, debug_interrupt_mode, Width::w32);
  Label interrupt;
  Label complete;
  code_.branch_zero_far(Reg::x1, interrupt, Width::w32, true);
  code_.constant(Reg::x17, debug_software_occurrence);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x17);
  code_.store(Reg::x0, cpu, offsetof(Cpu, debug_flags), Access::byte);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(ExecutionState::debugging));
  code_.store(Reg::x17, cpu, offsetof(Cpu, processing), Access::byte);
  code_.branch(complete);
  code_.bind(interrupt);
  mask(Reg::x1, Reg::x0, debug_interrupt_pending, Width::w32);
  Label pending;
  code_.branch_zero_far(Reg::x1, pending, Width::w32, true);
  code_.load(Reg::x1, cpu, offsetof(Registers, vba));
  constexpr Word bank = 0xffff00;
  constexpr Word offset = 6;
  mask(Reg::x1, Reg::x1, bank, Width::w32);
  code_.add(Reg::x1, Reg::x1, offset);
  code_.load(Reg::x2,
             context,
             offsetof(NativeContext, events) + offsetof(NativeEvents, debug_vector),
             Access::doubleword);
  code_.store(Reg::x1, Reg::x2, 0);
  code_.constant(Reg::x17, debug_interrupt_pending);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x17);
  code_.store(Reg::x0, cpu, offsetof(Cpu, debug_flags), Access::byte);
  code_.bind(pending);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(ExecutionState::running));
  code_.store(Reg::x17, cpu, offsetof(Cpu, processing), Access::byte);
  code_.bind(complete);
}

void Emitter::reset_peripherals(const BlockInstruction &operation) {
  boundary(NativeBoundary::owner);
  publish();
  code_.load(Reg::x0,
             context,
             offsetof(NativeContext, events) + offsetof(NativeEvents, io),
             Access::doubleword);
  code_.constant(Reg::x1, operation.pc);
  code_.constant(Reg::x16, reinterpret_cast<std::uintptr_t>(&reset_io));
  code_.call(Reg::x16);
  mask(Reg::x0, Reg::x0, 1, Width::w32);
  Label accepted;
  code_.branch_zero_far(Reg::x0, accepted, Width::w32, true);
  code_.constant(Reg::x0, operation.instruction.opcode);
  fault(operation, FaultKind::host_event, Reg::x0);
  code_.bind(accepted);
}

void Emitter::exception_request(const BlockInstruction &operation) {
  code_.load(Reg::x2,
             context,
             offsetof(NativeContext, events) + offsetof(NativeEvents, vector),
             Access::doubleword);
  code_.load(Reg::x0, Reg::x2, offsetof(VectorState, phase), Access::byte);
  Label pending;
  code_.constant(Reg::x17, static_cast<std::uint8_t>(VectorPhase::pending));
  code_.compare(Reg::x0, Reg::x17);
  code_.branch_far(Condition::equal, pending);
  Label retained;
  code_.branch_zero_far(Reg::x0, retained, Width::w32, true);
  const bool illegal = operation.instruction.operation == Operation::illegal;
  const bool instruction_fault = illegal || operation.instruction.operation == Operation::undefined;
  code_.constant(Reg::x0, (operation.pc + (illegal ? 2U : 1U)) & word_mask);
  code_.store(Reg::x0, Reg::x2, offsetof(VectorState, resume));
  code_.bind(retained);
  code_.load(Reg::x0, cpu, offsetof(Registers, vba));
  constexpr Word bank = 0xffff00;
  constexpr Word illegal_offset = 4;
  constexpr Word trap_offset = 8;
  mask(Reg::x0, Reg::x0, bank, Width::w32);
  code_.add(Reg::x0, Reg::x0, instruction_fault ? illegal_offset : trap_offset);
  code_.store(Reg::x0, Reg::x2, offsetof(VectorState, address));
  constexpr unsigned maximum_level = 3;
  code_.constant(Reg::x17, maximum_level);
  code_.store(Reg::x17, Reg::x2, offsetof(VectorState, level));
  code_.constant(Reg::x17, static_cast<std::uint8_t>(VectorPhase::pending));
  code_.store(Reg::x17, Reg::x2, offsetof(VectorState, phase), Access::byte);
  code_.store(Reg::zr, Reg::x2, offsetof(VectorState, words), Access::doubleword);
  code_.store(Reg::zr, Reg::x2, offsetof(VectorState, loaded), Access::byte);
  code_.bind(pending);
}

bool Emitter::event(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  if (kind == Operation::reset_peripherals) {
    reset_peripherals(operation);
    return true;
  }
  const bool debug = kind == Operation::debug || kind == Operation::conditional_debug;
  if (!debug && kind != Operation::illegal && kind != Operation::undefined &&
      kind != Operation::trap && kind != Operation::conditional_trap) {
    return false;
  }
  Label complete;
  if (kind == Operation::conditional_debug || kind == Operation::conditional_trap) {
    condition(Reg::x7, operation.instruction.opcode & 15U);
    code_.branch_zero_far(Reg::x7, complete);
  }
  if (debug) {
    debug_request();
  } else {
    exception_request(operation);
  }
  code_.constant(Reg::x17, static_cast<std::uint8_t>(NativeBoundary::owner));
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
  code_.bind(complete);
  return true;
}
}
