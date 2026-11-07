// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::boundary(const NativeBoundary state) {
  flow_ = state == NativeBoundary::owner ? Flow::owner : Flow::context;
  code_.constant(Reg::x17, static_cast<std::uint8_t>(state));
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
}

void Emitter::control_byte(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto field = instruction.opcode & 3U;
  const unsigned shift = field == 0 || field == 3 ? 8U : 0U;
  constexpr Word byte_mask = 0xff;
  const auto value = ((instruction.opcode >> 8) & byte_mask) << shift;
  const auto mask = byte_mask << shift;
  const auto target = field >= 2 ? Reg::x0 : status;
  if (field >= 2) { code_.load(target, cpu, offsetof(Registers, omr)); }
  const bool conjunction = instruction.operation == Operation::and_control;
  code_.constant(Reg::x17, conjunction ? value | ~mask : value);
  code_.alu(conjunction ? Alu::bit_and : Alu::bit_or, target, target, Reg::x17);
  if (field >= 2) { code_.store(target, cpu, offsetof(Registers, omr)); }
  if (field == 0) { boundary(NativeBoundary::context); }
}

bool Emitter::control(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto kind = instruction.operation;
  if (kind == Operation::relative_branch) {
    branch(operation);
    return true;
  }
  const bool jumping = kind == Operation::jump_short || kind == Operation::call_short ||
                       kind == Operation::jump_effective || kind == Operation::call_effective ||
                       kind == Operation::conditional_jump;
  if (jumping) {
    jump(operation);
    return true;
  }
  if (kind == Operation::return_subroutine || kind == Operation::return_interrupt) {
    return_(operation);
    return true;
  }
  if (kind == Operation::wait || kind == Operation::stop) {
    code_.constant(Reg::x17,
                   static_cast<std::uint8_t>(kind == Operation::wait ? ExecutionState::waiting
                                                                     : ExecutionState::stopped));
    code_.store(Reg::x17, cpu, offsetof(Cpu, processing), Access::byte);
    boundary(NativeBoundary::owner);
    return true;
  }
  if (kind == Operation::and_control || kind == Operation::or_control) {
    control_byte(operation);
    return true;
  }
  if (kind == Operation::unsupported) {
    code_.constant(Reg::x0, instruction.opcode);
    fault(operation, FaultKind::unsupported_instruction, Reg::x0);
    return true;
  }
  return false;
}
}
