// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::finish_repeat(const BlockInstruction &operation) {
  code_.constant(Reg::x0, operation.instruction.opcode);
  code_.store(Reg::x0, cpu, offsetof(Cpu, repeat_opcode));
  code_.constant(Reg::x17, 1);
  code_.store(Reg::x17, cpu, offsetof(Cpu, repeat_loaded), Access::byte);
  code_.load(Reg::x0, cpu, offsetof(Cpu, repeat_remaining));
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x0, Reg::x0, Reg::x17);
  code_.store(Reg::x0, cpu, offsetof(Cpu, repeat_remaining));
  Label completed;
  code_.branch_zero_far(Reg::x0, completed);
  code_.store(Reg::x0, cpu, offsetof(Registers, lc));
  code_.constant(pc, operation.pc);
  retire(operation);
  code_.load(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
  code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
  code_.branch(repeat_header_);
  code_.bind(completed);
  code_.load(Reg::x0, cpu, offsetof(Cpu, repeat_lc));
  code_.store(Reg::x0, cpu, offsetof(Registers, lc));
  code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_loaded), Access::byte);
  code_.constant(Reg::x17, 1);
  code_.store(Reg::x17, cpu, offsetof(Cpu, repeat_completed), Access::byte);
  finish_loop(operation);
  retire(operation);
  code_.branch(done_);
}
}
