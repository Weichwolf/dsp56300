// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::paired_address(const BlockInstruction &operation,
                             const AddressForm form,
                             const std::size_t offset) {
  if (guarded()) {
    effective(operation, form);
    code_.store(Reg::x0, Reg::sp, offset);
    return;
  }
  code_.load(Reg::x0, cpu, offsetof(Registers, r) + (form.index * sizeof(Word)));
  code_.store(Reg::x0, Reg::sp, offset);
  if (form.mode == 4) { return; }
  Label resume;
  failure_ = &resume;
  effective(operation, form);
  failure_ = nullptr;
  code_.bind(resume);
}

void Emitter::paired_capture(const BlockInstruction &operation,
                             const unsigned reg,
                             const BusOperand operand) {
  const bool x = operand.space == Space::x;
  if (operand.write) {
    read_register(operation, reg);
  } else {
    code_.load(Reg::x0, Reg::sp, x ? Frame::address : Frame::other_address);
    Label resume;
    failure_ = &resume;
    memory(operation, operand);
    failure_ = nullptr;
    code_.bind(resume);
  }
  code_.store(Reg::x0, Reg::sp, x ? Frame::value : Frame::other_value);
}

void Emitter::paired_commit(const BlockInstruction &operation,
                            const unsigned reg,
                            const BusOperand operand) {
  const bool x = operand.space == Space::x;
  code_.load(Reg::x0, Reg::sp, x ? Frame::value : Frame::other_value);
  if (operand.write) {
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, x ? Frame::address : Frame::other_address);
    Label resume;
    failure_ = &resume;
    memory(operation, operand);
    failure_ = nullptr;
    code_.bind(resume);
  } else {
    write_register(operation, reg);
  }
}

void Emitter::paired_move(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const auto xmode = (opcode >> 11) & 3U;
  const auto ymode = (opcode >> 20) & 3U;
  const auto index = (opcode >> 8) & 7U;
  const auto yindex = ((opcode >> 13) & 3U) + (index < 4 ? 4U : 0U);
  paired_address(operation, {.mode = xmode == 0 ? 4U : xmode, .index = index}, Frame::address);
  paired_address(
      operation, {.mode = ymode == 0 ? 4U : ymode, .index = yindex}, Frame::other_address);
  constexpr std::array xregs{x0_register, x1_register, a_register, b_register};
  constexpr std::array yregs{y0_register, y1_register, a_register, b_register};
  const auto xreg = xregs.at((opcode >> 18) & 3U);
  const auto yreg = yregs.at((opcode >> 16) & 3U);
  const BusOperand x{.space = Space::x, .write = (opcode & (1U << 15)) == 0};
  const BusOperand y{.space = Space::y, .write = (opcode & (1U << 22)) == 0};
  paired_capture(operation, xreg, x);
  if (guarded()) {
    paired_capture(operation, yreg, y);
    paired_commit(operation, xreg, x);
    paired_commit(operation, yreg, y);
    return;
  }
  Label both;
  Label complete;
  code_.constant(Reg::x17, io_begin);
  code_.load(Reg::x9, Reg::sp, Frame::address);
  code_.compare(Reg::x9, Reg::x17);
  code_.branch_far(Condition::no_carry, both);
  code_.load(Reg::x9, Reg::sp, Frame::other_address);
  code_.compare(Reg::x9, Reg::x17);
  code_.branch_far(Condition::no_carry, both);
  paired_commit(operation, xreg, x);
  code_.branch(complete);
  code_.bind(both);
  paired_capture(operation, yreg, y);
  paired_commit(operation, xreg, x);
  paired_commit(operation, yreg, y);
  code_.bind(complete);
  code_.load(
      Reg::x17, context, offsetof(NativeContext, failure) + offsetof(Fault, kind), Access::byte);
  code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
}
}
