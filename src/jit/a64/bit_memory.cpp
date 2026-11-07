// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstddef>

namespace dsp56300::detail::a64 {
bool Emitter::bit_address(const BlockInstruction &operation, const BitOperand operand) {
  if (operand.location == BitLocation::effective) {
    if (!numeric_mode(operation, true)) { return false; }
    const AddressForm form{.mode = operand.selector >> 3,
                           .index = operand.selector & 7U,
                           .extra = operation.instruction.extension};
    if (form.mode == 6 &&
        (form.index != 0 || operation.instruction.operation != Operation::bit_memory)) {
      code_.constant(Reg::x0, operation.instruction.opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
      return false;
    }
    effective(operation, form);
  } else {
    constexpr Word high_begin = 0xffffc0;
    Word base = 0;
    if (operand.location == BitLocation::port_low) { base = io_begin; }
    if (operand.location == BitLocation::port_high) { base = high_begin; }
    code_.constant(Reg::x0, base + operand.selector);
  }
  return true;
}

void Emitter::memory_bit(const BlockInstruction &operation, const BitAccess access) {
  if (!bit_address(operation, access.operand)) { return; }
  Label peripheral;
  Label complete;
  code_.constant(Reg::x17, io_begin);
  code_.compare(Reg::x0, Reg::x17);
  code_.branch_far(Condition::carry, peripheral);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  memory(operation, {.space = access.operand.space});
  modify_bit(access.action, access.bit);
  if (access.action != BitAction::test) {
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = access.operand.space, .write = true});
  }
  code_.load(Reg::x0, Reg::sp, Frame::secondary);
  code_.branch(complete);
  code_.bind(peripheral);
  constexpr std::array actions{
      BusAction::bit_test, BusAction::bit_clear, BusAction::bit_set, BusAction::bit_change};
  code_.constant(Reg::x1, access.bit);
  owned_access(operation,
               {.space = access.operand.space, .write = true, .bit = true},
               actions.at(static_cast<std::size_t>(access.action)));
  code_.bind(complete);
}
}
