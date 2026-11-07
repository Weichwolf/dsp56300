// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::bus_failure() {
  static_assert(sizeof(Fault) % sizeof(Word) == 0);
  constexpr auto source = Frame::access + offsetof(NativeAccess, failure);
  constexpr auto target = offsetof(NativeContext, failure);
  Label recorded;
  if (failure_ != nullptr) {
    code_.load(Reg::x9, context, target + offsetof(Fault, kind), Access::byte);
    code_.branch_zero_far(Reg::x9, recorded, Width::w32, true);
  }
  for (std::size_t offset = 0; offset < sizeof(Fault); offset += sizeof(Word)) {
    code_.load(Reg::x9, Reg::sp, source + offset);
    code_.store(Reg::x9, context, target + offset);
  }
  code_.bind(recorded);
  code_.branch(failure_ == nullptr ? done_ : *failure_);
}

void Emitter::memory_fault(const BlockInstruction &operation, const BusOperand operand) {
  constexpr auto failure = offsetof(NativeContext, failure);
  Label recorded;
  if (failure_ != nullptr) {
    code_.load(Reg::x17, context, failure + offsetof(Fault, kind), Access::byte);
    code_.branch_zero_far(Reg::x17, recorded, Width::w32, true);
  }
  code_.store(Reg::x0, context, failure + offsetof(Fault, address));
  code_.store(operand.write ? Reg::x1 : Reg::zr, context, failure + offsetof(Fault, value));
  code_.constant(Reg::x17, operation.pc);
  code_.store(Reg::x17, context, failure + offsetof(Fault, pc));
  code_.constant(Reg::x17, static_cast<std::uint8_t>(operand.space));
  code_.store(Reg::x17, context, failure + offsetof(Fault, space), Access::byte);
  auto kind = operand.write ? FaultKind::memory_write : FaultKind::memory_read;
  if (operand.bit) { kind = FaultKind::io_bit; }
  code_.constant(Reg::x9, static_cast<std::uint8_t>(kind));
  if (!operand.bit && operand.space != Space::program) {
    const auto peripheral = operand.write ? FaultKind::io_write : FaultKind::io_read;
    code_.constant(Reg::x10, static_cast<std::uint8_t>(peripheral));
    code_.constant(Reg::x17, io_begin);
    code_.compare(Reg::x0, Reg::x17);
    code_.select(Reg::x9, Reg::x10, Reg::x9, Condition::carry);
  }
  code_.store(Reg::x9, context, failure + offsetof(Fault, kind), Access::byte);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(NativeBoundary::owner));
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
  code_.bind(recorded);
  if (!operand.write) { code_.constant(Reg::x0, 0); }
  code_.branch(failure_ == nullptr ? done_ : *failure_);
}

void Emitter::memory_owned(const BlockInstruction &operation, const BusOperand operand) {
  owned_access(operation, operand, operand.write ? BusAction::write : BusAction::read);
}

void Emitter::owned_access(const BlockInstruction &operation,
                           const BusOperand operand,
                           const BusAction action) {
  constexpr auto request = Frame::access;
  constexpr auto failure = request + offsetof(NativeAccess, failure);
  for (std::size_t offset = 0; offset < sizeof(Fault); offset += sizeof(Word)) {
    if (failure_ == nullptr) {
      code_.store(Reg::zr, Reg::sp, failure + offset);
    } else {
      code_.load(Reg::x9, context, offsetof(NativeContext, failure) + offset);
      code_.store(Reg::x9, Reg::sp, failure + offset);
    }
  }
  code_.store(Reg::x0, Reg::sp, request + offsetof(NativeAccess, address));
  code_.store(operand.write ? Reg::x1 : Reg::zr, Reg::sp, request + offsetof(NativeAccess, value));
  code_.constant(Reg::x17, operation.pc);
  code_.store(Reg::x17, Reg::sp, request + offsetof(NativeAccess, pc));
  code_.constant(Reg::x17, static_cast<std::uint8_t>(operand.space));
  code_.store(Reg::x17, Reg::sp, request + offsetof(NativeAccess, space), Access::byte);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(action));
  code_.store(Reg::x17, Reg::sp, request + offsetof(NativeAccess, action), Access::byte);
  code_.load(Reg::x16,
             context,
             offsetof(NativeContext, bus) + offsetof(NativeBus, access),
             Access::doubleword);
  Label available;
  code_.branch_zero_far(Reg::x16, available, Width::x64, true);
  memory_fault(operation, operand);
  code_.bind(available);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(NativeBoundary::owner));
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
  publish();
  code_.load(Reg::x0,
             context,
             offsetof(NativeContext, bus) + offsetof(NativeBus, owner),
             Access::doubleword);
  code_.add(Reg::x1, Reg::sp, request, Width::x64);
  code_.call(Reg::x16);
  if (operand.bit || !operand.write) {
    code_.load(Reg::x0, Reg::sp, request + offsetof(NativeAccess, value));
    mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  }
  code_.load(Reg::x9, Reg::sp, failure + offsetof(Fault, kind), Access::byte);
  Label accepted;
  code_.branch_zero_far(Reg::x9, accepted);
  bus_failure();
  code_.bind(accepted);
}
}
