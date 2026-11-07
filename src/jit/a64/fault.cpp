// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::fault(const BlockInstruction &operation, const FaultKind kind, const Reg value) {
  constexpr auto failure = offsetof(NativeContext, failure);
  code_.constant(pc, (operation.pc + operation.instruction.words) & word_mask);
  code_.store(value, context, failure + offsetof(Fault, value));
  code_.constant(Reg::x17, static_cast<std::uint8_t>(kind));
  code_.store(Reg::x17, context, failure + offsetof(Fault, kind), Access::byte);
  code_.constant(Reg::x17, static_cast<std::uint8_t>(Space::program));
  code_.store(Reg::x17, context, failure + offsetof(Fault, space), Access::byte);
  code_.constant(Reg::x17, operation.pc);
  code_.store(Reg::x17, context, failure + offsetof(Fault, address));
  code_.store(Reg::x17, context, failure + offsetof(Fault, pc));
  code_.constant(Reg::x17, static_cast<std::uint8_t>(NativeBoundary::owner));
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
  code_.branch(done_);
}
}
