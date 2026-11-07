// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::memory_windows(const BusOperand operand, const MemoryLabels labels) {
  constexpr auto windows = offsetof(NativeContext, bus) + offsetof(NativeBus, windows);
  const auto bank = (operand.space == Space::x ? 0U : 1U) * sizeof(NativeWindows);
  code_.load(Reg::x15, context, windows + bank + offsetof(NativeWindows, data), Access::doubleword);
  code_.load(
      Reg::x16, context, windows + bank + offsetof(NativeWindows, count), Access::doubleword);
  Label search;
  Label next;
  Label ready;
  code_.bind(search);
  code_.branch_zero_far(Reg::x16, labels.unmapped, Width::x64);
  code_.load(Reg::x9, Reg::x15, offsetof(NativeWindow, address));
  code_.alu(Alu::subtract, Reg::x11, Reg::x0, Reg::x9);
  code_.load(Reg::x10, Reg::x15, offsetof(NativeWindow, words), Access::doubleword);
  code_.compare(Reg::x11, Reg::x10, Width::x64);
  code_.branch_far(Condition::carry, next);
  if (operand.write) {
    code_.load(Reg::x9, Reg::x15, offsetof(NativeWindow, writable), Access::byte);
    code_.branch_zero_far(Reg::x9, labels.unmapped);
    code_.load(Reg::x9, Reg::x15, offsetof(NativeWindow, aliases_program), Access::byte);
    code_.branch_zero_far(Reg::x9, labels.unmapped, Width::w32, true);
  }
  code_.load(Reg::x13, Reg::x15, offsetof(NativeWindow, known), Access::doubleword);
  code_.branch_zero_far(Reg::x13, ready, Width::x64);
  code_.alu(Alu::add, Reg::x13, Reg::x13, Reg::x11, Width::x64);
  if (!operand.write) {
    code_.load(Reg::x9, Reg::x13, 0, Access::byte);
    code_.branch_zero_far(Reg::x9, labels.unmapped);
  }
  code_.bind(ready);
  code_.load(Reg::x12, Reg::x15, offsetof(NativeWindow, data), Access::doubleword);
  code_.shift(Shift::left, Reg::x11, Reg::x11, 2, Width::x64);
  code_.alu(Alu::add, Reg::x12, Reg::x12, Reg::x11, Width::x64);
  code_.constant(Reg::x14, 0);
  code_.branch(labels.mapped);
  code_.bind(next);
  code_.add(Reg::x15, Reg::x15, sizeof(NativeWindow), Width::x64);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x16, Reg::x16, Reg::x17, Width::x64);
  code_.branch(search);
}
}
