// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <bit>
#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::guarded_effective(const AddressForm form) {
  if (form.mode == 6) {
    code_.constant(Reg::x0, form.extra & word_mask);
    return;
  }
  const auto offset = form.index * sizeof(Word);
  code_.load(Reg::x0, cpu, offsetof(Registers, r) + offset);
  if (form.mode == 4) { return; }
  if (form.mode == 1) {
    code_.load(Reg::x9, cpu, offsetof(Registers, n) + offset);
    code_.alu(Alu::add, Reg::x9, Reg::x0, Reg::x9);
  } else if (form.mode == 2 || form.mode == 7) {
    code_.subtract(Reg::x9, Reg::x0, 1);
  } else {
    code_.add(Reg::x9, Reg::x0, 1);
    const auto modifier = region_.addresses.at(form.index).modifier;
    if (modifier != modifier_mask) {
      const auto length = modifier + 1;
      const auto span = std::bit_ceil(length) - 1;
      mask(Reg::x10, Reg::x0, word_mask & ~span);
      code_.constant(Reg::x17, length);
      code_.alu(Alu::add, Reg::x10, Reg::x10, Reg::x17);
      code_.compare(Reg::x9, Reg::x10);
      code_.alu(Alu::subtract, Reg::x11, Reg::x9, Reg::x17);
      code_.select(Reg::x9, Reg::x11, Reg::x9, Condition::carry);
    }
  }
  mask(Reg::x9, Reg::x9, word_mask, Width::w32);
  code_.store(Reg::x9, cpu, offsetof(Registers, r) + offset);
  if (form.mode == 7) { code_.move(Reg::x0, Reg::x9); }
}

void Emitter::guarded_memory(const BusOperand operand) {
  const auto base = operand.space == Space::x ? Reg::x14 : Reg::x15;
  code_.shift(Shift::left, Reg::x12, Reg::x0, 2, Width::x64);
  code_.alu(Alu::add, Reg::x12, base, Reg::x12, Width::x64);
  if (operand.write) {
    code_.store(Reg::x1, Reg::x12, 0);
  } else {
    code_.load(Reg::x0, Reg::x12, 0);
    mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  }
}
}
