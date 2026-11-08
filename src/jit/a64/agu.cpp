// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::reverse_address() {
  code_.alu(Alu::subtract, Reg::x9, Reg::zr, Reg::x1, Width::x64);
  code_.compare(Reg::x1, Reg::zr, Width::x64);
  code_.select(Reg::x9, Reg::x9, Reg::x1, Condition::negative, Width::x64);
  code_.reverse_bits(Reg::x0, Reg::x0);
  code_.reverse_bits(Reg::x9, Reg::x9);
  code_.alu(Alu::add, Reg::x10, Reg::x0, Reg::x9);
  code_.alu(Alu::subtract, Reg::x11, Reg::x0, Reg::x9);
  code_.select(Reg::x0, Reg::x11, Reg::x10, Condition::negative);
  code_.reverse_bits(Reg::x0, Reg::x0);
}

void Emitter::ring_address(Label &linear) {
  Label stride;
  Label ordinary;
  Label complete;
  code_.add(Reg::x9, Reg::x3, 1);
  code_.count_leading_zeroes(Reg::x10, Reg::x3);
  code_.constant(Reg::x17, host_word_bits);
  code_.alu(Alu::subtract, Reg::x10, Reg::x17, Reg::x10);
  code_.constant(Reg::x11, 1);
  code_.shift(Shift::left, Reg::x10, Reg::x11, Reg::x10);
  code_.alu(Alu::subtract, Reg::x10, Reg::x10, Reg::x11);
  code_.alu(Alu::subtract, Reg::x11, Reg::zr, Reg::x1, Width::x64);
  code_.compare(Reg::x1, Reg::zr, Width::x64);
  code_.select(Reg::x11, Reg::x11, Reg::x1, Condition::negative, Width::x64);
  code_.compare(Reg::x11, Reg::x9, Width::x64);
  code_.branch_far(Condition::above, stride);
  code_.branch_far(Condition::not_equal, ordinary);
  code_.branch_zero_far(Reg::x2, ordinary);
  code_.bind(stride);
  code_.alu(Alu::bit_and, Reg::x11, Reg::x1, Reg::x10);
  code_.branch_zero_far(Reg::x11, linear);
  code_.bind(ordinary);
  code_.alu(Alu::bit_clear, Reg::x11, Reg::x0, Reg::x10);
  code_.alu(Alu::add, Reg::x12, Reg::x11, Reg::x9);
  code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.compare(Reg::x0, Reg::x12, Width::x64);
  Label lower;
  code_.branch_far(Condition::less, lower);
  code_.alu(Alu::subtract, Reg::x0, Reg::x0, Reg::x9, Width::x64);
  code_.branch(complete);
  code_.bind(lower);
  code_.compare(Reg::x0, Reg::x11, Width::x64);
  code_.branch_far(Condition::greater_equal, complete);
  code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x9, Width::x64);
  code_.bind(complete);
}

void Emitter::multiple_address(const BlockInstruction &operation) {
  Label invalid;
  Label complete;
  mask(Reg::x11, Reg::x3, multiple_reserved_bit);
  code_.branch_zero_far(Reg::x11, invalid, Width::w32, true);
  mask(Reg::x9, Reg::x3, multiple_wrap_mask);
  code_.branch_zero_far(Reg::x9, invalid);
  code_.add(Reg::x10, Reg::x9, 1);
  code_.alu(Alu::bit_and, Reg::x10, Reg::x10, Reg::x9);
  code_.branch_zero_far(Reg::x10, invalid, Width::w32, true);
  code_.alu(Alu::add, Reg::x11, Reg::x0, Reg::x1);
  code_.alu(Alu::bit_and, Reg::x11, Reg::x11, Reg::x9);
  code_.alu(Alu::bit_clear, Reg::x0, Reg::x0, Reg::x9);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x11);
  code_.branch(complete);
  code_.bind(invalid);
  fault(operation, FaultKind::unsupported_mode, Reg::x3);
  code_.bind(complete);
}

void Emitter::update_address(const BlockInstruction &operation) {
  Label linear;
  Label reverse;
  Label multiple;
  Label complete;
  code_.constant(Reg::x17, modifier_mask);
  code_.compare(Reg::x3, Reg::x17);
  code_.branch_far(Condition::equal, linear);
  code_.branch_zero_far(Reg::x3, reverse);
  mask(Reg::x9, Reg::x3, multiple_wrap_bit);
  code_.branch_zero_far(Reg::x9, multiple, Width::w32, true);
  ring_address(linear);
  code_.branch(complete);
  code_.bind(linear);
  code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x1);
  code_.branch(complete);
  code_.bind(reverse);
  reverse_address();
  code_.branch(complete);
  code_.bind(multiple);
  multiple_address(operation);
  code_.bind(complete);
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
}

void Emitter::effective(const BlockInstruction &operation, const AddressForm form) {
  if (guarded()) {
    guarded_effective(form);
    return;
  }
  const auto offset = form.index * sizeof(Word);
  code_.load(Reg::x0, cpu, offsetof(Registers, r) + offset);
  if (form.mode == 4) { return; }
  if (form.mode == 6) {
    code_.constant(Reg::x0, form.extra & word_mask);
    if (form.index != 0 && form.index != 4) {
      code_.constant(Reg::x0, form.index);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    }
    return;
  }
  code_.load(Reg::x3, cpu, offsetof(Registers, m) + offset);
  mask(Reg::x3, Reg::x3, modifier_mask);
  const bool register_offset = form.mode == 0 || form.mode == 1 || form.mode == 5;
  if (register_offset) {
    code_.load(Reg::x1, cpu, offsetof(Registers, n) + offset);
    code_.extract(Reg::x9, Reg::x1, 0, word_bits, Width::x64, true);
    code_.compare(Reg::x9, Reg::zr, Width::x64);
    code_.set(Reg::x2, Condition::greater);
    code_.compare(Reg::x3, Reg::zr);
    code_.select(Reg::x1, Reg::x1, Reg::x9, Condition::equal, Width::x64);
  } else {
    code_.constant(Reg::x1, 1);
    code_.constant(Reg::x2, 1);
  }
  if (form.mode == 0 || form.mode == 2 || form.mode == 7) {
    code_.alu(Alu::subtract, Reg::x1, Reg::zr, Reg::x1, Width::x64);
  }
  code_.move(Reg::x8, Reg::x0, Width::w32);
  update_address(operation);
  if (form.mode != 5) { code_.store(Reg::x0, cpu, offsetof(Registers, r) + offset); }
  if (form.mode != 5 && form.mode != 7) { code_.move(Reg::x0, Reg::x8, Width::w32); }
}
}
