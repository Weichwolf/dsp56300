// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

namespace dsp56300::detail::a64 {
void Emitter::viterbi_shift(const BlockInstruction &operation) {
  if (!numeric_mode(operation, true)) { return; }
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  if (form.mode == 6 && form.index != 0) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  const auto source = (opcode & (1U << 16)) != 0 ? accumulator_b : accumulator_a;
  code_.extract(Reg::x0, source, word_bits + padding_bits, word_bits, Width::x64);
  code_.store(Reg::x0, Reg::sp, Frame::value);
  code_.extract(Reg::x0, source, padding_bits, word_bits, Width::x64);
  code_.shift(Shift::left, Reg::x0, Reg::x0, 1);
  if ((opcode & 16U) != 0) { code_.add(Reg::x0, Reg::x0, 1); }
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  code_.store(Reg::x0, Reg::sp, Frame::other_value);
  effective(operation, form);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  code_.load(Reg::x1, Reg::sp, Frame::value);
  memory(operation, {.space = Space::x, .write = true});
  code_.load(Reg::x0, Reg::sp, Frame::address);
  code_.load(Reg::x1, Reg::sp, Frame::other_value);
  memory(operation, {.space = Space::y, .write = true});
}
}
