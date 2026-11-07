// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
void Emitter::unary(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode & byte_mask;
  const auto kind = opcode & alu_mask;
  const bool destination_b = (opcode & 8U) != 0;
  if (opcode == 0) { return; }
  if (kind == alu_transfer || kind == alu_x0 || kind == alu_y0 || kind == alu_x1 ||
      kind == alu_y1) {
    source(Reg::x0, (opcode >> 4) & 7U, destination_b);
    code_.move(destination(opcode), Reg::x0);
    return;
  }
  if (kind == alu_complement) {
    logic(operation);
    return;
  }
  code_.load(Reg::x0, Reg::sp, destination_b ? sizeof(std::uint64_t) : 0, Access::doubleword);
  code_.constant(Reg::x3, 0);
  switch (kind) {
    case alu_clear: code_.constant(Reg::x0, 0); break;
    case alu_test:
    case alu_round: break;
    case alu_absolute:
    case alu_negate:
      code_.move(Reg::x1, Reg::x0);
      code_.alu(Alu::subtract_flags, Reg::x0, Reg::zr, Reg::x0, Width::x64);
      code_.set(Reg::x3, Condition::overflow);
      if (kind == alu_absolute) {
        code_.compare(Reg::x1, Reg::zr, Width::x64);
        code_.select(Reg::x0, Reg::x0, Reg::x1, Condition::negative, Width::x64);
      }
      break;
    default: throw std::invalid_argument("A64 DSP unary lowering is incomplete");
  }
  complete(operation, kind == alu_test, false, kind == alu_round);
}

void Emitter::logic(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto kind = opcode & 7U;
  const bool immediate = instruction.operation == Operation::immediate_alu;
  code_.load(Reg::x0, Reg::sp, (opcode & 8U) != 0 ? sizeof(std::uint64_t) : 0, Access::doubleword);
  code_.extract(Reg::x0, Reg::x0, main_shift, word_bits, Width::x64);
  if (!immediate && (opcode & alu_mask) == alu_complement) {
    code_.constant(Reg::x17, word_mask);
    code_.alu(Alu::bit_xor, Reg::x0, Reg::x0, Reg::x17);
  } else {
    if (immediate) {
      code_.constant(Reg::x1,
                     instruction.words == 2 ? instruction.extension : (opcode >> 8) & six_bit_mask);
    } else {
      const auto selector = (opcode >> 4) & 7U;
      source(Reg::x1, selector, (opcode & 8U) != 0);
      code_.extract(Reg::x1, Reg::x1, main_shift, word_bits, Width::x64);
    }
    Alu function = Alu::bit_and;
    if (kind == 2) {
      function = Alu::bit_or;
    } else if (kind == 3) {
      function = Alu::bit_xor;
    }
    code_.alu(function, Reg::x0, Reg::x0, Reg::x1);
  }
  logical_flags(Reg::x0);
  const auto target = destination(opcode);
  constexpr auto middle_mask = std::uint64_t{word_mask} << main_shift;
  mask(target, target, ~middle_mask);
  code_.shift(Shift::left, Reg::x0, Reg::x0, main_shift, Width::x64);
  code_.alu(Alu::bit_or, target, target, Reg::x0, Width::x64);
}
}
