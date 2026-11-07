// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::calculation_offset(const Word opcode) {
  const auto index = (opcode >> 8) & 7U;
  const bool displacement = (opcode & (1U << 14)) == 0;
  const auto mode = (opcode >> 11) & 3U;
  if (displacement) {
    constexpr Word upper = 0x70;
    constexpr Word short_mask = 0x7f;
    const auto raw = ((opcode >> 7) & upper) | ((opcode >> 4) & 15U);
    const auto expanded = (raw & 64U) != 0 ? raw | (word_mask ^ short_mask) : raw;
    code_.constant(Reg::x1, expanded);
  } else if (mode < 2) {
    code_.load(Reg::x1, cpu, offsetof(Registers, n) + (index * sizeof(Word)));
  } else {
    code_.constant(Reg::x1, 1);
    code_.constant(Reg::x2, 1);
    if (mode == 2) { code_.alu(Alu::subtract, Reg::x1, Reg::zr, Reg::x1, Width::x64); }
    return;
  }
  code_.extract(Reg::x9, Reg::x1, 0, word_bits, Width::x64, true);
  code_.compare(Reg::x3, Reg::zr);
  code_.select(Reg::x1, Reg::x1, Reg::x9, Condition::equal, Width::x64);
  code_.compare(displacement ? Reg::x1 : Reg::x9, Reg::zr, Width::x64);
  code_.set(Reg::x2, Condition::greater);
  if (!displacement && mode == 0) {
    code_.alu(Alu::subtract, Reg::x1, Reg::zr, Reg::x1, Width::x64);
  }
}

void Emitter::address_calculation(const BlockInstruction &operation) {
  if (!numeric_mode(operation, true)) { return; }
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto index = (opcode >> 8) & 7U;
  const bool relative = instruction.operation == Operation::load_relative_address;
  const bool displacement = !relative && (opcode & (1U << 14)) == 0;
  const auto target = displacement ? r_register_base + (opcode & 15U) : opcode & five_bit_mask;
  if (target < x0_register) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if (relative) {
    if (instruction.words == 2) {
      code_.constant(Reg::x0, instruction.extension);
    } else {
      code_.load(Reg::x0, cpu, offsetof(Registers, r) + (index * sizeof(Word)));
    }
    code_.constant(Reg::x17, operation.pc);
    code_.alu(Alu::add, Reg::x0, Reg::x0, Reg::x17);
    mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  } else {
    code_.load(Reg::x0, cpu, offsetof(Registers, r) + (index * sizeof(Word)));
    code_.load(Reg::x3, cpu, offsetof(Registers, m) + (index * sizeof(Word)));
    mask(Reg::x3, Reg::x3, modifier_mask);
    calculation_offset(opcode);
    update_address(operation);
  }
  write_register(operation, target);
}
}
