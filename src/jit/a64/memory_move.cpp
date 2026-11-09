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
void Emitter::class_two_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  if (form.mode == 6) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  effective(operation, form);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  const bool y = (opcode & (1U << 15)) != 0;
  const auto target = (opcode & (1U << 16)) != 0 ? b_register : a_register;
  read_register(operation, target);
  code_.store(Reg::x0, Reg::sp, Frame::value);
  read_register(operation, y ? y0_register : x0_register);
  code_.store(Reg::x0, Reg::sp, Frame::secondary);
  code_.load(Reg::x1, Reg::sp, Frame::value);
  code_.load(Reg::x0, Reg::sp, Frame::address);
  memory(operation, {.space = y ? Space::y : Space::x, .write = true});
  code_.load(Reg::x0, Reg::sp, Frame::secondary);
  write_register(operation, target);
}

void Emitter::memory_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto target = ((opcode >> 16) & 7U) | ((opcode >> 17) & high_register_mask);
  const bool extended = (opcode & (1U << 14)) != 0;
  const bool load = (opcode & (1U << 15)) != 0;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  if (extended) {
    effective(operation, form);
  } else {
    code_.constant(Reg::x0, (opcode >> 8) & six_bit_mask);
  }
  code_.store(Reg::x0, Reg::sp, Frame::address);
  const auto space = (opcode & (1U << 19)) != 0 ? Space::y : Space::x;
  if (load) {
    if (extended && form.mode == 6 && form.index == 4) {
      code_.constant(Reg::x0, instruction.extension & word_mask);
    } else {
      memory(operation, {.space = space});
    }
    write_register(operation, target);
  } else {
    read_register(operation, target);
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = space, .write = true});
  }
}

void Emitter::memory_transfer(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto kind = instruction.operation;
  const bool control =
      kind == Operation::move_control_effective || kind == Operation::move_control_short;
  const auto target = control ? m_register_base + (opcode & five_bit_mask) : opcode & six_bit_mask;
  if (control && !valid_register(target)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if (!numeric_mode(operation, true)) { return; }
  const bool load = (opcode & (1U << 15)) != 0;
  const bool extended =
      kind == Operation::move_control_effective || kind == Operation::move_program_effective;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  const bool immediate = control && extended && form.mode == 6 && form.index == 4 && load;
  if (!valid_register(target) || (extended && form.mode == 6 && form.index != 0 && !immediate)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if (extended) {
    effective(operation, form);
  } else {
    code_.constant(Reg::x0, (opcode >> 8) & six_bit_mask);
  }
  code_.store(Reg::x0, Reg::sp, Frame::address);
  auto space = Space::program;
  if (control) { space = (opcode & (1U << 6)) != 0 ? Space::y : Space::x; }
  if (load) {
    if (immediate) {
      code_.constant(Reg::x0, instruction.extension & word_mask);
    } else {
      memory(operation, {.space = space});
    }
    write_register(operation, target);
  } else {
    read_register(operation, target);
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = space, .write = true});
  }
}

void Emitter::displacement_move(const BlockInstruction &operation) {
  if (!numeric_mode(operation, true)) { return; }
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const bool long_form = instruction.words == 2;
  const auto target = opcode & (long_form ? six_bit_mask : Word{15});
  if (!valid_register(target)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  const auto index = (opcode >> 8) & 7U;
  Word displacement = instruction.extension;
  if (!long_form) {
    constexpr Word upper = 0x7e;
    constexpr Word sign = 0x40;
    constexpr Word short_mask = 0x7f;
    displacement = ((opcode >> 10) & upper) | ((opcode >> 6) & 1U);
    if ((displacement & sign) != 0) { displacement |= word_mask ^ short_mask; }
  }
  const auto offset = index * sizeof(Word);
  code_.load(Reg::x0, cpu, offsetof(Registers, r) + offset);
  code_.load(Reg::x3, cpu, offsetof(Registers, m) + offset);
  mask(Reg::x3, Reg::x3, modifier_mask);
  code_.constant(Reg::x1, displacement);
  code_.extract(Reg::x9, Reg::x1, 0, word_bits, Width::x64, true);
  code_.compare(Reg::x9, Reg::zr, Width::x64);
  code_.set(Reg::x2, Condition::greater);
  code_.compare(Reg::x3, Reg::zr);
  code_.select(Reg::x1, Reg::x1, Reg::x9, Condition::equal, Width::x64);
  update_address(operation);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  const auto space = ((opcode >> (long_form ? 16U : 5U)) & 1U) != 0 ? Space::y : Space::x;
  const bool load = ((opcode >> (long_form ? 6U : 4U)) & 1U) != 0;
  if (load) {
    memory(operation, {.space = space});
    write_register(operation, target);
  } else {
    read_register(operation, target);
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = space, .write = true});
  }
}
}
