// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

namespace dsp56300::detail::a64 {
namespace {
struct Port {
  Word address;
  Space space;
  bool write;
};

Port port(const Instruction &instruction) {
  const auto opcode = instruction.opcode;
  constexpr Word high_group = Word{1} << 19;
  if ((opcode & high_group) != 0) {
    constexpr Word high_io = 0xffffc0;
    return {.address = high_io + (opcode & six_bit_mask),
            .space = (opcode & (1U << 16)) != 0 ? Space::y : Space::x,
            .write = (opcode & (1U << 15)) != 0};
  }
  auto index = opcode & six_bit_mask;
  bool y = (opcode & (1U << 6)) != 0;
  unsigned direction = 15;
  if (instruction.operation == Operation::move_peripheral_register) {
    index = (opcode & five_bit_mask) | ((opcode >> 1) & 32U);
    y = (opcode & (1U << 5)) != 0;
  } else if (instruction.operation == Operation::move_peripheral_memory) {
    y = (opcode & (1U << 14)) == 0;
  } else {
    direction = 14;
  }
  return {.address = io_begin + index,
          .space = y ? Space::y : Space::x,
          .write = (opcode & (1U << direction)) != 0};
}
}

void Emitter::peripheral_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto device = port(instruction);
  if (instruction.operation == Operation::move_peripheral_register) {
    const auto target = (opcode >> 8) & six_bit_mask;
    if (!valid_register(target)) {
      code_.constant(Reg::x0, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
      return;
    }
    const bool accumulator = target == a_register || target == b_register;
    const bool data_alu = target >= x0_register && target <= b_register;
    if ((accumulator || (data_alu && (operation.mode & arithmetic_16) != 0)) &&
        !numeric_mode(operation, true)) {
      return;
    }
    if (device.write) {
      read_register(operation, target);
      code_.move(Reg::x1, Reg::x0, Width::w32);
      code_.constant(Reg::x0, device.address);
      memory(operation, {.space = device.space, .write = true});
    } else {
      code_.constant(Reg::x0, device.address);
      memory(operation, {.space = device.space});
      write_register(operation, target);
    }
    return;
  }
  if (!numeric_mode(operation, arithmetic_16 | arithmetic_saturation)) { return; }
  const bool program = instruction.operation == Operation::move_peripheral_program;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  const bool immediate = form.mode == 6 && form.index == 4;
  if (immediate && (program || !device.write)) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  effective(operation, form);
  code_.store(Reg::x0, Reg::sp, Frame::address);
  auto space = (opcode & (1U << 6)) != 0 ? Space::y : Space::x;
  if (program) { space = Space::program; }
  if (device.write) {
    if (immediate) {
      code_.constant(Reg::x0, instruction.extension);
    } else {
      memory(operation, {.space = space});
    }
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.constant(Reg::x0, device.address);
    memory(operation, {.space = device.space, .write = true});
  } else {
    code_.constant(Reg::x0, device.address);
    memory(operation, {.space = device.space});
    code_.move(Reg::x1, Reg::x0, Width::w32);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = space, .write = true});
  }
}
}
