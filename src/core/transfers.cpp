// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word direction_bit = Word{1} << 15;
constexpr Word y_space_bit = Word{1} << 6;
constexpr Word high_io_group = Word{1} << 19;
constexpr Word high_io_begin = 0xffffc0;

AddressForm transfer_form(const Instruction &decoded) noexcept {
  return {.mode = (decoded.opcode >> 11) & 7,
          .index = (decoded.opcode >> 8) & 7,
          .extra = decoded.extension};
}

bool valid_register(const unsigned number) noexcept {
  return (number >= x0_register && number < m_register_base + agu_registers) ||
         number == ep_register || number == vba_register || number == sc_register ||
         (number >= sz_register && number <= lc_register);
}

bool accumulator(const unsigned number) noexcept {
  return number == a_register || number == b_register;
}

struct Peripheral {
  Word address = 0;
  Space space = Space::x;
  bool write = false;
};

Peripheral peripheral(const Instruction &decoded) noexcept {
  const auto opcode = decoded.opcode;
  if ((opcode & high_io_group) != 0) {
    return {.address = high_io_begin + (opcode & six_bit_mask),
            .space = (opcode & (1U << 16)) != 0 ? Space::y : Space::x,
            .write = (opcode & direction_bit) != 0};
  }
  auto index = opcode & six_bit_mask;
  bool y = (opcode & y_space_bit) != 0;
  auto direction = direction_bit;
  if (decoded.operation == Operation::move_peripheral_register) {
    index = (opcode & five_bit_mask) | ((opcode >> 1) & 32U);
    y = (opcode & (1U << 5)) != 0;
  } else if (decoded.operation == Operation::move_peripheral_memory) {
    y = (opcode & (1U << 14)) == 0;
  } else {
    direction = Word{1} << 14;
  }
  return {.address = io_begin + index,
          .space = y ? Space::y : Space::x,
          .write = (opcode & direction) != 0};
}
}

void Core::Impl::control_move(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto control = m_register_base + (opcode & five_bit_mask);
  if (!valid_register(control)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  if (decoded.operation == Operation::move_control_immediate) {
    write_move_register(control, (opcode >> 8) & byte_mask);
    return;
  }
  const bool load = (opcode & direction_bit) != 0;
  if (decoded.operation == Operation::move_control_register) {
    control_register_move(opcode);
    return;
  }
  if (!numeric_mode_supported()) { return; }
  const auto form = transfer_form(decoded);
  const bool immediate =
      decoded.operation == Operation::move_control_effective && form.mode == 6 && form.index == 4;
  if (decoded.operation == Operation::move_control_effective && form.mode == 6 && form.index != 0 &&
      (!load || form.index != 4)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto address = decoded.operation == Operation::move_control_short
                           ? (opcode >> 8) & six_bit_mask
                           : effective(form);
  if (fault.kind != FaultKind::none) { return; }
  const auto space = (opcode & y_space_bit) != 0 ? Space::y : Space::x;
  if (load) {
    const auto value = immediate ? decoded.extension : read_memory(space, address);
    if (fault.kind == FaultKind::none) { write_move_register(control, value); }
  } else {
    const auto value = read_move_register(control);
    if (fault.kind == FaultKind::none) { write_memory(space, address, value); }
  }
}

void Core::Impl::control_register_move(const Word opcode) {
  const auto control = m_register_base + (opcode & five_bit_mask);
  const auto other = (opcode >> 8) & six_bit_mask;
  if (!valid_register(other)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  if (accumulator(other) && !numeric_mode_supported()) { return; }
  const bool load = (opcode & direction_bit) != 0;
  const auto value = read_move_register(load ? other : control);
  if (fault.kind == FaultKind::none) { write_move_register(load ? control : other, value); }
}

void Core::Impl::program_move(const Instruction &decoded) {
  if (!numeric_mode_supported()) { return; }
  const auto opcode = decoded.opcode;
  const auto reg = opcode & six_bit_mask;
  const auto form = transfer_form(decoded);
  if (!valid_register(reg) || (decoded.operation == Operation::move_program_effective &&
                               form.mode == 6 && form.index != 0)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto address = decoded.operation == Operation::move_program_short
                           ? (opcode >> 8) & six_bit_mask
                           : effective(form);
  if (fault.kind != FaultKind::none) { return; }
  const bool load = (opcode & direction_bit) != 0;
  const auto value = load ? read_memory(Space::program, address) : read_move_register(reg);
  if (fault.kind != FaultKind::none) { return; }
  if (load) {
    write_move_register(reg, value);
  } else {
    write_memory(Space::program, address, value);
  }
}

void Core::Impl::peripheral_register_move(const Instruction &decoded) {
  const auto port = peripheral(decoded);
  const auto opcode = decoded.opcode;
  const auto reg = (opcode >> 8) & six_bit_mask;
  if (!valid_register(reg)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const bool data_alu = reg >= x0_register && reg <= b_register;
  // SA Data-ALU register mapping (FM3.4.1) is not the ordinary 24-bit mapping.
  if ((accumulator(reg) || (data_alu && (cpu.reg.sr & arithmetic_16) != 0)) &&
      !numeric_mode_supported()) {
    return;
  }
  const auto value = port.write ? read_move_register(reg) : read_memory(port.space, port.address);
  if (fault.kind != FaultKind::none) { return; }
  if (port.write) {
    write_memory(port.space, port.address, value);
  } else {
    write_move_register(reg, value);
  }
}

void Core::Impl::peripheral_move(const Instruction &decoded) {
  if (decoded.operation == Operation::move_peripheral_register) {
    peripheral_register_move(decoded);
    return;
  }
  const auto port = peripheral(decoded);
  const auto opcode = decoded.opcode;
  // FM3.4.1/13-134: this path copies full bus words without a Data-ALU register.
  if (!numeric_mode_supported(arithmetic_16)) { return; }
  const auto form = transfer_form(decoded);
  const bool program = decoded.operation == Operation::move_peripheral_program;
  const bool immediate = form.mode == 6 && form.index == 4;
  if (immediate && (program || !port.write)) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto address = effective(form);
  if (fault.kind != FaultKind::none) { return; }
  auto space = (opcode & y_space_bit) != 0 ? Space::y : Space::x;
  if (program) { space = Space::program; }
  if (port.write) {
    const auto value = immediate ? decoded.extension : read_memory(space, address);
    if (fault.kind == FaultKind::none) { write_memory(port.space, port.address, value); }
  } else {
    const auto value = read_memory(port.space, port.address);
    if (fault.kind == FaultKind::none) { write_memory(space, address, value); }
  }
}
} // namespace dsp56300
