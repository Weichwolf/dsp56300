// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"
#include "transfer_info_support.hpp"
#include "transfers.hpp"

#include <cstdint>

namespace dsp56300::test::transfer_info_test {
namespace {
constexpr Word port_register_high = 0x084000U;
constexpr Word port_register_low_y = 0x044020U;
constexpr Word port_register_low_x = 0x044080U;
constexpr Word port_high_base = 0xffffc0U;
constexpr Word port_low_base = 0xffff80U;
constexpr Word port_program_high = 0x084040U;
constexpr Word port_memory_high = 0x084080U;
constexpr Word port_memory_low_y = 0x070080U;
constexpr Word port_memory_low_x = 0x074000U;
constexpr Word byte_ones = 0xff;
constexpr Word five_bit_mask = 31U;

void control_register(const unsigned ordinal) {
  const auto target = ordinal & five_bit_mask;
  const auto other = (ordinal >> 5) & 63U;
  const bool load = (ordinal & (1U << 11)) != 0;
  const Word opcode = 0x0440a0U | target | (other << 8) | (load ? 0x8000U : 0U);
  if (registers.at(32U + target) == RegisterId::none || registers.at(other) == RegisterId::none) {
    unsupported(opcode);
    return;
  }
  verify(opcode,
         expected(InstructionKind::move_control_register,
                  1,
                  {directed({.source = reg(registers.at(other)),
                             .destination = reg(registers.at(32U + target))},
                            load)}));
}

void control_memory(const unsigned ordinal) {
  const auto target = ordinal & five_bit_mask;
  const auto field = (ordinal >> 5) & 63U;
  const bool load = (ordinal & (1U << 11)) != 0;
  const bool y = (ordinal & (1U << 12)) != 0;
  const bool extended = (ordinal & (1U << 13)) != 0;
  const auto mode = field / 8;
  const auto index = field % 8;
  const Word opcode = 0x050020U | target | (field << 8) | (load ? 0x8000U : 0U) | (y ? 0x40U : 0U) |
                      (extended ? 0x4000U : 0U);
  if (registers.at(32U + target) == RegisterId::none ||
      (extended && mode == 6 && index != 0 && (!load || index != 4))) {
    unsupported(opcode);
    return;
  }
  const auto space = y ? TransferSpace::y : TransferSpace::x;
  auto operand = absolute(space, field);
  if (extended) {
    operand = mode == 6 && index == 4 ? extension() : ea(space, {.mode = mode, .index = index});
  }
  verify(
      opcode,
      expected(
          extended ? InstructionKind::move_control_effective : InstructionKind::move_control_short,
          extended && mode == 6 ? 2U : 1U,
          {directed({.source = operand, .destination = reg(registers.at(32U + target))}, load)}));
}

void program_case(const unsigned ordinal) {
  const auto selector = ordinal & 63U;
  const auto field = (ordinal >> 6) & 63U;
  const bool load = (ordinal & (1U << 12)) != 0;
  const bool extended = (ordinal & (1U << 13)) != 0;
  const auto mode = field / 8;
  const auto index = field % 8;
  const Word opcode =
      0x070000U | selector | (field << 8) | (load ? 0x8000U : 0U) | (extended ? 0x4080U : 0U);
  if (registers.at(selector) == RegisterId::none || (extended && mode == 6 && index != 0)) {
    unsupported(opcode);
    return;
  }
  const auto operand = extended ? ea(TransferSpace::program, {.mode = mode, .index = index})
                                : absolute(TransferSpace::program, field);
  verify(
      opcode,
      expected(extended ? InstructionKind::move_program_effective
                        : InstructionKind::move_program_short,
               extended && mode == 6 ? 2U : 1U,
               {directed({.source = operand, .destination = reg(registers.at(selector))}, load)}));
}

void port_register(const unsigned ordinal) {
  const auto address = ordinal & 63U;
  const auto selector = (ordinal >> 6) & 63U;
  const bool write = (ordinal & (1U << 12)) != 0;
  const bool y = (ordinal & (1U << 13)) != 0;
  const bool high = (ordinal & (1U << 14)) != 0;
  Word opcode = 0;
  if (high) {
    opcode = port_register_high | (y ? 0x10000U : 0U) | address;
  } else {
    opcode = (y ? port_register_low_y : port_register_low_x) | (address & five_bit_mask) |
             ((address & 32U) << 1);
  }
  opcode |= (selector << 8) | (write ? 0x8000U : 0U);
  if (registers.at(selector) == RegisterId::none) {
    unsupported(opcode);
    return;
  }
  verify(opcode,
         expected(InstructionKind::move_peripheral_register,
                  1,
                  {directed({.source = absolute(y ? TransferSpace::y : TransferSpace::x,
                                                (high ? port_high_base : port_low_base) + address),
                             .destination = reg(registers.at(selector))},
                            !write)}));
}

struct PortEncoding {
  Word address, field;
  bool write, y, port_y, high, program;
};

Word memory_port_opcode(const PortEncoding input) {
  Word opcode = 0;
  if (input.high) {
    opcode = (input.program ? port_program_high : port_memory_high) |
             (input.port_y ? 0x10000U : 0U) | (input.write ? 0x8000U : 0U) |
             (!input.program && input.y ? 0x40U : 0U);
  } else if (input.program) {
    opcode = 0x008000U | (input.write ? 0x4000U : 0U) | (input.port_y ? 0x40U : 0U);
  } else {
    opcode = (input.port_y ? port_memory_low_y : port_memory_low_x) | (input.write ? 0x8000U : 0U) |
             (input.y ? 0x40U : 0U);
  }
  return opcode | (input.field << 8) | input.address;
}

void port_memory(const unsigned ordinal) {
  const auto address = ordinal & 63U;
  const auto field = (ordinal >> 6) & 63U;
  const bool write = (ordinal & (1U << 12)) != 0;
  const bool y = (ordinal & (1U << 13)) != 0;
  const bool port_y = (ordinal & (1U << 14)) != 0;
  const bool high = (ordinal & (1U << 15)) != 0;
  const bool program = (ordinal & (1U << 16)) != 0;
  if (program && y) { return; } // P: has no encoded X/Y-data-space selector.
  const auto mode = field / 8;
  const auto index = field % 8;
  const auto opcode = memory_port_opcode({.address = address,
                                          .field = field,
                                          .write = write,
                                          .y = y,
                                          .port_y = port_y,
                                          .high = high,
                                          .program = program});
  if (mode == 6 && index != 0 && (index != 4 || !write || program)) {
    unsupported(opcode);
    return;
  }
  auto space = y ? TransferSpace::y : TransferSpace::x;
  if (program) { space = TransferSpace::program; }
  const auto operand =
      mode == 6 && index == 4 ? extension() : ea(space, {.mode = mode, .index = index});
  verify(opcode,
         expected(program ? InstructionKind::move_peripheral_program
                          : InstructionKind::move_peripheral_memory,
                  mode == 6 ? 2U : 1U,
                  {directed({.source = absolute(port_y ? TransferSpace::y : TransferSpace::x,
                                                (high ? port_high_base : port_low_base) + address),
                             .destination = operand},
                            !write)}));
}

void displaced_case(const unsigned ordinal) {
  const auto selector = ordinal & 15U;
  const auto index = (ordinal >> 4) & 7U;
  const auto raw = (ordinal >> 7) & 127U;
  const bool y = (ordinal & (1U << 14)) != 0;
  const bool load = (ordinal & (1U << 15)) != 0;
  const Word opcode = 0x020080U | ((raw >> 1) << 11) | ((raw & 1U) << 6) | (index << 8) |
                      (y ? 0x20U : 0U) | (load ? 0x10U : 0U) | selector;
  if (registers.at(selector) == RegisterId::none) {
    unsupported(opcode);
    return;
  }
  const auto operand = memory(y ? TransferSpace::y : TransferSpace::x,
                              {.kind = AddressKind::displacement,
                               .index = static_cast<std::uint8_t>(index),
                               .literal = raw < 64 ? raw : word_mask - (127U - raw)});
  verify(
      opcode,
      expected(InstructionKind::move_displacement,
               1,
               {directed({.source = operand, .destination = reg(registers.at(selector))}, load)}));
}

void long_displaced_case(const unsigned ordinal) {
  const auto selector = ordinal & 63U;
  const auto index = (ordinal >> 6) & 7U;
  const bool y = (ordinal & 512U) != 0;
  const bool load = (ordinal & 1024U) != 0;
  const Word opcode =
      0x0a7080U | (y ? 0x10000U : 0U) | (index << 8) | (load ? 0x40U : 0U) | selector;
  if (registers.at(selector) == RegisterId::none) {
    unsupported(opcode);
    return;
  }
  verify(opcode,
         expected(InstructionKind::move_displacement,
                  2,
                  {directed({.source = memory(y ? TransferSpace::y : TransferSpace::x,
                                              {.kind = AddressKind::displacement_extension,
                                               .index = static_cast<std::uint8_t>(index)}),
                             .destination = reg(registers.at(selector))},
                            load)}));
}
} // namespace

void control() {
  for (unsigned ordinal = 0; ordinal < 4096; ++ordinal) { control_register(ordinal); }
  for (unsigned ordinal = 0; ordinal < 16384; ++ordinal) { control_memory(ordinal); }
  for (unsigned target = 0; target < 32; ++target) {
    const Word opcode = 0x05ffa0U | target;
    if (registers.at(32U + target) == RegisterId::none) {
      unsupported(opcode);
    } else {
      verify(opcode,
             expected(InstructionKind::move_control_immediate,
                      1,
                      {{.source = immediate(byte_ones),
                        .destination = reg(registers.at(32U + target))}}));
    }
  }
}

void program() {
  for (unsigned ordinal = 0; ordinal < 16384; ++ordinal) { program_case(ordinal); }
}

void peripheral() {
  for (unsigned ordinal = 0; ordinal < 32768; ++ordinal) { port_register(ordinal); }
  for (unsigned ordinal = 0; ordinal < 131072; ++ordinal) { port_memory(ordinal); }
}

void displacement() {
  for (unsigned ordinal = 0; ordinal < 65536; ++ordinal) { displaced_case(ordinal); }
  for (unsigned ordinal = 0; ordinal < 2048; ++ordinal) { long_displaced_case(ordinal); }
}
} // namespace dsp56300::test::transfer_info_test
