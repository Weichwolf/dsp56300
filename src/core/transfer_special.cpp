// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "instruction_decoder.hpp"
#include "transfer_info.hpp"
#include "transfers.hpp"

#include <cstdint>

namespace dsp56300::detail::transfer_info {
namespace {
constexpr Word short_mask = 0x1f;
constexpr Word direction = 0x8000, space_bit = 0x40, byte_mask = 0xff, register_mask = 63;

void selected(Builder &builder, const EncodedTransfer read, const bool load) noexcept {
  builder.move(load ? read
                    : EncodedTransfer{.source = read.destination, .destination = read.source});
}

struct Port {
  TransferOperand operand;
  bool write = false;
};

Port port(const Word opcode, const InstructionKind kind) noexcept {
  constexpr Word high_group = 0x80000;
  constexpr Word high_base = 0xffffc0;
  constexpr Word low_base = 0xffff80;
  if ((opcode & high_group) != 0) {
    return {.operand =
                Builder::memory((opcode & (1U << 16)) != 0 ? TransferSpace::y : TransferSpace::x,
                                {.literal = high_base + (opcode & register_mask)}),
            .write = (opcode & direction) != 0};
  }
  auto index = opcode & register_mask;
  bool y = (opcode & space_bit) != 0;
  auto write_bit = direction;
  if (kind == InstructionKind::move_peripheral_register) {
    index = (opcode & short_mask) | ((opcode >> 1) & 32U);
    y = (opcode & 32U) != 0;
  } else if (kind == InstructionKind::move_peripheral_memory) {
    y = (opcode & (1U << 14)) == 0;
  } else {
    write_bit = 1U << 14;
  }
  return {.operand = Builder::memory(y ? TransferSpace::y : TransferSpace::x,
                                     {.literal = low_base + index}),
          .write = (opcode & write_bit) != 0};
}
} // namespace

void control(Builder &builder, const Word opcode, const InstructionKind kind) noexcept {
  const auto target = builder.reg(32U + (opcode & short_mask));
  if (kind == InstructionKind::move_control_immediate) {
    builder.move(
        {.source = Builder::literal({.value = (opcode >> 8) & byte_mask}), .destination = target});
    return;
  }
  const bool load = (opcode & direction) != 0;
  if (kind == InstructionKind::move_control_register) {
    selected(builder,
             {.source = builder.reg((opcode >> 8) & register_mask), .destination = target},
             load);
    return;
  }
  const auto space = (opcode & space_bit) != 0 ? TransferSpace::y : TransferSpace::x;
  const auto memory = kind == InstructionKind::move_control_short
                          ? Builder::memory(space, {.literal = (opcode >> 8) & register_mask})
                          : builder.effective(space, effective_encoding(opcode), load);
  selected(builder, {.source = memory, .destination = target}, load);
}

void program(Builder &builder, const Word opcode, const InstructionKind kind) noexcept {
  const auto memory =
      kind == InstructionKind::move_program_short
          ? Builder::memory(TransferSpace::program, {.literal = (opcode >> 8) & register_mask})
          : builder.effective(TransferSpace::program, effective_encoding(opcode));
  selected(builder,
           {.source = memory, .destination = builder.reg(opcode & register_mask)},
           (opcode & direction) != 0);
}

void peripheral(Builder &builder, const Word opcode, const InstructionKind kind) noexcept {
  const auto peripheral = port(opcode, kind);
  if (kind == InstructionKind::move_peripheral_register) {
    selected(
        builder,
        {.source = peripheral.operand, .destination = builder.reg((opcode >> 8) & register_mask)},
        !peripheral.write);
    return;
  }
  const bool program = kind == InstructionKind::move_peripheral_program;
  auto space = (opcode & space_bit) != 0 ? TransferSpace::y : TransferSpace::x;
  if (program) { space = TransferSpace::program; }
  const auto memory =
      builder.effective(space, effective_encoding(opcode), !program && peripheral.write);
  selected(builder, {.source = peripheral.operand, .destination = memory}, !peripheral.write);
}

void displacement(Builder &builder, const Word opcode, const bool long_form) noexcept {
  const auto selector = opcode & (long_form ? register_mask : 15U);
  Word literal = 0;
  if (!long_form) {
    constexpr Word upper = 0x7e;
    constexpr Word sign = 0x40;
    constexpr Word mask = 0x7f;
    literal = ((opcode >> 10) & upper) | ((opcode >> 6) & 1U);
    if ((literal & sign) != 0) { literal |= word_mask & ~mask; }
  }
  const auto memory = Builder::memory(
      (opcode & (1U << (long_form ? 16U : 5U))) != 0 ? TransferSpace::y : TransferSpace::x,
      {.kind = long_form ? AddressKind::displacement_extension : AddressKind::displacement,
       .index = static_cast<std::uint8_t>((opcode >> 8) & 7U),
       .literal = literal});
  selected(builder,
           {.source = memory, .destination = builder.reg(selector)},
           (opcode & (1U << (long_form ? 6U : 4U))) != 0);
}
} // namespace dsp56300::detail::transfer_info
