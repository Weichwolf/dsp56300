// SPDX-License-Identifier: GPL-3.0-only
#include "pair_read_loop.hpp"

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
#include "transfers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace dsp56300::detail {
namespace {
std::optional<EncodedTransfer> transfer(const Instruction &instruction) noexcept {
  const auto shape = inspect_transfers(instruction.opcode);
  if (!shape || shape->count != 1 || shape->address_update || shape->parallel_alu != 0) {
    return {};
  }
  return shape->moves.front();
}

std::optional<PortAddress> input(const Instruction &instruction) noexcept {
  if (instruction.operation != Operation::move_peripheral_register) { return {}; }
  const auto move = transfer(instruction);
  if (!move || move->destination.reg != RegisterId::a1 ||
      move->source.kind != TransferOperandKind::memory ||
      move->source.address.kind != AddressKind::absolute ||
      move->source.address.literal < io_begin) {
    return {};
  }
  if (move->source.space != TransferSpace::x && move->source.space != TransferSpace::y) {
    return {};
  }
  return PortAddress{.space = move->source.space == TransferSpace::x ? Space::x : Space::y,
                     .address = move->source.address.literal};
}

std::optional<PortAddress>
edge_port(const Instruction &instruction, const BitAction action, const Word bit) noexcept {
  if (instruction.operation != Operation::bit_memory || bit_action(instruction.opcode) != action ||
      (instruction.opcode & five_bit_mask) != bit) {
    return {};
  }
  const auto operand = bit_operand(instruction);
  constexpr Word high_port = 0xffffc0;
  if (operand.location != BitLocation::port_low && operand.location != BitLocation::port_high) {
    return {};
  }
  const auto base = operand.location == BitLocation::port_low ? io_begin : high_port;
  return PortAddress{.space = operand.space, .address = base + operand.selector};
}

bool shift_a(const Instruction &instruction, const unsigned width) noexcept {
  constexpr Word direction_and_destination = (1U << 6U) | 1U;
  return instruction.operation == Operation::logical_shift_immediate &&
         (instruction.opcode & direction_and_destination) == 0 &&
         ((instruction.opcode >> 1U) & five_bit_mask) == width;
}

bool save_high(const Instruction &instruction) noexcept {
  if (instruction.operation != Operation::parallel) { return false; }
  const auto move = transfer(instruction);
  return move && move->source.reg == RegisterId::a1 && move->destination.reg == RegisterId::x1;
}

bool keep_byte(const Instruction &instruction) noexcept {
  constexpr Word alu_kind_and_destination = 0xf;
  constexpr Word and_a = 6;
  return instruction.operation == Operation::immediate_alu && instruction.words == 2 &&
         (instruction.opcode & alu_kind_and_destination) == and_a &&
         instruction.extension == byte_mask;
}

bool merge_high(const Instruction &instruction) noexcept {
  constexpr Word or_x1_a = (6U << 4U) | 2U;
  return instruction.operation == Operation::parallel &&
         instruction.opcode == (no_move_group | or_x1_a);
}

std::optional<unsigned> destination(const Instruction &instruction) noexcept {
  if (instruction.operation != Operation::parallel) { return {}; }
  const auto move = transfer(instruction);
  if (!move || move->source.reg != RegisterId::a1 ||
      move->destination.kind != TransferOperandKind::memory ||
      move->destination.space != TransferSpace::y ||
      move->destination.address.kind != AddressKind::post_increment) {
    return {};
  }
  return move->destination.address.index;
}

enum Step : std::uint8_t {
  first_read,
  first_low,
  shift_high,
  first_high,
  save,
  second_read,
  second_low,
  mask_low,
  shift_low,
  merge,
  second_high,
  write,
  length
};
}

std::optional<PairedPortRead> paired_port_read(const ImagePlan &image, const Word body) noexcept {
  constexpr Word setup_width = 2;
  if (body < setup_width || body > word_mask) { return {}; }
  const auto *setup = image.find(body - setup_width);
  if (setup == nullptr || setup->operation != Operation::loop_immediate ||
      setup->words != setup_width || setup->extension < body || setup->extension > word_mask) {
    return {};
  }
  std::array<Instruction, Step::length> steps{};
  std::array<unsigned, Step::length> cycles{};
  const auto end = setup->extension;
  unsigned elapsed = 0;
  unsigned retired = 0;
  std::size_t used = 0;
  for (auto pc = body; pc <= end;) {
    const auto *instruction = image.find(pc);
    if (instruction == nullptr || instruction->words == 0 || instruction->words > end - pc + 1 ||
        (pc == end && instruction->operation != Operation::nop)) {
      return {};
    }
    if (instruction->operation != Operation::nop) {
      if (used == steps.size()) { return {}; }
      steps[used] = *instruction;
      cycles[used++] = elapsed;
    }
    elapsed += instruction->cycles;
    ++retired;
    pc += instruction->words;
  }
  if (used != steps.size()) { return {}; }
  const auto data = input(steps[Step::first_read]);
  const auto bit = steps[Step::first_low].opcode & five_bit_mask;
  const auto strobe = edge_port(steps[Step::first_low], BitAction::clear, bit);
  const auto reg = destination(steps[Step::write]);
  if (bit >= word_bits || !data || !strobe || !reg || data == strobe ||
      input(steps[Step::second_read]) != data ||
      edge_port(steps[Step::first_high], BitAction::set, bit) != strobe ||
      edge_port(steps[Step::second_low], BitAction::clear, bit) != strobe ||
      edge_port(steps[Step::second_high], BitAction::set, bit) != strobe ||
      !shift_a(steps[Step::shift_high], 16) || !shift_a(steps[Step::shift_low], 8) ||
      !save_high(steps[Step::save]) || !keep_byte(steps[Step::mask_low]) ||
      !merge_high(steps[Step::merge])) {
    return {};
  }
  return PairedPortRead{.data = *data,
                        .strobe = *strobe,
                        .end = setup->extension,
                        .bit = bit,
                        .destination_register = *reg,
                        .retired = retired,
                        .cycles = elapsed,
                        .reads = {cycles[Step::first_read], cycles[Step::second_read]},
                        .edges = {cycles[Step::first_low],
                                  cycles[Step::first_high],
                                  cycles[Step::second_low],
                                  cycles[Step::second_high]}};
}

} // namespace dsp56300::detail
