// SPDX-License-Identifier: GPL-3.0-only
#include "jit/fixed_pass.hpp"

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
#include "transfers.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <span>

namespace dsp56300::detail {
namespace {
class Effects {
public:
  explicit Effects(std::span<const PassCell> stable_reads) : stable_reads_(stable_reads) {}

  [[nodiscard]] bool step(const Instruction &instruction, const Instruction *next);
  [[nodiscard]] std::optional<PassEffects> finish(FixedPassCost cost) const;

private:
  [[nodiscard]] bool bit(const Instruction &instruction);
  [[nodiscard]] bool moves(const Instruction &instruction, const Instruction *next);
  [[nodiscard]] bool access(Space space, PassAddress resolved, bool write, Word mask = word_mask);
  [[nodiscard]] std::optional<PassAddress> address(const EncodedAddress &form, Word extra);
  [[nodiscard]] bool
  access_operand(const TransferOperand &operand, Word extra, bool write, Word mask = word_mask);
  void define_address(EncodedTransfer move, Word extra);

  std::span<const PassCell> stable_reads_;
  std::array<std::optional<PassAddress>, agu_registers> registers_{};
  std::set<PassRead> reads_;
  std::set<PassCell> writes_;
  std::map<PassCell, Word> device_reads_;
  std::uint8_t linear_ = 0;
};

std::optional<PassAddress> Effects::address(const EncodedAddress &form, const Word extra) {
  if (form.kind == AddressKind::absolute) { return PassAddress{.offset = form.literal}; }
  if (form.kind == AddressKind::absolute_extension) { return PassAddress{.offset = extra}; }
  const auto index = form.index;
  if (index >= registers_.size() || !registers_[index]) { return std::nullopt; }
  const auto old = *registers_[index];
  if (form.kind == AddressKind::indirect) { return old; }
  if (form.kind != AddressKind::post_increment) { return std::nullopt; }
  linear_ |= static_cast<std::uint8_t>(1U << index);
  registers_[index]->offset = (old.offset + 1U) & word_mask;
  return old;
}

bool Effects::access(const Space space,
                     const PassAddress resolved,
                     const bool write,
                     const Word mask) {
  if (resolved.base) {
    if (write) { return false; }
    reads_.insert({.space = space, .address = resolved});
    return true;
  }
  const PassCell cell{.space = space, .address = resolved.offset};
  if (resolved.offset >= io_begin) {
    if (write || resolved.offset > word_mask ||
        std::ranges::find(stable_reads_, cell) == stable_reads_.end()) {
      return false;
    }
    device_reads_[cell] |= mask;
  } else if (write) {
    writes_.insert(cell);
  } else {
    reads_.insert({.space = space, .address = resolved});
  }
  return true;
}

bool Effects::access_operand(const TransferOperand &operand,
                             const Word extra,
                             const bool write,
                             const Word mask) {
  if (operand.kind != TransferOperandKind::memory) {
    const auto reg = operand.reg;
    return !write || reg < RegisterId::n0 || reg >= RegisterId::x;
  }
  // Double-bank peripheral accesses have a different observable protocol.
  if (operand.space != TransferSpace::x && operand.space != TransferSpace::y) { return false; }
  const auto resolved = address(operand.address, extra);
  const auto space = operand.space == TransferSpace::x ? Space::x : Space::y;
  return resolved && access(space, *resolved, write, mask);
}

void Effects::define_address(const EncodedTransfer move, const Word extra) {
  if (move.destination.kind != TransferOperandKind::register_value ||
      move.destination.reg < RegisterId::r0 || move.destination.reg > RegisterId::r7) {
    return;
  }
  auto &result = registers_[static_cast<std::size_t>(move.destination.reg) - r_register_base];
  const auto &source = move.source;
  result.reset();
  if (source.kind == TransferOperandKind::immediate) {
    result = PassAddress{.offset = (source.immediate << source.immediate_shift) & word_mask};
  } else if (source.kind == TransferOperandKind::extension) {
    result = PassAddress{.offset = extra};
  } else if (source.kind == TransferOperandKind::memory && source.space == TransferSpace::x &&
             source.address.kind == AddressKind::absolute && source.address.literal < io_begin) {
    result = PassAddress{.offset = 0,
                         .base = PassCell{.space = Space::x, .address = source.address.literal}};
  }
}

Word surviving_port_bits(const Instruction &load,
                         const Instruction *next,
                         const EncodedTransfer &move) noexcept {
  if (load.operation != Operation::move_peripheral_register || next == nullptr ||
      next->operation != Operation::immediate_alu || (next->opcode & 7U) != 6U ||
      move.source.kind != TransferOperandKind::memory ||
      move.destination.kind != TransferOperandKind::register_value) {
    return word_mask;
  }
  const bool to_b = (next->opcode & 8U) != 0;
  const auto reg = move.destination.reg;
  if (reg != (to_b ? RegisterId::b : RegisterId::a) &&
      reg != (to_b ? RegisterId::b1 : RegisterId::a1)) {
    return word_mask;
  }
  const Word bits = next->words == 2 ? next->extension : (next->opcode >> 8U) & six_bit_mask;
  // The full accumulator load sign-extends into A2/B2. AND updates only A1/B1.
  return bits | (reg == RegisterId::a || reg == RegisterId::b ? sign24_bit : Word{0});
}

bool Effects::moves(const Instruction &instruction, const Instruction *next) {
  const auto transfer = inspect_transfers(instruction.opcode);
  if (!transfer || transfer->address_update) { return false; }
  const auto operands = std::span(transfer->moves).first(transfer->count);
  for (const auto &move : operands) {
    const auto mask = operands.size() == 1 && transfer->parallel_alu == 0
                          ? surviving_port_bits(instruction, next, move)
                          : word_mask;
    if (!access_operand(move.source, instruction.extension, false, mask) ||
        !access_operand(move.destination, instruction.extension, true)) {
      return false;
    }
  }
  for (const auto &move : operands) { define_address(move, instruction.extension); }
  return true;
}

bool Effects::bit(const Instruction &instruction) {
  const auto operand = bit_operand(instruction);
  if (operand.location != BitLocation::short_address) { return false; }
  const PassAddress at{.offset = operand.selector};
  const bool write = instruction.operation == Operation::bit_memory &&
                     bit_action(instruction.opcode) != BitAction::test;
  return access(operand.space, at, false) && (!write || access(operand.space, at, true));
}

bool Effects::step(const Instruction &instruction, const Instruction *next) {
  const auto kind = instruction.operation;
  constexpr std::array cpu_only{Operation::nop,
                                Operation::call_short,
                                Operation::call_effective,
                                Operation::jump_short,
                                Operation::jump_effective,
                                Operation::return_subroutine,
                                Operation::relative_branch,
                                Operation::conditional_jump,
                                Operation::loop_immediate,
                                Operation::immediate_alu};
  if (std::ranges::find(cpu_only, kind) != cpu_only.end()) { return true; }
  if (kind == Operation::bit_memory || kind == Operation::bit_jump ||
      kind == Operation::bit_branch) {
    return bit(instruction);
  }
  return (kind == Operation::parallel || kind == Operation::move_peripheral_register) &&
         moves(instruction, next);
}

std::optional<PassEffects> Effects::finish(const FixedPassCost cost) const {
  if (std::ranges::any_of(reads_, [&](const PassRead &read) {
        return read.address.base && writes_.contains(*read.address.base);
      })) {
    return std::nullopt;
  }
  PassEffects result{.cost = cost,
                     .reads = {reads_.begin(), reads_.end()},
                     .writes = {writes_.begin(), writes_.end()},
                     .device_reads = {},
                     .device_masks = {},
                     .linear = linear_};
  for (const auto &[cell, mask] : device_reads_) {
    result.device_reads.push_back(cell);
    result.device_masks.push_back(mask);
  }
  return result;
}
}

std::optional<PassEffects> fixed_pass_effects(const ImagePlan &image,
                                              const std::span<const Word> path,
                                              const std::span<const PassCell> stable_reads) {
  const auto cost = image.fixed_pass_cost(path);
  if (!cost) { return std::nullopt; }
  Effects effects(stable_reads);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto *next = i + 1 < path.size() ? image.find(path[i + 1]) : nullptr;
    if (!effects.step(*image.find(path[i]), next)) { return std::nullopt; }
  }
  return effects.finish(*cost);
}
}
