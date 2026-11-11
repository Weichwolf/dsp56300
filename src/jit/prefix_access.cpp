// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/prefixes.hpp"
#include "jit/preparation.hpp"
#include "transfers.hpp"

#include <algorithm>
#include <cstddef>
#include <span>

namespace dsp56300::detail {
namespace {
bool ordinary(const TransferOperand &operand,
              const Instruction &instruction,
              const bool write,
              const std::span<const RegionLayout> memory) {
  if (operand.kind == TransferOperandKind::register_value) {
    return operand.reg < RegisterId::ep || operand.reg > RegisterId::lc;
  }
  if (operand.kind != TransferOperandKind::memory) { return !write; }
  const auto kind = operand.address.kind;
  if (kind != AddressKind::absolute && kind != AddressKind::absolute_extension) { return false; }
  const auto address =
      kind == AddressKind::absolute ? operand.address.literal : instruction.extension;
  const auto fits = [&](const Space space) {
    return std::ranges::any_of(memory, [&](const auto &region) {
      return region.space == space && address >= region.first &&
             std::size_t{address - region.first} < region.size &&
             (write ? region.writable && !region.aliases_program && space != Space::program
                    : !region.tracked);
    });
  };
  switch (operand.space) {
    case TransferSpace::program: return fits(Space::program);
    case TransferSpace::x: return fits(Space::x);
    case TransferSpace::y: return fits(Space::y);
    case TransferSpace::long_pair: return fits(Space::x) && fits(Space::y);
    default: return false;
  }
}
}

bool prefix_boundary(const Instruction &instruction, const std::span<const RegionLayout> memory) {
  if (instruction.operation == Operation::nop) { return false; }
  const auto flow = successors({.instruction = instruction, .pc = 0});
  if (flow.indirect || flow.call) { return true; }
  const auto kind = instruction.operation;
  if (kind == Operation::jump_short || kind == Operation::jump_effective ||
      kind == Operation::conditional_jump || kind == Operation::relative_branch) {
    return false;
  }
  const auto transfers = inspect_transfers(instruction.opcode);
  if (!transfers) { return true; }
  return std::ranges::any_of(std::span(transfers->moves).first(transfers->count),
                             [&](const auto &move) {
                               return !ordinary(move.source, instruction, false, memory) ||
                                      !ordinary(move.destination, instruction, true, memory);
                             });
}
}
