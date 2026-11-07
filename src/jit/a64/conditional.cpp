// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
bool Emitter::numeric_mode(const BlockInstruction &operation, const bool saturating) {
  constexpr Word unsupported =
      compatibility_mode | multiply_mode | arithmetic_16 | arithmetic_saturation;
  const auto allowed = saturating ? arithmetic_saturation : 0U;
  if ((operation.mode & (unsupported & ~allowed)) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down)) {
    fault(operation, FaultKind::unsupported_mode, status);
    return false;
  }
  return true;
}

bool Emitter::supported_alu(const Word opcode) {
  constexpr Word reserved_a = 4;
  constexpr Word reserved_b = 12;
  constexpr Word invalid = 8;
  return opcode != reserved_a && opcode != reserved_b && opcode != invalid;
}

void Emitter::conditional_transfer(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  Label complete;
  condition(Reg::x7, (opcode >> 12) & 15U);
  code_.branch_zero_far(Reg::x7, complete);
  const bool address_only = (opcode & (1U << 11)) != 0;
  if (!address_only && numeric_mode(operation, true)) {
    const auto selector = (opcode >> 4) & 7U;
    if (selector != 0 && selector < 4) {
      code_.constant(Reg::x0, opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    } else {
      source(Reg::x0, selector, (opcode & 8U) != 0);
      code_.move(destination(opcode), Reg::x0);
    }
  }
  if (address_only || (opcode & (1U << 16)) != 0) {
    code_.load(Reg::x0, cpu, offsetof(Registers, r) + (((opcode >> 8) & 7U) * sizeof(Word)));
    code_.store(Reg::x0, cpu, offsetof(Registers, r) + ((opcode & 7U) * sizeof(Word)));
  }
  code_.bind(complete);
}

void Emitter::conditional_alu(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  if (!numeric_mode(operation, supported_alu(opcode & byte_mask))) { return; }
  Label complete;
  condition(Reg::x7, (opcode >> 8) & 15U);
  code_.branch_zero_far(Reg::x7, complete);
  constexpr auto saved_status = 4 * sizeof(std::uint64_t);
  const bool update = (opcode & (1U << 12)) != 0;
  if (!update) { code_.store(status, Reg::sp, saved_status); }
  alu(operation);
  if (!update) { code_.load(status, Reg::sp, saved_status); }
  code_.bind(complete);
}
}
