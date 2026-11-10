// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <span>

namespace dsp56300::detail::a64 {
bool Emitter::guarded_control(const std::span<const BlockInstruction> operations) {
  if (operations.size() < 2) { return false; }
  const auto &last = operations.back();
  const auto &previous = operations[operations.size() - 2];
  if (at_loop_end(previous) || last.mode != previous.mode ||
      last.loop_address != previous.loop_address ||
      last.pc != ((previous.pc + previous.instruction.words) & word_mask)) {
    return false;
  }
  const auto kind = last.instruction.operation;
  const auto opcode = last.instruction.opcode;
  if (kind == Operation::relative_branch) {
    constexpr Word short_mask = 0xff0000;
    constexpr Word short_form = 0x050000;
    return (opcode & (1U << ((opcode & short_mask) == short_form ? 10 : 6))) != 0;
  }
  if (kind == Operation::conditional_jump) {
    if ((opcode & (1U << 16)) != 0) { return false; }
    constexpr Word short_address = 0x40000;
    if ((opcode & short_address) != 0) { return true; }
  } else if (kind == Operation::jump_short) {
    return true;
  } else if (kind != Operation::jump_effective) {
    return false;
  }
  const auto mode = (opcode >> 11) & 7U;
  return mode == 4 || (mode == 6 && ((opcode >> 8) & 7U) == 0);
}

void Emitter::guarded_control_body(const std::span<const BlockInstruction> operations) {
  guarded_body(operations.first(operations.size() - 1));
  const auto &last = operations.back();
  code_.constant(pc, (last.pc + last.instruction.words) & word_mask);
  flow_ = Flow::sequential;
  old_inputs_ = 0;
  operation(last);
  code_.store(Reg::zr, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
  code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_completed), Access::byte);
  if (region_.loop) { finish_loop(last); }
  guarded_retire(operations);
}
}
