// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "vector_instruction.hpp"

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word vector_base_mask = 0xffff00, illegal_vector = 4, trap_vector = 8;
constexpr Word short_address_mask = 0xfff, priority_mask = 0x300;
constexpr unsigned maximum_level = 3, priority_shift = 8;

}

void Core::Impl::execute_exception(const Instruction &decoded) {
  if (decoded.operation == Operation::conditional_trap &&
      !condition({.status = cpu.reg.sr, .selector = decoded.opcode & 15U})) {
    return;
  }
  // REP defers and coalesces instruction exceptions until all attempts finish.
  if (vector.phase == VectorPhase::pending) { return; }
  const bool illegal = decoded.operation == Operation::illegal;
  const bool instruction_fault = illegal || decoded.operation == Operation::undefined;

  const auto foreground_resume = illegal ? address_add(current_pc, 2) : cpu.reg.pc;
  const auto resume = vector.phase == VectorPhase::none ? foreground_resume : vector.resume;
  vector = {.resume = resume,
            .address = (cpu.reg.vba & vector_base_mask) |
                       (instruction_fault ? illegal_vector : trap_vector),
            .level = maximum_level,
            .phase = VectorPhase::pending};
}

void Core::Impl::call_vector(const Instruction &decoded) {
  const auto target = decoded.operation == Operation::call_short
                          ? decoded.opcode & short_address_mask
                          : control_address(decoded);
  if (fault.kind != FaultKind::none) { return; }
  push(vector.resume, cpu.reg.sr);
  if (fault.kind != FaultKind::none) { return; }
  constexpr Word cleared =
      scale_up | scale_down | arithmetic_16 | loop_flag | forever_flag | priority_mask;
  const auto priority = vector.level == maximum_level ? maximum_level : vector.level + 1;

  cpu.reg.sr = (cpu.reg.sr & ~cleared) | (priority << priority_shift);
  cpu.reg.pc = target;

  vector.phase = VectorPhase::none;
}

void Core::Impl::step_vector_semantic(const Instruction &decoded) {
  // The inserted vector contains exactly two words (FM 2.3.2.5/2.3.2.7).
  // A second-slot two-word call must not consume an invented zero extension.
  if (decoded.words == 2 && vector.phase == VectorPhase::second) {
    fail(FaultKind::unsupported_instruction, Space::program, current_pc, decoded.opcode);
    return;
  }
  if (decoded.operation == Operation::call_short ||
      decoded.operation == Operation::call_effective) {
    call_vector(decoded);
  } else {
    if (!permitted_vector_instruction(decoded)) {
      fail(FaultKind::unsupported_instruction, Space::program, current_pc, decoded.opcode);
      return;
    }
    execute_operation(decoded);
    if (fault.kind != FaultKind::none) { return; }
    if (decoded.operation == Operation::undefined) {
      retire_instruction(decoded.cycles);
      return;
    }
    if (decoded.words == 2 || vector.phase == VectorPhase::second) {

      vector.phase = VectorPhase::none;
      cpu.reg.pc = vector.resume;
      cpu.interrupt_inhibited = true;
    } else {

      vector.phase = VectorPhase::second;
    }
  }

  if (fault.kind == FaultKind::none) { retire_instruction(decoded.cycles); }
}
} // namespace dsp56300
