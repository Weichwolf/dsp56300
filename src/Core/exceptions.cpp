// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"

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
  // Repeated ILLEGAL raises one pending exception, admitted after REP finishes.
  if (vector.phase != VectorPhase::none) { return; }
  const bool illegal = decoded.operation == Operation::illegal;
  vector = {.resume = illegal ? address_add(current_pc, 2) : cpu.reg.pc,
            .address = (cpu.reg.vba & vector_base_mask) | (illegal ? illegal_vector : trap_vector),
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

void Core::Impl::step_vector() {
  if (vector.phase == VectorPhase::pending) {
    cpu.reg.pc = vector.resume;
    cpu.repeat_completed = false;
    vector.phase = VectorPhase::first;
  }
  current_pc = (vector.address + (vector.phase == VectorPhase::second ? 1U : 0U)) & word_mask;
  const auto decoded = instruction(current_pc);
  if (fault.kind != FaultKind::none) { return; }
  current_words = decoded.words;
  if (decoded.operation == Operation::call_short ||
      decoded.operation == Operation::call_effective) {
    call_vector(decoded);
  } else {
    if ((decoded.operation != Operation::parallel && decoded.operation != Operation::nop) ||
        (decoded.words == 2 && vector.phase == VectorPhase::second)) {
      fail(FaultKind::unsupported_instruction, Space::program, current_pc, decoded.opcode);
      return;
    }
    execute(decoded);
    if (fault.kind != FaultKind::none) { return; }
    if (decoded.words == 2 || vector.phase == VectorPhase::second) {
      vector.phase = VectorPhase::none;
      cpu.reg.pc = vector.resume;
      cpu.interrupt_inhibited = true;
    } else {
      vector.phase = VectorPhase::second;
    }
  }
  if (fault.kind == FaultKind::none) {
    ++cpu.instructions;
    cpu.cycles += decoded.cycles;
  }
}
} // namespace dsp56300
