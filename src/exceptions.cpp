// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "vector_instruction.hpp"

#include <array>
#if defined(__aarch64__)
#include "jit.hpp"

#include <cassert>
#endif

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
  if (vector.phase != VectorPhase::none) { return; }
  const bool illegal = decoded.operation == Operation::illegal;
  const bool instruction_fault = illegal || decoded.operation == Operation::undefined;

  vector = {.resume = illegal ? address_add(current_pc, 2) : cpu.reg.pc,
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

Instruction Core::Impl::vector_instruction() {

  if (vector.phase == VectorPhase::first && !vector.loaded) {
    const auto first = fetch_program(vector.address);
    if (fault.kind != FaultKind::none) { return {}; }
    const auto second = fetch_program((vector.address + 1) & word_mask);
    if (fault.kind != FaultKind::none) { return {}; }
    vector.words = std::array{first, second};
    vector.loaded = true;
  }
  if (!vector.loaded) { return instruction(current_pc); }
  auto decoded = decode(vector.words.at(vector.phase == VectorPhase::second ? 1U : 0U));
  if (decoded.words == 2 && vector.phase == VectorPhase::first) {
    decoded.extension = vector.words.at(1);
  }
  return decoded;
}

void Core::Impl::step_vector() {
  if (vector.phase == VectorPhase::pending) {

    cpu.reg.pc = vector.resume;
    cpu.repeat_completed = false;
    vector.phase = VectorPhase::first;
  }
  current_pc = (vector.address + (vector.phase == VectorPhase::second ? 1U : 0U)) & word_mask;
  const auto decoded = vector_instruction();
  if (fault.kind != FaultKind::none) { return; }
  current_words = decoded.words;
#if defined(__aarch64__)
  const auto retired = jit->run_vector(
      {.instruction = decoded, .pc = current_pc, .mode = cpu.reg.sr & numeric_mode_mask});
  assert(retired == 1 || fault.kind != FaultKind::none);
  if (retired != 0) { synchronize_cache_mode(); }
  synchronize_program_view();
#else
  step_vector_semantic(decoded);
#endif
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
