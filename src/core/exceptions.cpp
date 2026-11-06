// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word vector_base_mask = 0xffff00, illegal_vector = 4, trap_vector = 8;
constexpr Word short_address_mask = 0xfff, priority_mask = 0x300;
constexpr unsigned maximum_level = 3, priority_shift = 8;
constexpr Word transfer_direction = Word{1} << 15;

bool permitted_fast_move(const Instruction &decoded) {
  Word reg = 0;
  bool load = false;
  if (decoded.operation == Operation::move_program_effective ||
      decoded.operation == Operation::move_program_short) {
    reg = decoded.opcode & six_bit_mask;
    load = (decoded.opcode & transfer_direction) != 0;
  } else if (decoded.operation == Operation::move_peripheral_register) {
    reg = (decoded.opcode >> 8) & six_bit_mask;
    // MOVEP W writes the peripheral; MOVEM W reads program memory.
    load = (decoded.opcode & transfer_direction) == 0;
  } else {
    return false;
  }
  // FM A.3.7: SSH reads pop; these control destinations are prohibited in fast vectors.
  if (!load) { return reg != ssh_register; }
  return reg != la_register && reg != lc_register && reg != sp_register && reg != sc_register &&
         reg != ssh_register && reg != ssl_register;
}
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

Instruction Core::Impl::vector_instruction() {

  if (vector.phase == VectorPhase::first && !vector.words) {
    const auto first = fetch_program(vector.address);
    if (fault.kind != FaultKind::none) { return {}; }
    const auto second = fetch_program((vector.address + 1) & word_mask);
    if (fault.kind != FaultKind::none) { return {}; }
    vector.words = std::array{first, second};
  }
  if (!vector.words) { return instruction(current_pc); }
  auto decoded = decode(vector.words->at(vector.phase == VectorPhase::second ? 1U : 0U));
  if (decoded.words == 2 && vector.phase == VectorPhase::first) {
    decoded.extension = vector.words->at(1);
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
  if (decoded.operation == Operation::call_short ||
      decoded.operation == Operation::call_effective) {
    call_vector(decoded);
  } else {
    if ((decoded.operation != Operation::parallel && decoded.operation != Operation::nop &&
         decoded.operation != Operation::move_peripheral_memory &&
         decoded.operation != Operation::move_peripheral_program && !permitted_fast_move(decoded) &&
         decoded.operation != Operation::reset_peripherals &&
         decoded.operation != Operation::debug &&
         decoded.operation != Operation::conditional_debug) ||
        (decoded.words == 2 && vector.phase == VectorPhase::second)) {
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

  synchronize_cache_mode();
  if (fault.kind == FaultKind::none) { retire_instruction(decoded.cycles); }
}
} // namespace dsp56300
