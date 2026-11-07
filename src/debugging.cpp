// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <cstdint>

namespace dsp56300 {
using namespace detail;

void Core::Impl::request_debug(const bool software) {
  if ((cpu.debug_flags & debug_interrupt_mode) != 0) {
    if ((cpu.debug_flags & debug_interrupt_pending) == 0) {
      constexpr Word vector_bank = 0xffff00;
      constexpr Word debug_offset = 6;

      debug_vector = (cpu.reg.vba & vector_bank) | debug_offset;
      cpu.debug_flags |= debug_interrupt_pending;
    }

    cpu.processing = ExecutionState::running;
    return;
  }

  cpu.debug_flags = static_cast<std::uint8_t>((cpu.debug_flags & ~debug_software_occurrence) |
                                              (software ? debug_software_occurrence : 0));
  cpu.processing = ExecutionState::debugging;
}

void Core::Impl::execute_debug(const Instruction &decoded) {
  if (decoded.operation == Operation::conditional_debug &&
      !condition({.status = cpu.reg.sr, .selector = decoded.opcode & 15U})) {
    return;
  }
  request_debug(true);
}

void Core::Impl::admit_debug() {

  vector = {
      .resume = cpu.reg.pc, .address = debug_vector, .level = 3, .phase = VectorPhase::pending};
  cpu.debug_flags &= static_cast<std::uint8_t>(~debug_interrupt_pending);
  cpu.interrupt_inhibited = false;
}
} // namespace dsp56300
