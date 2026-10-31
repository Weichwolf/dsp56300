// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "hidden_retirement.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace dsp56300 {
using namespace detail;

void Core::Impl::mark_hidden(const RetirementField field) const {
  if (!pcu_publication()) { return; }
  if (field >= RetirementField::count) { throw std::logic_error("Unknown native hidden writer"); }
  staged->hidden_result.fields |= retirement_bit(field);
}

void Core::Impl::mark_debug(const std::uint8_t mask) const {
  if (!pcu_publication()) { return; }
  mark_hidden(RetirementField::debug_flags);
  staged->hidden_result.debug_mask |= mask;
}

void Core::Impl::mark_vector() const {
  mark_hidden(RetirementField::vector_resume);
  mark_hidden(RetirementField::vector_address);
  mark_hidden(RetirementField::vector_level);
  mark_hidden(RetirementField::vector_phase);
}

void Core::Impl::retire_instruction(const unsigned cycles) {
  ++cpu.instructions;
  cpu.cycles += cycles;
  if (pcu_publication()) {
    ++staged->hidden_result.delta.instructions;
    staged->hidden_result.delta.estimated_cycles += cycles;
  }
}

void Core::Impl::retire_owned() {
  auto &work = *staged;
  if (work.publication == InstructionPublication::pcu_updates) {
    merge_hidden_retirement(cpu, vector, debug_vector, repeated, work.hidden_result);
  } else {
    auto cache = std::move(icache);
    restore_execution(work.candidate);
    icache = std::move(cache);
  }
}
} // namespace dsp56300
