// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "translation.hpp"

#include <cassert>

namespace dsp56300 {
void Core::Impl::step_native(const detail::Instruction &decoded) {
  current_pc = cpu.reg.pc;
  current_words = decoded.words;
  const bool repeating = cpu.repeat_remaining != 0;
  const auto retired = jit->run_captured({.instruction = decoded,
                                          .pc = current_pc,
                                          .mode = cpu.reg.sr & detail::numeric_mode_mask,
                                          .loop_address = detail::active_loop_address(cpu.reg)});
  assert(retired == 1 || fault.kind != FaultKind::none);
  if (retired != 0 && repeating) { repeated = decoded; }
  synchronize_cache_mode();
}
}
