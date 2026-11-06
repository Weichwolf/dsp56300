// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "region_guards.hpp"

#include <cstddef>

namespace dsp56300::detail {
void BlockEmitter::initialize_loop_window() {
  if (!loop_window_) { return; }
  x64::Label rejected;
  code_.store_constant(x64::Reg::sp, window_flag_offset, x64::Width::byte, 0);
  emit_region_window(code_, region_, loop_size_, rejected);
  code_.store_constant(x64::Reg::sp, window_flag_offset, x64::Width::byte, 1);
  code_.bind(rejected);
}

void BlockEmitter::guard_region_header() {
  auto &failed = exits_.at(dirty_accumulators());
  if (!loop_window_) {
    region_guard(code_, x64::Reg::r12, failed);
    return;
  }
  emit_region_loop_guard(code_, x64::Reg::r12, region_, failed);
  x64::Label proved;
  x64::Label checked;
  constexpr unsigned group_cmp = 7;
  code_.memory_immediate(group_cmp, x64::Reg::sp, window_flag_offset, x64::Width::byte, 0);
  code_.jump_if(x64::Branch::equal, checked);
  // A retained window proves continuing iterations. Recheck the last one.
  constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
  code_.memory_immediate(group_cmp, x64::Reg::r12, lc_offset, x64::Width::dword, 1);
  code_.jump_if(x64::Branch::not_equal, proved);
  code_.bind(checked);
  emit_region_address_guards(code_, x64::Reg::r12, region_, failed);
  code_.bind(proved);
}
} // namespace dsp56300::detail
