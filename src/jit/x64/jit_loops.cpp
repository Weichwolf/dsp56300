// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <cstddef>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
constexpr auto stack_offset = offsetof(Cpu, reg) + offsetof(Registers, stack);
constexpr auto sp_offset = offsetof(Cpu, reg) + offsetof(Registers, sp);
constexpr unsigned group_and = 4, group_sub = 5, group_cmp = 7;
constexpr unsigned op_sub = 0x29, shift_left = 4, shift_right = 5;
}

void JitEmitter::loop_guard(const NativeOp &operation) {
  if (!at_loop_end(operation)) { return; }
  // Final iteration and malformed cleanup remain authoritative general execution.
  code_.load(Reg::ax, Reg::r12, lc_offset, Width::dword);
  code_.immediate(group_cmp, Reg::ax, 1, Width::dword);
  exit_if(Branch::below_equal);
}

void JitEmitter::loop_continue() {
  code_.load(Reg::ax, Reg::r12, sp_offset, Width::dword);
  code_.immediate(group_and, Reg::ax, stack_entries - 1, Width::dword);
  // Indexed addresses scale by four; a frame contains two stored host words.
  code_.shift(shift_left, Reg::ax, 1, Width::dword);
  code_.load_indexed(
      Reg::dx, {.base = Reg::r12, .index = Reg::ax, .offset = stack_offset}, Width::qword);
  code_.shift(shift_right, Reg::dx, word_bits);
  code_.store(Reg::r12, pc_offset, Reg::dx, Width::dword);
  code_.memory_immediate(group_sub, Reg::r12, lc_offset, Width::dword, 1);
}

void JitEmitter::loop_backedge() {
  // loop_continue() leaves the current saved loop-start PC in EDX. The frame
  // may be unrelated to this block, even when its ending LA is unchanged.
  code_.immediate(group_cmp, Reg::dx, loop_pc_, Width::dword);
  exit_if(Branch::not_equal);
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code_.binary(op_sub, {.destination = Reg::ax, .source = Reg::r15}, Width::dword);
  code_.immediate(group_cmp, Reg::ax, loop_size_ - 1, Width::dword);
  exit_if(Branch::below_equal);
  // The header was compiled with empty accumulator caches. Re-establish its
  // memory inputs on every backedge rather than assuming unproven loop phis.
  flush_accumulators(dirty_accumulators());
  code_.jump(loop_header_);
}
} // namespace dsp56300::detail
