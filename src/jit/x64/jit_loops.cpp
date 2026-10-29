// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
constexpr auto stack_offset = offsetof(Cpu, reg) + offsetof(Registers, stack);
constexpr auto sp_offset = offsetof(Cpu, reg) + offsetof(Registers, sp);
constexpr auto sc_offset = offsetof(Cpu, reg) + offsetof(Registers, sc);
constexpr auto la_offset = offsetof(Cpu, reg) + offsetof(Registers, la);
constexpr auto sr_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
constexpr unsigned group_add = 0, group_or = 1;
constexpr unsigned group_and = 4, group_sub = 5, group_cmp = 7;
constexpr unsigned op_sub = 0x29, op_or = 0x09, op_test = 0x85, shift_left = 4, shift_right = 5;
constexpr Word stack_pointer_mask = 63, relative_bit = 16;

Word setup_count(const NativeOp &operation) {
  const auto opcode = operation.instruction.opcode;
  return ((opcode >> 8) & byte_mask) | ((opcode & 15U) << 8);
}

Word setup_end(const NativeOp &operation) {
  const auto &instruction = operation.instruction;
  return ((instruction.opcode & relative_bit) != 0 ? operation.pc + instruction.extension
                                                   : instruction.extension) &
         word_mask;
}
}

bool JitEmitter::is_loop_setup(const NativeOp &operation) {
  return operation.instruction.operation == Operation::loop_immediate &&
         operation.instruction.words == 2 && setup_count(operation) != 0 &&
         !at_loop_end(operation) && setup_end(operation) != ((operation.pc + 1) & word_mask);
}

void JitEmitter::loop_push(const Reg frame) {
  code_.load(Reg::cx, Reg::r12, sp_offset, Width::dword);
  code_.immediate(group_add, Reg::cx, 1, Width::dword);
  code_.immediate(group_and, Reg::cx, stack_pointer_mask, Width::dword);
  code_.store(Reg::r12, sp_offset, Reg::cx, Width::dword);
  code_.memory_immediate(group_add, Reg::r12, sc_offset, Width::dword, 1);
  code_.immediate(group_and, Reg::cx, stack_entries - 1, Width::dword);
  code_.shift(shift_left, Reg::cx, 1, Width::dword);
  code_.store_indexed(
      {.base = Reg::r12, .index = Reg::cx, .offset = stack_offset}, frame, Width::qword);
}

void JitEmitter::loop_setup(const NativeOp &operation) {
  x64::Label slow;
  // A failed second push must retain the general executor's first push and LC.
  // Guard the complete capacity before making any native change.
  code_.load(Reg::ax, Reg::r12, sc_offset, Width::dword);
  code_.immediate(group_cmp, Reg::ax, stack_entries - 2, Width::dword);
  code_.jump_if(Branch::above_equal, slow);
  code_.load(Reg::ax, Reg::r12, la_offset, Width::dword);
  code_.shift(shift_left, Reg::ax, word_bits);
  code_.load(Reg::dx, Reg::r12, lc_offset, Width::dword);
  code_.binary(op_or, {.destination = Reg::ax, .source = Reg::dx});
  loop_push(Reg::ax);
  const auto next = (operation.pc + operation.instruction.words) & word_mask;
  code_.constant(Reg::ax, std::uint64_t{next} << word_bits);
  read_status(Reg::dx);
  code_.binary(op_or, {.destination = Reg::ax, .source = Reg::dx});
  loop_push(Reg::ax);
  code_.immediate(group_and, Reg::dx, ~forever_flag, Width::dword);
  code_.immediate(group_or, Reg::dx, loop_flag, Width::dword);
  code_.store(Reg::r12, sr_offset, Reg::dx, Width::dword);
  code_.store_constant(Reg::r12, la_offset, Width::dword, setup_end(operation));
  code_.store_constant(Reg::r12, lc_offset, Width::dword, setup_count(operation));
  retire(operation, true);
  code_.jump(exits_.at(dirty_accumulators()));
  code_.bind(slow);
  helper(runtime_.execute, operation);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  code_.jump_if(Branch::equal, exits_[0]);
  code_.immediate(group_add, Reg::r15, 1, Width::dword);
  code_.jump(exits_[0]);
}

void JitEmitter::loop_guard(const NativeOp &operation) {
  if (!at_loop_end(operation)) { return; }
  materialize_pending_accounting();
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
  // Ordinary headers still require canonical inputs. Closed resident loops
  // initialized both slots before admission and preserve them across this edge.
  if (!loop_residency_) { flush_accumulators(dirty_accumulators()); }
  code_.jump(loop_header_);
}

void JitEmitter::initialize_loop_residency() {
  if (!loop_residency_) { return; }
  // Initial rejection retains the original zero-dirty exit. Subsequent guards
  // can fail after complete iterations and must publish both initialized slots.
  region_guard(code_, Reg::r12, exits_[0]);
  code_.load(Reg::bp, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, a));
  code_.load(Reg::bx, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, b));
  if (loop_bank_cache_) {
    code_.packed_load_lane<64>(x64::Xmm::x5, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, x));
    code_.packed_load_lane<64>(x64::Xmm::x6, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, y));
  }
  accumulator_dirty_.fill(true);
}

void JitEmitter::materialize_loop_banks() {
  code_.packed_to(Reg::ax, x64::Xmm::x5);
  code_.store(Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, x), Reg::ax);
  code_.packed_to(Reg::ax, x64::Xmm::x6);
  code_.store(Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, y), Reg::ax);
}
} // namespace dsp56300::detail
