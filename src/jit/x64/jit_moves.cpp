// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "native_region.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned op_test = 0x85, group_add = 0, group_and = 4;
}

void JitEmitter::general_moves(const NativeOp &operation) {
  helper(runtime_.moves, operation);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  // helper() materialized all previous results. Preserve the helper's own
  // partial fault state rather than flushing pre-transfer cached operands.
  code_.jump_if(Branch::equal, exits_[0]);
  restore_inputs();
}

void JitEmitter::move_capture(const NativeMove &move, const Word mode) {
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  if (move.immediate()) {
    code_.constant(value, move.address.extra);
  } else if (move.load) {
    const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
    code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
    code_.load_indexed(value,
                       {.base = Reg::r11, .index = move.space == Space::x ? Reg::cx : Reg::r8});
    code_.immediate(group_and, value, word_mask, Width::dword);
  } else {
    move_read_register(value, move, mode);
  }
}

void JitEmitter::move_commit(const NativeMove &move) {
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  if (move.load) {
    move_write_register(value, move.reg);
  } else {
    const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
    code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
    code_.store_indexed({.base = Reg::r11, .index = move.space == Space::x ? Reg::cx : Reg::r8},
                        value);
  }
}

void JitEmitter::register_move(const NativeRegisterMove &move, const Word mode) {
  capture_inputs();
  move_read_register(Reg::si, {.address = {}, .reg = move.source}, mode);
  move_write_register(Reg::si, move.destination);
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, native_moves));
  code_.memory_immediate(group_add, Reg::ax, 0, Width::qword, 1);
}

void JitEmitter::moves(const NativeOp &operation) {
  const auto immediate = plan_address_immediate(operation.instruction);
  if (immediate.valid()) {
    address_immediate(immediate);
    return;
  }
  if (pure_address_update(operation.instruction)) {
    address_update(operation);
    return;
  }
  const auto reg = plan_register_move(operation.instruction);
  if (reg.valid()) {
    register_move(reg, operation.mode);
    return;
  }
  const auto plan = plan_moves(operation.instruction);
  const std::span transfers(plan.transfers.data(), plan.count);
  for (const auto &move : transfers) {
    if (!move.load && runtime_.memory.at(move.space == Space::x ? 0 : 1).aliases_program) {
      general_moves(operation);
      return;
    }
  }
  if (transfers.empty()) {
    general_moves(operation);
    return;
  }
  if (region_mode_ == RegionMode::guarded) {
    for (const auto &move : transfers) { region_address(move); }
    capture_inputs();
    for (const auto &move : transfers) { move_update(move); }
    for (const auto &move : transfers) { move_capture(move, operation.mode); }
    for (const auto &move : transfers) { move_commit(move); }
    code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, native_moves));
    code_.memory_immediate(group_add, Reg::ax, 0, Width::qword, 1);
    return;
  }
  x64::Label slow;
  x64::Label done;
  const auto first_guard = ring_guards_.size();
  for (const auto &move : transfers) { move_guard(move, slow); }
  capture_inputs();
  for (const auto &move : transfers) { move_update(move); }
  // Capture both ports before either destination changes, including host aliases.
  for (const auto &move : transfers) { move_capture(move, operation.mode); }
  for (const auto &move : transfers) { move_commit(move); }
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, native_moves));
  code_.memory_immediate(group_add, Reg::ax, 0, Width::qword, 1);
  code_.jump(done);
  code_.bind(slow);
  // Late ring checks retain this instruction's target after its local label dies.
  for (auto index = first_guard; index < ring_guards_.size(); ++index) {
    ring_guards_[index].slow.offset = slow.offset;
    ring_guards_[index].slow.bound = true;
  }
  general_moves(operation);
  code_.bind(done);
}
} // namespace dsp56300::detail
