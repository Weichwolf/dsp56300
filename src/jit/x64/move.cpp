// SPDX-License-Identifier: GPL-3.0-only
#include "move.hpp"

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"
#include "value_ranges.hpp"

#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned op_test = 0x85, group_and = 4;
}

void BlockEmitter::general_moves(const BlockInstruction &operation) {
  if (repeat_mode_) {
    code_.jump(exits_.at(dirty_accumulators()));
    return;
  }
  if (range_mode_ == RangeMode::automatic && direct_memory(operation)) {
    // Only guarded RAM continues into code that retains value-range proofs.
    // A real observer executes the complete operation and leaves this entry.
    helper(runtime_.execute, operation);
    code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
    code_.jump_if(Branch::equal, exits_[0]);
    code_.immediate(0, Reg::r15, 1, Width::dword);
    code_.jump(exits_[0]);
    return;
  }
  helper(runtime_.moves, operation);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  // helper() materialized all previous results. Preserve the helper's own
  // partial fault state rather than flushing pre-transfer cached operands.
  code_.jump_if(Branch::equal, exits_[0]);
  restore_inputs();
}

void BlockEmitter::move_capture(const NativeMove &move, const Word mode) {
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  if (move.immediate()) {
    code_.constant(value, move.address.extra);
  } else if (move.load) {
    const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
    const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
    if (runtime_.windows.at(move.space == Space::x ? 0 : 1).empty()) {
      code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
      code_.load_indexed(value, {.base = Reg::r11, .index = address});
    } else {
      code_.load(value, address, 0, Width::dword);
    }
    code_.immediate(group_and, value, word_mask, Width::dword);
  } else {
    move_read_register(value, move, mode);
  }
}

void BlockEmitter::move_commit(const NativeMove &move) {
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  if (move.load) {
    move_write_register(value, move.reg);
  } else {
    const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
    const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
    if (runtime_.windows.at(move.space == Space::x ? 0 : 1).empty()) {
      code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
      code_.store_indexed({.base = Reg::r11, .index = address}, value);
    } else {
      code_.store(address, 0, value, Width::dword);
      mark_window_written(move);
    }
  }
}

void BlockEmitter::register_move(const NativeRegisterMove &move, const Word mode) {
  capture_inputs();
  move_read_register(Reg::si, {.address = {}, .reg = move.source}, mode);
  move_write_register(Reg::si, move.destination);
}

void BlockEmitter::move_guards(const NativeMoves &moves, x64::Label &slow) {
  if (moves.long_pair) {
    long_guards(moves, slow);
    return;
  }
  for (const auto &move : std::span(moves.transfers).first(moves.count)) { move_guard(move, slow); }
}

void BlockEmitter::move_values(const NativeMoves &moves, const Word mode) {
  const auto transfers = std::span(moves.transfers).first(moves.count);
  capture_inputs();
  if (moves.long_pair) {
    move_update(transfers.front());
    long_capture(moves, mode);
  } else {
    for (const auto &move : transfers) { move_update(move); }
    for (const auto &move : transfers) { move_capture(move, mode); }
  }
  // Capture both ports before either destination changes, including host aliases.
  if (moves.long_pair && transfers.front().load) {
    long_commit(moves);
  } else {
    for (const auto &move : transfers) { move_commit(move); }
  }
}

void BlockEmitter::moves(const BlockInstruction &operation) {
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
    move_values(plan, operation.mode);
    return;
  }
  x64::Label slow;
  x64::Label done;
  const auto first_guard = ring_guards_.size();
  move_guards(plan, slow);
  move_values(plan, operation.mode);
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
