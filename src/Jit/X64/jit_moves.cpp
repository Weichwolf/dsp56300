// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
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
constexpr std::array register_offsets{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
}

void JitEmitter::general_moves(const NativeOp &operation) {
  helper(runtime_.moves, operation);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  code_.exit_if(Branch::equal);
}

void JitEmitter::move_capture(const NativeMove &move, const Word mode) {
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  if (move.immediate()) {
    code_.constant(value, move.address.extra);
  } else if (move.load) {
    const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
    code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
    code_.load_indexed(value,
                       {.base = Reg::r11, .index = move.space == Space::x ? Reg::cx : Reg::r14});
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
    code_.store_indexed({.base = Reg::r11, .index = move.space == Space::x ? Reg::cx : Reg::r14},
                        value);
  }
}

void JitEmitter::moves(const NativeOp &operation) {
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
  x64::Label slow;
  x64::Label done;
  for (const auto &move : transfers) { move_guard(move, slow); }
  if ((operation.instruction.opcode & byte_mask) != 0) {
    for (unsigned index = 0; index < register_offsets.size(); ++index) {
      code_.load(Reg::ax, Reg::r12, offsetof(Cpu, reg) + register_offsets.at(index));
      code_.store(Reg::bp, index * sizeof(std::uint64_t), Reg::ax);
    }
  }
  for (const auto &move : transfers) { move_update(move); }
  // Capture both ports before either destination changes, including host aliases.
  for (const auto &move : transfers) { move_capture(move, operation.mode); }
  for (const auto &move : transfers) { move_commit(move); }
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, native_moves));
  code_.memory_immediate(group_add, Reg::ax, 0, Width::qword, 1);
  code_.jump(done);
  code_.bind(slow);
  general_moves(operation);
  code_.bind(done);
}
} // namespace dsp56300::detail
