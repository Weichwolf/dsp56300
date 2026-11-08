// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "jit/x64/io.hpp"
#include "move.hpp"

#include <array>
#include <cstddef>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned group_or = 1, group_and = 4, group_xor = 6, group_cmp = 7, shift_right = 5;
constexpr unsigned op_test = 0x85;
constexpr Word bit_mask = 31;
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);

BusAction native_bit_action(const Instruction &instruction) {
  constexpr std::array actions{
      BusAction::bit_test, BusAction::bit_clear, BusAction::bit_set, BusAction::bit_change};
  const auto action = instruction.operation == Operation::bit_memory
                          ? bit_action(instruction.opcode)
                          : BitAction::test;
  return actions[static_cast<unsigned>(action)];
}

NativeMove bit_move(const Instruction &instruction) {
  const auto operand = bit_operand(instruction);
  if (operand.location == BitLocation::port_low || operand.location == BitLocation::port_high) {
    constexpr Word high_begin = 0xffffc0;
    const auto base = operand.location == BitLocation::port_low ? io_begin : high_begin;
    return {.address = {.mode = 6, .index = 0, .extra = base + operand.selector},
            .space = operand.space,
            .load = true};
  }
  const auto form = operand.location == BitLocation::short_address
                        ? AddressForm{.mode = 6, .index = 0, .extra = operand.selector}
                        : AddressForm{.mode = operand.selector >> 3U,
                                      .index = operand.selector & 7U,
                                      .extra = instruction.extension};
  return {.address = form, .space = operand.space, .load = true};
}

}

bool BlockEmitter::is_bits(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const bool ordinary = instruction.operation == Operation::bit_memory;
  if ((!ordinary && instruction.operation != Operation::bit_jump &&
       instruction.operation != Operation::bit_branch) ||
      (instruction.opcode & bit_mask) >= word_bits || at_loop_end(operation) ||
      (!ordinary && bit_call(instruction))) {
    return false;
  }
  const auto operand = bit_operand(instruction);
  if (operand.location == BitLocation::short_address ||
      operand.location == BitLocation::port_high || operand.location == BitLocation::port_low) {
    return true;
  }
  if (operand.location != BitLocation::effective) { return false; }
  const auto mode = operand.selector >> 3U;
  return mode == 4 || (ordinary && mode == 6 && (operand.selector & 7U) == 0);
}

void BlockEmitter::bit_result(const BlockInstruction &operation,
                              NativeMove move,
                              const bool publish) {
  const auto &instruction = operation.instruction;
  const auto value = move.space == Space::x ? Reg::si : Reg::di;
  const auto bit = instruction.opcode & bit_mask;
  move_capture(move, operation.mode);
  code_.mov({.destination = Reg::dx, .source = value}, Width::dword);
  code_.shift(shift_right, Reg::dx, bit, Width::dword);
  code_.immediate(group_and, Reg::dx, 1, Width::dword);
  if (instruction.operation == Operation::bit_memory) {
    const auto action = bit_action(instruction.opcode);
    if (action != BitAction::test) {
      auto group = group_xor;
      if (action == BitAction::clear) { group = group_and; }
      if (action == BitAction::set) { group = group_or; }
      const auto mask = Word{1} << bit;
      code_.immediate(
          group, value, action == BitAction::clear ? word_mask ^ mask : mask, Width::dword);
      move.load = false;
      move_commit(move);
    }
  }
  bit_complete(operation, publish);
}

void BlockEmitter::bit_complete(const BlockInstruction &operation, const bool publish) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::bit_memory) {
    merge_status(Reg::dx, carry);
    retire(operation, publish);
    return;
  }
  const auto next = (operation.pc + instruction.words) & word_mask;
  const auto target = instruction.operation == Operation::bit_jump
                          ? instruction.extension
                          : (operation.pc + instruction.extension) & word_mask;
  code_.store_constant(Reg::r12, pc_offset, Width::dword, next);
  x64::Label untaken;
  code_.binary(op_test, {.destination = Reg::dx, .source = Reg::dx}, Width::dword);
  const bool wanted = (instruction.opcode & 32U) != 0;
  code_.jump_if(wanted ? Branch::equal : Branch::not_equal, untaken);
  code_.store_constant(Reg::r12, pc_offset, Width::dword, target);
  code_.bind(untaken);
  retire(operation, false);
  code_.jump(exits_[0]);
}

bool BlockEmitter::bits(const BlockInstruction &operation, const bool publish) {
  if (!is_bits(operation)) { return false; }
  // A failed guard delegates the complete instruction. Commit preceding cached
  // accumulators on both paths so its partial fault state remains authoritative.
  flush_accumulators(dirty_accumulators());
  accumulator_dirty_.fill(false);
  const auto move = bit_move(operation.instruction);
  const auto bank = move.space == Space::x ? 0U : 1U;
  const bool modify = operation.instruction.operation == Operation::bit_memory &&
                      bit_action(operation.instruction.opcode) != BitAction::test;
  x64::Label slow;
  x64::Label done;
  const bool direct = move.address.mode == 6 && move.address.extra >= io_begin;
  if (!direct) {
    const auto &memory = runtime_.memory.at(bank);
    if ((runtime_.windows.at(bank).empty() && memory.data == nullptr) ||
        (modify && memory.aliases_program)) {
      code_.jump(slow);
    }
    if (modify) {
      auto store = move;
      store.load = false;
      move_guard(store, slow);
    }
    move_guard(move, slow);
    bit_result(operation, move, publish);
    code_.jump(done);
  }
  code_.bind(slow);
  x64::Label fallback;
  if (!repeat_mode_ && (direct || move.address.mode != 6)) {
    // A failed RAM guard can also mean a real indirect peripheral address.
    // Supported address forms have no AGU update, so recomputing it is inert.
    move_effective(move);
    if (!direct) {
      code_.immediate(
          group_cmp, move.space == Space::x ? Reg::cx : Reg::r8, io_begin, Width::dword);
      code_.jump_if(Branch::below, fallback);
    }
    code_.constant(Reg::dx, operation.instruction.opcode & bit_mask);
    native_io_access(code_,
                     operation,
                     {.space = move.space,
                      .action = native_bit_action(operation.instruction),
                      .address = move.space == Space::x ? Reg::cx : Reg::r8},
                     {.fallback = fallback, .failure = exits_[0]});
    bit_complete(operation, publish);
    if (operation.instruction.operation == Operation::bit_memory) {
      code_.memory_immediate(
          group_cmp, Reg::r13, offsetof(NativeContext, boundary), Width::byte, 0);
      code_.jump_if(Branch::not_equal, exits_[0]);
    }
    code_.jump(done);
  }
  code_.bind(fallback);
  instruction_fallback(operation);
  code_.bind(done);
  return true;
}
} // namespace dsp56300::detail
