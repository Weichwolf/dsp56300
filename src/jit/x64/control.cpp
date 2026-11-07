// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr Word short_mask = 0xfff;
constexpr Word stack_pointer_mask = 63;
constexpr unsigned unconditional = 16;
constexpr unsigned condition_mask = 127;
constexpr unsigned group_and = 4;
constexpr unsigned group_sub = 5;
constexpr unsigned group_cmp = 7;
constexpr unsigned op_add = 0x01;
constexpr unsigned op_or = 0x09;
constexpr unsigned shift_right = 5;
constexpr unsigned shift_left = 4;
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
constexpr auto sp_offset = offsetof(Cpu, reg) + offsetof(Registers, sp);
constexpr auto sc_offset = offsetof(Cpu, reg) + offsetof(Registers, sc);
constexpr auto stack_offset = offsetof(Cpu, reg) + offsetof(Registers, stack);

alignas(64) constexpr auto conditions = [] {
  std::array<std::array<std::uint8_t, condition_mask + 1>, unconditional> table{};
  for (unsigned selector = 0; selector < table.size(); ++selector) {
    for (unsigned status = 0; status < table[selector].size(); ++status) {
      table[selector][status] =
          static_cast<std::uint8_t>(condition({.status = status, .selector = selector}));
    }
  }
  return table;
}();

struct ControlPlan {
  Word target = 0;
  unsigned selector = unconditional;
  unsigned offset_register = agu_registers;
  bool call = false, returns = false, valid = false;
};

ControlPlan relative_plan(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr Word group_mask = 0xff0000;
  constexpr Word short_group = 0x050000;
  const bool short_form = (opcode & group_mask) == short_group;
  const bool always = (opcode & (Word{1} << (short_form ? 11U : 7U))) != 0;
  const bool call = (opcode & (Word{1} << (short_form ? 10U : 6U))) == 0;
  auto offset = operation.instruction.extension;
  unsigned reg = agu_registers;
  if (short_form) {
    constexpr Word upper = 0x1e0;
    constexpr Word lower = 0x1f;
    constexpr Word sign = 0x100;
    constexpr Word offset_mask = 0x1ff;
    offset = ((opcode >> 1U) & upper) | (opcode & lower);
    if ((offset & sign) != 0) { offset |= word_mask ^ offset_mask; }
  } else if ((opcode & (Word{1} << 11U)) != 0) {
    reg = (opcode >> 8U) & 7U;
    offset = 0;
  }
  const auto condition_shift = short_form ? 12U : 0U;
  return {.target = (operation.pc + offset) & word_mask,
          .selector = always ? unconditional : (opcode >> condition_shift) & 15U,
          .offset_register = reg,
          .call = call,
          .valid = true};
}

ControlPlan plan(const BlockInstruction &operation) {
  if (at_loop_end(operation)) { return {}; }
  const auto kind = operation.instruction.operation;
  if (kind == Operation::return_subroutine) { return {.returns = true, .valid = true}; }
  if (kind == Operation::relative_branch) { return relative_plan(operation); }
  const bool conditional = kind == Operation::conditional_jump;
  const bool supported = conditional || kind == Operation::call_short ||
                         kind == Operation::call_effective || kind == Operation::jump_effective;
  if (!supported) { return {}; }
  const auto opcode = operation.instruction.opcode;
  const bool short_form =
      conditional ? (opcode & (Word{1} << 18U)) != 0 : kind == Operation::call_short;
  constexpr unsigned absolute_ea = 0x30;
  constexpr unsigned ea_mask = 0x3f;
  if (!short_form && ((opcode >> 8U) & ea_mask) != absolute_ea) { return {}; }
  const auto condition_shift = short_form ? 12U : 0U;
  return {.target = short_form ? opcode & short_mask : operation.instruction.extension,
          .selector = conditional ? (opcode >> condition_shift) & 15U : unconditional,
          .call = conditional ? (opcode & (Word{1} << 16U)) != 0
                              : kind == Operation::call_short || kind == Operation::call_effective,
          .valid = true};
}

void emit_target(x64::Code &code, const ControlPlan &branch) {
  code.constant(Reg::ax, branch.target);
  if (branch.offset_register < agu_registers) {
    code.load(Reg::dx,
              Reg::r12,
              offsetof(Cpu, reg) + offsetof(Registers, r) + (branch.offset_register * sizeof(Word)),
              Width::dword);
    code.binary(op_add, {.destination = Reg::ax, .source = Reg::dx}, Width::dword);
    code.immediate(group_and, Reg::ax, word_mask, Width::dword);
  }
  code.store(Reg::r12, pc_offset, Reg::ax, Width::dword);
}

} // namespace

bool BlockEmitter::is_control_flow(const BlockInstruction &operation) {
  return plan(operation).valid;
}

void BlockEmitter::control_condition(const unsigned selector, x64::Label &untaken) {
  read_status(Reg::ax);
  constexpr std::array<Word, 8> masks{carry, 0, zero, negative, 0, extension, limit, 0};
  const auto mask = masks.at(selector & 7U);
  if (mask != 0) {
    code_.immediate(group_and, Reg::ax, mask, Width::dword);
    code_.jump_if((selector & 8U) != 0 ? Branch::equal : Branch::not_equal, untaken);
    return;
  }
  code_.immediate(group_and, Reg::ax, condition_mask, Width::dword);
  code_.constant(Reg::dx, reinterpret_cast<std::uintptr_t>(conditions.at(selector).data()));
  code_.binary(op_add, {.destination = Reg::ax, .source = Reg::dx});
  code_.load(Reg::ax, Reg::ax, 0, Width::byte);
  code_.immediate(group_cmp, Reg::ax, 0, Width::dword);
  code_.jump_if(Branch::equal, untaken);
}

void BlockEmitter::control_pop() {
  code_.load(Reg::ax, Reg::r12, sp_offset, Width::dword);
  code_.mov({.destination = Reg::cx, .source = Reg::ax}, Width::dword);
  code_.immediate(group_and, Reg::cx, stack_entries - 1, Width::dword);
  code_.shift(shift_left, Reg::cx, 1, Width::dword);
  code_.load_indexed(
      Reg::dx, {.base = Reg::r12, .index = Reg::cx, .offset = stack_offset}, Width::qword);
  code_.immediate(group_sub, Reg::ax, 1, Width::dword);
  code_.immediate(group_and, Reg::ax, stack_pointer_mask, Width::dword);
  code_.store(Reg::r12, sp_offset, Reg::ax, Width::dword);
  code_.memory_immediate(group_sub, Reg::r12, sc_offset, Width::dword, 1);
  code_.shift(shift_right, Reg::dx, word_bits);
  code_.store(Reg::r12, pc_offset, Reg::dx, Width::dword);
}

bool BlockEmitter::boundary_operation(const BlockInstruction &operation, const bool publish) {
  if (bits(operation, publish)) { return true; }
  if (is_loop_setup(operation)) {
    loop_setup(operation);
    return true;
  }
  const auto branch = plan(operation);
  if (!branch.valid) { return false; }
  const auto next = (operation.pc + operation.instruction.words) & word_mask;
  x64::Label untaken;
  x64::Label complete;
  x64::Label slow;
  if (branch.selector != unconditional) { control_condition(branch.selector, untaken); }
  if (branch.call || branch.returns) {
    code_.load(Reg::ax, Reg::r12, sc_offset, Width::dword);
    code_.immediate(
        group_cmp, Reg::ax, branch.call ? static_cast<Word>(stack_entries - 1) : 0U, Width::dword);
    code_.jump_if(branch.call ? Branch::above_equal : Branch::equal, slow);
  }
  if (branch.returns) {
    control_pop();
  } else {
    if (branch.call) {
      code_.constant(Reg::ax, std::uint64_t{next} << word_bits);
      read_status(Reg::dx);
      code_.binary(op_or, {.destination = Reg::ax, .source = Reg::dx});
      loop_push(Reg::ax);
    }
    emit_target(code_, branch);
  }
  if (branch.selector != unconditional) {
    code_.jump(complete);
    code_.bind(untaken);
    code_.store_constant(Reg::r12, pc_offset, Width::dword, next);
  }
  code_.bind(complete);
  retire(operation, false);
  code_.jump(exits_.at(dirty_accumulators()));
  if (branch.call || branch.returns) {
    code_.bind(slow);
    constexpr auto failure = offsetof(NativeContext, failure);
    code_.load(Reg::ax, Reg::r12, sp_offset, Width::dword);
    code_.store(Reg::r13, failure + offsetof(Fault, value), Reg::ax, Width::dword);
    code_.store_constant(Reg::r13,
                         failure + offsetof(Fault, kind),
                         Width::byte,
                         static_cast<Word>(FaultKind::stack));
    code_.store_constant(
        Reg::r13, failure + offsetof(Fault, space), Width::byte, static_cast<Word>(Space::program));
    code_.store_constant(Reg::r13, failure + offsetof(Fault, address), Width::dword, operation.pc);
    code_.store_constant(Reg::r13, failure + offsetof(Fault, pc), Width::dword, operation.pc);
    code_.store_constant(Reg::r13,
                         offsetof(NativeContext, boundary),
                         Width::byte,
                         static_cast<Word>(NativeBoundary::owner));
    code_.store_constant(Reg::r12, offsetof(Cpu, interrupt_inhibited), Width::byte, 0);
    if (branch.returns) {
      code_.store_constant(Reg::r12, pc_offset, Width::dword, next);
    } else {
      emit_target(code_, branch);
    }
    code_.jump(exits_.at(dirty_accumulators()));
  }
  return true;
}
} // namespace dsp56300::detail
