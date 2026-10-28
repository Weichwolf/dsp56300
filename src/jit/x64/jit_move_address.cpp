// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned op_add = 0x01, op_sub = 0x29, op_xor = 0x31, op_and = 0x21, op_test = 0x85;
constexpr unsigned op_cmp = 0x39, move_above_equal = 0x43;
constexpr unsigned op_return = 0xc3;
constexpr unsigned group_add = 0, group_and = 4, group_sub = 5, group_cmp = 7;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
constexpr auto n_offset = offsetof(Cpu, reg) + offsetof(Registers, n);
constexpr auto m_offset = offsetof(Cpu, reg) + offsetof(Registers, m);

}

void JitEmitter::address_immediate(const NativeAddressImmediate &move) {
  const bool offset_register = move.destination >= n_register_base;
  const auto index = move.destination - (offset_register ? n_register_base : r_register_base);
  const auto bank = offset_register ? offsetof(Registers, n) : offsetof(Registers, r);
  code_.store_constant(
      Reg::r12, offsetof(Cpu, reg) + bank + (index * sizeof(Word)), Width::dword, move.value);
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, native_moves));
  code_.memory_immediate(group_add, Reg::ax, 0, Width::qword, 1);
}

// Address generation touches CPU registers only; no RAM/I/O address is dereferenced.
void JitEmitter::address_update(const NativeOp &operation) {
  const auto opcode = operation.instruction.opcode;
  const NativeMove update{.address = {.mode = (opcode >> 11) & 3, .index = (opcode >> 8) & 7}};
  const auto offset = update.address.index * sizeof(Word);
  x64::Label slow;
  x64::Label done;
  code_.load(Reg::ax, Reg::r12, m_offset + offset, Width::dword);
  code_.immediate(group_and, Reg::ax, modifier_mask, Width::dword);
  code_.immediate(group_cmp, Reg::ax, modifier_mask, Width::dword);
  code_.jump_if(Branch::not_equal, slow);
  move_delta(update);
  code_.immediate(group_and, Reg::si, word_mask, Width::dword);
  code_.store(Reg::r12, r_offset + offset, Reg::si, Width::dword);
  code_.jump(done);
  code_.bind(slow);
  general_moves(operation);
  code_.bind(done);
}

// Keep prospective R updates in SI/DI until every transfer guard has passed.
// move_update consumes them before those registers capture X/Y data values.
void JitEmitter::move_delta(const NativeMove &move) {
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  const auto next = move.space == Space::x ? Reg::si : Reg::di;
  const auto &form = move.address;
  const auto offset = form.index * sizeof(Word);
  const bool subtract = form.mode == 0 || form.mode == 2 || form.mode == 7;
  code_.load(address, Reg::r12, r_offset + offset, Width::dword);
  code_.mov({.destination = next, .source = address}, Width::dword);
  if (form.mode < 2 || form.mode == 5) {
    code_.load(Reg::r9, Reg::r12, n_offset + offset, Width::dword);
    code_.binary(
        subtract ? op_sub : op_add, {.destination = next, .source = Reg::r9}, Width::dword);
  } else {
    code_.immediate(subtract ? group_sub : group_add, next, 1, Width::dword);
  }
}

// AX holds the matching low16 M. Keep both ports prospective until bounds pass.
void JitEmitter::move_profiled_next(const NativeMove &move, const Word modifier) {
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  const auto next = move.space == Space::x ? Reg::si : Reg::di;
  const auto length = modifier + 1;
  if (std::has_single_bit(length)) {
    code_.constant(Reg::ax, modifier);
    move_ring_delta(move);
    return;
  }
  const auto span_mask = std::bit_ceil(length) - 1;
  move_delta(move);
  code_.mov({.destination = Reg::r9, .source = address}, Width::dword);
  code_.immediate(group_and, Reg::r9, word_mask & ~span_mask, Width::dword);
  code_.immediate(group_add, Reg::r9, length, Width::dword);
  code_.mov({.destination = Reg::ax, .source = next}, Width::dword);
  code_.immediate(group_sub, Reg::ax, length, Width::dword);
  code_.binary(op_cmp, {.destination = next, .source = Reg::r9}, Width::dword);
  code_.move_if(move_above_equal, {.destination = next, .source = Reg::ax});
  code_.immediate(group_and, next, word_mask, Width::dword);
}

void JitEmitter::move_next(const NativeMove &move) {
  const auto next = move.space == Space::x ? Reg::si : Reg::di;
  const auto offset = move.address.index * sizeof(Word);
  code_.load(Reg::ax, Reg::r12, m_offset + offset, Width::dword);
  code_.immediate(group_and, Reg::ax, modifier_mask, Width::dword);
  const auto modifier = modifiers_.at(move.address.index) & modifier_mask;
  auto &guard = ring_guards_.emplace_back();
  guard.move = move;
  guard.profiled = move.address.mode == 3 && modifier > 0 && modifier < multiple_wrap_bit;
  code_.immediate(group_cmp, Reg::ax, guard.profiled ? modifier : modifier_mask, Width::dword);
  code_.jump_if(Branch::not_equal, guard.entry);
  if (guard.profiled) {
    move_profiled_next(move, modifier);
  } else {
    move_delta(move);
    code_.immediate(group_and, next, word_mask, Width::dword);
  }
  code_.bind(guard.resume);
}

// Internal leaf: AX >= $8000; return ZF=1 and AX=mask iff valid, preserving port state.
void JitEmitter::emit_ring_check(x64::Label &entry) {
  x64::Label invalid;
  code_.bind(entry);
  code_.immediate(group_cmp, Reg::ax, multiple_wrap_bit | multiple_reserved_bit, Width::dword);
  code_.jump_if(Branch::above_equal, invalid);
  code_.immediate(group_and, Reg::ax, multiple_wrap_mask, Width::dword);
  code_.immediate(group_cmp, Reg::ax, 0, Width::dword);
  code_.jump_if(Branch::equal, invalid);
  code_.mov({.destination = Reg::r9, .source = Reg::ax}, Width::dword);
  code_.immediate(group_add, Reg::r9, 1, Width::dword);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::r9}, Width::dword);
  code_.raw({op_return}); // RET, retaining the validation flags.
  code_.bind(invalid);
  code_.constant(Reg::r9, 1);
  code_.binary(op_test, {.destination = Reg::r9, .source = Reg::r9}, Width::dword);
  code_.raw({op_return});
}

void JitEmitter::emit_ring_guards() {
  if (ring_guards_.empty()) { return; }

  struct Routine {
    NativeMove move;
    x64::Label entry;
  };

  std::vector<Routine> routines;
  x64::Label check;
  for (auto &guard : ring_guards_) {
    if (!guard.slow.bound) { throw std::logic_error("Unbound native ring fallback"); }
    code_.bind(guard.entry);
    if (guard.profiled) {
      x64::Label non_linear;
      code_.immediate(group_cmp, Reg::ax, modifier_mask, Width::dword);
      code_.jump_if(Branch::not_equal, non_linear);
      move_delta(guard.move);
      code_.immediate(
          group_and, guard.move.space == Space::x ? Reg::si : Reg::di, word_mask, Width::dword);
      code_.jump(guard.resume);
      code_.bind(non_linear);
    }
    // Ordinary modulo/reverse fallback needs neither a call nor R/N loads.
    code_.immediate(group_cmp, Reg::ax, multiple_wrap_bit - 1, Width::dword);
    code_.jump_if(Branch::below_equal, guard.slow);
    const auto matches = [&guard](const auto &candidate) {
      return candidate.move.space == guard.move.space &&
             candidate.move.address.mode == guard.move.address.mode &&
             candidate.move.address.index == guard.move.address.index;
    };
    if (std::ranges::count_if(ring_guards_, matches) > 1) {
      auto routine = std::ranges::find_if(routines, matches);
      if (routine == routines.end()) {
        routines.push_back({.move = guard.move, .entry = {}});
        routine = routines.end() - 1;
      }
      code_.call(routine->entry);
      code_.jump_if(Branch::not_equal, guard.slow);
    } else {
      code_.call(check);
      code_.jump_if(Branch::not_equal, guard.slow);
      move_ring_delta(guard.move);
    }
    code_.jump(guard.resume);
  }
  for (auto &routine : routines) {
    x64::Label done;
    code_.bind(routine.entry);
    code_.call(check);
    code_.jump_if(Branch::not_equal, done);
    move_ring_delta(routine.move);
    code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::r9}, Width::dword);
    code_.bind(done);
    code_.raw({op_return});
  }
  emit_ring_check(check);
}

void JitEmitter::move_ring_delta(const NativeMove &move) {
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  const auto next = move.space == Space::x ? Reg::si : Reg::di;
  move_delta(move);
  code_.binary(op_xor, {.destination = next, .source = address}, Width::dword);
  code_.binary(op_and, {.destination = next, .source = Reg::ax}, Width::dword);
  code_.binary(op_xor, {.destination = next, .source = address}, Width::dword);
}

void JitEmitter::move_guard(const NativeMove &move, x64::Label &slow) {
  if (move.immediate()) { return; }
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  const auto &form = move.address;
  const auto offset = form.index * sizeof(Word);
  if (form.mode == 6) {
    code_.constant(address, form.extra);
  } else {
    if (form.mode != 4) {
      move_next(move);
      if (form.mode == 5 || form.mode == 7) {
        code_.mov({.destination = address, .source = move.space == Space::x ? Reg::si : Reg::di},
                  Width::dword);
      }
    } else {
      code_.load(address, Reg::r12, r_offset + offset, Width::dword);
    }
    code_.immediate(group_and, address, word_mask, Width::dword);
  }
  const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
  const auto bound = static_cast<Word>(std::min(memory.words, std::size_t{io_begin}));
  code_.immediate(group_cmp, address, bound, Width::dword);
  code_.jump_if(Branch::above_equal, slow);
}

void JitEmitter::move_update(const NativeMove &move) {
  const auto &form = move.address;
  if (form.mode == 4 || form.mode == 5 || form.mode == 6) { return; }
  const auto offset = form.index * sizeof(Word);
  code_.store(
      Reg::r12, r_offset + offset, move.space == Space::x ? Reg::si : Reg::di, Width::dword);
}
} // namespace dsp56300::detail
