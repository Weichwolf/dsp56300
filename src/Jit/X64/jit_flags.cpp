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
#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr Word arithmetic_changed = overflow | zero | negative | unnormalized | extension;
constexpr Word observed_flags = arithmetic_changed | carry;
constexpr Word alu_mask = 0xf7, multiply_bit = 0x80;
constexpr unsigned op_or = 0x09, op_xor = 0x31, op_test = 0x85;
constexpr unsigned group_sub = 5, group_cmp = 7, group_and = 4, group_xor = 6;
constexpr unsigned shift_left = 4, shift_right = 5;
constexpr unsigned condition_zero = 0x94, condition_below = 0x92;
constexpr auto status_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
constexpr unsigned normal_integer_shift = 55, negative_shift = 60;

enum TransferCode : std::uint8_t {
  tfr_other = 0x01,
  tfr_x0 = 0x41,
  tfr_y0 = 0x51,
  tfr_x1 = 0x61,
  tfr_y1 = 0x71
};

bool reads_carry(const NativeOp &operation) {
  if (operation.instruction.operation != Operation::parallel) { return false; }
  const auto opcode = operation.instruction.opcode;
  const auto group = (opcode >> 4) & 7U;
  const auto kind = opcode & 7U;
  return (opcode & multiply_bit) == 0 && (group == 2 || group == 3) && (kind == 1 || kind == 5);
}
}

Word JitEmitter::written_flags(const NativeOp &operation) {
  if (operation.instruction.operation == Operation::nop) { return 0; }
  if (is_addition(operation)) { return observed_flags; }
  const auto alu = operation.instruction.opcode & byte_mask;
  const auto kind = alu & alu_mask;
  if (alu == 0 || kind == tfr_other || kind == tfr_x0 || kind == tfr_y0 || kind == tfr_x1 ||
      kind == tfr_y1) {
    return 0;
  }
  return arithmetic_changed;
}

std::vector<Word> JitEmitter::plan_flags(const std::span<const NativeOp> operations) const {
  std::vector<Word> result(operations.size(), observed_flags);
  Word live = observed_flags;
  for (std::size_t i = operations.size(); i != 0; --i) {
    const auto &operation = operations[i - 1];
    if (!lowerable(operation)) {
      live = observed_flags;
      continue;
    }
    // Moves may call observers/fault or invalidate code after their ALU. A
    // loop-ending guard may exit before its ALU. Neither boundary hides flags.
    const bool boundary = has_moves(operation) || at_loop_end(operation);
    if (boundary) { live = observed_flags; }
    result[i - 1] = live;
    live &= ~written_flags(operation);
    if (reads_carry(operation)) { live |= carry; }
    if (boundary) { live = observed_flags; }
  }
  return result;
}

void JitEmitter::arithmetic_flags(const Word mode, const bool carry_changed) {
  const unsigned shift = normal_integer_shift + ((mode & scale_down) != 0 ? 1U : 0U) -
                         ((mode & scale_up) != 0 ? 1U : 0U);
  bool initialized = false;
  const auto merge = [this, &initialized](const Reg source) {
    if (initialized) {
      code_.binary(op_or, {.destination = Reg::r8, .source = source}, Width::dword);
    } else {
      code_.mov({.destination = Reg::r8, .source = source}, Width::dword);
      initialized = true;
    }
  };
  if ((live_flags_ & zero) != 0) {
    code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax});
    code_.set(condition_zero, Reg::r8);
    code_.shift(shift_left, Reg::r8, 2, Width::dword);
    initialized = true;
  }
  if ((live_flags_ & negative) != 0) {
    code_.mov({.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_right, Reg::r9, negative_shift);
    code_.immediate(group_and, Reg::r9, negative, Width::dword);
    merge(Reg::r9);
  }
  if ((live_flags_ & extension) != 0) {
    code_.mov({.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_right, Reg::r9, shift);
    code_.immediate(group_sub, Reg::r9, 1);
    const auto maximum_integer = static_cast<Word>(~std::uint64_t{0} >> shift);
    code_.immediate(group_cmp, Reg::r9, maximum_integer - 1);
    code_.set(condition_below, Reg::r9);
    code_.shift(shift_left, Reg::r9, 5, Width::dword);
    merge(Reg::r9);
  }
  if ((live_flags_ & unnormalized) != 0) {
    code_.mov({.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_left, Reg::r9, 1);
    code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::ax});
    code_.shift(shift_right, Reg::r9, shift);
    code_.immediate(group_xor, Reg::r9, 1, Width::dword);
    code_.immediate(group_and, Reg::r9, 1, Width::dword);
    code_.shift(shift_left, Reg::r9, 4, Width::dword);
    merge(Reg::r9);
  }
  // Sticky L records every overflow, including operations whose ordinary
  // result flags are overwritten before any possible observer or exit.
  code_.constant(Reg::r11, limit | (live_flags_ & overflow));
  code_.multiply({.destination = Reg::r10, .source = Reg::r11});
  merge(Reg::r10);
  if (carry_changed && (live_flags_ & carry) != 0) {
    code_.binary(op_or, {.destination = Reg::r8, .source = Reg::dx}, Width::dword);
  }
  const auto changed = live_flags_ & (arithmetic_changed | (carry_changed ? carry : 0));
  code_.load(Reg::r9, Reg::r12, status_offset, Width::dword);
  if (changed != 0) { code_.immediate(group_and, Reg::r9, ~changed, Width::dword); }
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  code_.store(Reg::r12, status_offset, Reg::r8, Width::dword);
}
} // namespace dsp56300::detail
