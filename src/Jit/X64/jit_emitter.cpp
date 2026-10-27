// SPDX-License-Identifier: GPL-3.0-only
#include "jit_emitter.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr auto status_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
constexpr auto la_offset = offsetof(Cpu, reg) + offsetof(Registers, la);
constexpr std::array register_offsets{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
constexpr unsigned op_add = 0x01, op_sub = 0x29, op_or = 0x09, op_xor = 0x31;
constexpr unsigned group_add = 0, group_and = 4, group_sub = 5, group_cmp = 7;
constexpr unsigned shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_overflow = 0x90, condition_zero = 0x94, condition_below = 0x92;
constexpr unsigned low_word_shift = 40, high_word_shift = 16, aligned_shift = 8;
constexpr unsigned product_shift = padding_bits + 1, carry_overflow_multiplier = overflow | limit;
constexpr Word unsupported_modes =
    compatibility_mode | multiply_mode | arithmetic_16 | arithmetic_saturation;
constexpr std::array<std::uint8_t, 2> call_ax{0xff, 0xd0};
constexpr std::array<std::uint8_t, 3> compare_ax_one{0x83, 0xf8, 0x01};
constexpr std::array<std::uint8_t, 2> borrow_ax_ax{0x19, 0xc0};
constexpr std::array<std::uint8_t, 3> increment_retired{0x41, 0xff, 0xc7};
constexpr std::array<std::uint8_t, 4> entry_landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr std::array<std::uint8_t, 9> save_nonvolatile{
    0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x55};
constexpr std::array<std::uint8_t, 10> restore_and_return{
    0x5d, 0x41, 0x5f, 0x41, 0x5e, 0x41, 0x5d, 0x41, 0x5c, 0xc3};

enum AluByte : std::uint8_t {
  alu_clear = 0x13,
  alu_round = 0x11,
  alu_x0 = 0x41,
  alu_y0 = 0x51,
  alu_x1 = 0x61,
  alu_y1 = 0x71,
  alu_abs = 0x26,
  alu_neg = 0x36,
  alu_mask = 0xf7,
  op_test = 0x85
};

constexpr Word if_mask = 0xffe000, if_group = 0x202000;
static_assert(std::is_standard_layout_v<Cpu> && std::is_standard_layout_v<NativeContext>);
static_assert(offsetof(Cpu, interrupt_inhibited) == offsetof(Cpu, repeat_completed) + 1);
}

JitEmitter::JitEmitter(const JitRuntime runtime, const bool optimize)
    : runtime_(runtime), optimize_(optimize) {}

bool JitEmitter::has_moves(const NativeOp &operation) {
  return operation.instruction.operation == Operation::parallel &&
         (operation.instruction.opcode & upper_move_mask) != no_move_group;
}

bool JitEmitter::lowerable(const NativeOp &operation) const {
  if (!optimize_) { return false; }
  const auto &decoded = operation.instruction;
  if (decoded.operation == Operation::nop) { return true; }
  if ((operation.mode & unsupported_modes) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down)) {
    return false;
  }
  if (decoded.operation == Operation::immediate_alu) { return is_addition(operation); }
  if (decoded.operation != Operation::parallel) { return false; }
  const auto opcode = decoded.opcode;
  if ((opcode & if_mask) == if_group) { return false; }
  // Exclude all PCU register operands: they can modify control/stack state.
  if ((opcode & memory_group_mask) == memory_group && (opcode & long_selector_mask) != 0) {
    const auto reg = ((opcode >> 16) & 7) | ((opcode >> 17) & high_register_mask);
    if (reg >= m_register_base) { return false; }
  }
  const auto alu = opcode & byte_mask;
  if (alu == 0) { return plan_moves(decoded).count != 0; }
  if (is_addition(operation)) { return true; }
  if ((alu & 0x80U) != 0) { return true; }
  const auto operation_code = alu & alu_mask;
  return operation_code == alu_clear || operation_code == 0x01 || operation_code == alu_x0 ||
         operation_code == alu_y0 || operation_code == alu_x1 || operation_code == alu_y1 ||
         operation_code == alu_abs || operation_code == alu_neg || operation_code == alu_round;
}

void JitEmitter::value(const Reg destination, const unsigned index, const bool latched) {
  code_.load(destination,
             latched ? Reg::bp : Reg::r12,
             latched ? index * sizeof(std::uint64_t)
                     : offsetof(Cpu, reg) + register_offsets.at(index));
}

void JitEmitter::input(const Reg destination,
                       const unsigned selector,
                       const bool destination_b,
                       const bool latched) {
  if (selector < 2) {
    value(destination, destination_b ? 0 : 1, latched);
    return;
  }
  const auto bank = selector == 2 || selector == 3 ? selector : (selector & 1U) + 2;
  value(destination, bank, latched);
  if (selector < 4) {
    code_.shift(shift_left, destination, high_word_shift);
    code_.shift(shift_arithmetic, destination, aligned_shift);
  } else {
    code_.shift(shift_left, destination, selector < 6 ? low_word_shift : high_word_shift);
    if (selector < 6) {
      code_.shift(shift_arithmetic, destination, aligned_shift);
    } else {
      code_.shift(shift_arithmetic, destination, low_word_shift);
      code_.shift(shift_left, destination, main_shift);
    }
  }
}

void JitEmitter::helper(const JitHelper function, const NativeOp &operation) {
  code_.load(Reg::di, Reg::r13, offsetof(NativeContext, runtime) + offsetof(JitRuntime, owner));
  code_.constant(Reg::si, reinterpret_cast<std::uintptr_t>(&operation));
  code_.constant(Reg::ax, std::bit_cast<std::uintptr_t>(function));
  code_.raw(call_ax);
}

void JitEmitter::loop_guard(const NativeOp &operation) {
  code_.load(Reg::ax, Reg::r12, status_offset, Width::dword);
  code_.immediate(group_and, Reg::ax, loop_flag, Width::dword);
  // An inactive loop gets an impossible LA value, avoiding an extra branch.
  code_.raw(compare_ax_one); // CMP EAX,1; inactive gives CF=1.
  code_.raw(borrow_ax_ax);   // SBB EAX,EAX: inactive=-1, active=0.
  code_.load(Reg::dx, Reg::r12, la_offset, Width::dword);
  code_.binary(op_or, {.destination = Reg::ax, .source = Reg::dx}, Width::dword);
  code_.immediate(group_cmp, Reg::ax, operation.pc + operation.instruction.words - 1, Width::dword);
  code_.exit_if(Branch::equal);
}

void JitEmitter::arithmetic_flags(const Word mode, const bool carry_changed) {
  constexpr unsigned normal_integer_shift = 55;
  constexpr unsigned negative_shift = 60;
  const unsigned shift = normal_integer_shift + ((mode & scale_down) != 0 ? 1U : 0U) -
                         ((mode & scale_up) != 0 ? 1U : 0U);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}); // TEST result,result.
  code_.set(condition_zero, Reg::r8);
  code_.shift(shift_left, Reg::r8, 2, Width::dword);
  code_.mov({.destination = Reg::r9, .source = Reg::ax});
  code_.shift(shift_right, Reg::r9, negative_shift);
  code_.immediate(group_and, Reg::r9, negative, Width::dword);
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  code_.mov({.destination = Reg::r9, .source = Reg::ax});
  code_.shift(shift_right, Reg::r9, shift);
  code_.immediate(group_sub, Reg::r9, 1);
  const auto maximum_integer = static_cast<Word>(~std::uint64_t{0} >> shift);
  code_.immediate(group_cmp, Reg::r9, maximum_integer - 1);
  code_.set(condition_below, Reg::r9);
  code_.shift(shift_left, Reg::r9, 5, Width::dword);
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  code_.mov({.destination = Reg::r9, .source = Reg::ax});
  code_.shift(shift_left, Reg::r9, 1);
  code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::ax});
  code_.shift(shift_right, Reg::r9, shift);
  code_.immediate(6, Reg::r9, 1, Width::dword); // XOR inverts the U bit.
  code_.immediate(group_and, Reg::r9, 1, Width::dword);
  code_.shift(shift_left, Reg::r9, 4, Width::dword);
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  code_.constant(Reg::r11, carry_overflow_multiplier);
  code_.multiply({.destination = Reg::r10, .source = Reg::r11});
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r10}, Width::dword);
  if (carry_changed) {
    code_.binary(op_or, {.destination = Reg::r8, .source = Reg::dx}, Width::dword);
  }
  constexpr Word changed = negative | zero | extension | unnormalized | overflow;
  code_.memory_immediate(
      group_and, Reg::r12, status_offset, Width::dword, ~(changed | (carry_changed ? carry : 0)));
  code_.load(Reg::r9, Reg::r12, status_offset, Width::dword);
  code_.binary(op_or, {.destination = Reg::r8, .source = Reg::r9}, Width::dword);
  code_.store(Reg::r12, status_offset, Reg::r8, Width::dword);
}

void JitEmitter::multiply(const NativeOp &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const auto pair = pairs.at((opcode >> 4) & 7);
  const auto operand = [this, &operation](const Reg reg, const unsigned selector) {
    value(reg, selector < 2 ? 2U : 3U, has_moves(operation));
    code_.shift(shift_left, reg, (selector & 1U) == 0 ? low_word_shift : high_word_shift);
    code_.shift(shift_arithmetic, reg, low_word_shift);
  };
  operand(Reg::ax, pair[0]);
  operand(Reg::dx, pair[1]);
  code_.multiply({.destination = Reg::ax, .source = Reg::dx});
  code_.shift(shift_left, Reg::ax, product_shift);
  if ((opcode & 4U) != 0) { code_.unary(3, Reg::ax); }
  if ((opcode & 2U) != 0) {
    value(Reg::dx, (opcode & 8U) != 0 ? 1U : 0U, has_moves(operation));
    code_.binary(op_add, {.destination = Reg::ax, .source = Reg::dx});
    code_.set(condition_overflow, Reg::r10);
  } else {
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  }
  if ((opcode & 1U) != 0) { round(operation.mode); }
  code_.store(Reg::r12, register_offsets.at((opcode & 8U) != 0 ? 1U : 0U), Reg::ax);
  arithmetic_flags(operation.mode);
}

void JitEmitter::round(const Word mode) {
  const unsigned cleared =
      main_shift + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  const auto half = std::uint64_t{1} << (cleared - 1);
  if ((mode & rounding_mode) != 0) {
    code_.constant(Reg::r11, half);
  } else {
    code_.mov({.destination = Reg::dx, .source = Reg::ax});
    code_.shift(shift_right, Reg::dx, cleared);
    code_.immediate(group_and, Reg::dx, 1, Width::dword);
    code_.shift(shift_left, Reg::dx, padding_bits);
    code_.constant(Reg::r11, half - (std::uint64_t{1} << padding_bits));
    code_.binary(op_add, {.destination = Reg::r11, .source = Reg::dx});
  }
  code_.binary(op_add, {.destination = Reg::ax, .source = Reg::r11});
  code_.set(condition_overflow, Reg::dx);
  code_.binary(op_or, {.destination = Reg::r10, .source = Reg::dx}, Width::dword);
  code_.shift(shift_right, Reg::ax, cleared);
  code_.shift(shift_left, Reg::ax, cleared);
}

void JitEmitter::native(const NativeOp &operation) {
  if (operation.instruction.operation == Operation::nop) { return; }
  if (is_addition(operation)) {
    addition(operation);
    return;
  }
  const auto opcode = operation.instruction.opcode & byte_mask;
  if (opcode == 0) { return; }
  const auto kind = opcode & alu_mask;
  const bool destination_b = (opcode & 8U) != 0;
  const auto destination = register_offsets.at(destination_b ? 1U : 0U);
  if ((opcode & 0x80U) != 0) {
    multiply(operation);
    return;
  }
  if (kind == alu_clear) {
    code_.binary(op_xor, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
  } else if (kind == alu_round) {
    value(Reg::ax, destination_b ? 1U : 0U, has_moves(operation));
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
    round(operation.mode);
  } else if ((kind & 7U) == 1) {
    input(Reg::ax, (opcode >> 4) & 7, destination_b, has_moves(operation));
    code_.store(Reg::r12, destination, Reg::ax);
    return;
  } else {
    value(Reg::ax, destination_b ? 1U : 0U, has_moves(operation));
    if (kind == alu_abs) {
      code_.mov({.destination = Reg::dx, .source = Reg::ax});
      code_.shift(shift_arithmetic, Reg::dx, host_sign_bit);
      code_.binary(op_xor, {.destination = Reg::ax, .source = Reg::dx});
      code_.binary(op_sub, {.destination = Reg::ax, .source = Reg::dx});
    } else {
      code_.unary(3, Reg::ax);
    }
    code_.set(condition_overflow, Reg::r10);
  }
  code_.store(Reg::r12, destination, Reg::ax);
  arithmetic_flags(operation.mode);
}

void JitEmitter::retire(const NativeOp &operation) {
  code_.store_constant(
      Reg::r12, pc_offset, Width::dword, (operation.pc + operation.instruction.words) & word_mask);
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::word, 0);
  code_.memory_immediate(group_add, Reg::r12, offsetof(Cpu, instructions), Width::qword, 1);
  code_.memory_immediate(
      group_add, Reg::r12, offsetof(Cpu, cycles), Width::qword, operation.instruction.cycles);
  code_.raw(increment_retired); // INC R15D (retired instructions).
  if (has_moves(operation)) {
    code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, dirty));
    code_.memory_immediate(group_cmp, Reg::ax, 0, Width::byte, 0);
    code_.exit_if(Branch::not_equal);
  }
}

std::vector<std::uint8_t> JitEmitter::generate(const std::span<const NativeOp> operations) {
  code_.raw(entry_landing); // ENDBR64.
  code_.raw(save_nonvolatile);
  code_.mov({.destination = Reg::r12, .source = Reg::di});
  code_.mov({.destination = Reg::r13, .source = Reg::si});
  code_.binary(op_xor, {.destination = Reg::r15, .source = Reg::r15}, Width::dword);
  code_.load(Reg::bp, Reg::r13, offsetof(NativeContext, runtime) + offsetof(JitRuntime, latched));
  for (const auto &operation : operations) {
    if (lowerable(operation)) {
      loop_guard(operation);
      if (has_moves(operation)) { moves(operation); }
      native(operation);
      retire(operation);
    } else {
      helper(runtime_.execute, operation);
      code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
      code_.exit_if(Branch::equal);
      code_.raw(increment_retired);
      code_.immediate(group_cmp, Reg::ax, 2, Width::dword);
      code_.exit_if(Branch::not_equal);
    }
  }
  code_.bind_exit();
  code_.mov({.destination = Reg::ax, .source = Reg::r15}, Width::dword);
  code_.raw(restore_and_return);
  return code_.bytes();
}
} // namespace dsp56300::detail
