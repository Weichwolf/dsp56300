// SPDX-License-Identifier: GPL-3.0-only
#include "jit_emitter.hpp"

#include "architecture.hpp"
#include "dataflow.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <algorithm>
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
constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
constexpr std::array register_offsets{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
constexpr unsigned op_add = 0x01, op_sub = 0x29, op_or = 0x09, op_xor = 0x31, op_cmp = 0x39;
constexpr unsigned group_and = 4, group_cmp = 7;
constexpr unsigned shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_overflow = 0x90;
constexpr unsigned low_word_shift = 40, high_word_shift = 16, aligned_shift = 8;
constexpr unsigned product_shift = padding_bits + 1;
constexpr Word unsupported_modes =
    compatibility_mode | multiply_mode | arithmetic_16 | arithmetic_saturation;
constexpr std::array<std::uint8_t, 2> call_ax{0xff, 0xd0};
constexpr std::array<std::uint8_t, 3> increment_retired{0x41, 0xff, 0xc7};
constexpr std::array<std::uint8_t, 4> entry_landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr std::array<std::uint8_t, 14> save_nonvolatile{
    0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x55, 0x53, 0x48, 0x83, 0xec, 0x08};
constexpr std::array<std::uint8_t, 15> restore_and_return{
    0x48, 0x83, 0xc4, 0x08, 0x5b, 0x5d, 0x41, 0x5f, 0x41, 0x5e, 0x41, 0x5d, 0x41, 0x5c, 0xc3};

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

JitEmitter::JitEmitter(const JitRuntime runtime,
                       const bool optimize,
                       const BatchBackend batch,
                       const std::array<Word, agu_registers> modifiers)
    : runtime_(runtime), modifiers_(modifiers), optimize_(optimize), batch_backend_(batch) {}

bool JitEmitter::has_moves(const NativeOp &operation) {
  return operation.instruction.operation == Operation::parallel &&
         (operation.instruction.opcode & upper_move_mask) != no_move_group;
}

bool JitEmitter::observable_moves(const NativeOp &operation) {
  return has_moves(operation) && !plan_register_move(operation.instruction).valid();
}

bool JitEmitter::lowerable(const NativeOp &operation) const {
  if (!optimize_) { return false; }
  const auto &decoded = operation.instruction;
  if (at_loop_end(operation) && (operation.mode & compatibility_mode) != 0) { return false; }
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
  if (alu == 0) { return plan_moves(decoded).count != 0 || plan_register_move(decoded).valid(); }
  if (is_addition(operation)) { return true; }
  if (is_right_shift(operation)) { return true; }
  if ((alu & 0x80U) != 0) { return true; }
  const auto operation_code = alu & alu_mask;
  return operation_code == alu_clear || operation_code == 0x01 || operation_code == alu_x0 ||
         operation_code == alu_y0 || operation_code == alu_x1 || operation_code == alu_y1 ||
         operation_code == alu_abs || operation_code == alu_neg || operation_code == alu_round;
}

void JitEmitter::current_value(const Reg destination, const unsigned index) {
  if (index < accumulator_dirty_.size() && accumulator_dirty_.at(index)) {
    code_.mov({.destination = destination, .source = index == 0 ? Reg::bp : Reg::bx});
  } else {
    code_.load(destination, Reg::r12, offsetof(Cpu, reg) + register_offsets.at(index));
  }
}

void JitEmitter::store_accumulator(const bool destination_b, const Reg source) {
  const auto index = destination_b ? 1U : 0U;
  code_.mov({.destination = destination_b ? Reg::bx : Reg::bp, .source = source});
  accumulator_dirty_.at(index) = true;
}

Word JitEmitter::dirty_accumulators() const {
  return (accumulator_dirty_[0] ? 1U : 0U) | (accumulator_dirty_[1] ? 2U : 0U);
}

void JitEmitter::flush_accumulators(const Word mask) {
  for (unsigned index = 0; index < accumulator_dirty_.size(); ++index) {
    if ((mask & (Word{1} << index)) != 0) {
      code_.store(Reg::r12,
                  offsetof(Cpu, reg) + register_offsets.at(index),
                  index == 0 ? Reg::bp : Reg::bx);
    }
  }
}

void JitEmitter::exit_if(const Branch condition) {
  code_.jump_if(condition, exits_.at(dirty_accumulators()));
}

void JitEmitter::finish() {
  const auto normal = dirty_accumulators();
  code_.bind(exits_.at(normal));
  flush_accumulators(normal);
  x64::Label returned;
  bool branched = false;
  for (Word mask = 0; mask < exits_.size(); ++mask) {
    if (mask == normal || exits_.at(mask).references.empty()) { continue; }
    if (!branched) {
      code_.jump(returned);
      branched = true;
    }
    code_.bind(exits_.at(mask));
    flush_accumulators(mask);
    code_.jump(returned);
  }
  if (branched) { code_.bind(returned); }
  materialize_retirement();
  materialize_status();
  code_.mov({.destination = Reg::ax, .source = Reg::r15}, Width::dword);
  code_.raw(restore_and_return);
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
  materialize_retirement();
  materialize_status();
  flush_accumulators(dirty_accumulators());
  code_.load(Reg::di, Reg::r13, offsetof(NativeContext, runtime) + offsetof(JitRuntime, owner));
  code_.constant(Reg::si, reinterpret_cast<std::uintptr_t>(&operation));
  code_.constant(Reg::ax, std::bit_cast<std::uintptr_t>(function));
  code_.raw(call_ax);
  initialize_retirement();
  initialize_status();
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
  store_accumulator((opcode & 8U) != 0, Reg::ax);
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
  if (is_right_shift(operation)) {
    right_shift(operation);
    return;
  }
  const auto opcode = operation.instruction.opcode & byte_mask;
  if (opcode == 0) { return; }
  const auto kind = opcode & alu_mask;
  const bool destination_b = (opcode & 8U) != 0;
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
    store_accumulator(destination_b, Reg::ax);
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
  store_accumulator(destination_b, Reg::ax);
  arithmetic_flags(operation.mode);
}

void JitEmitter::retire(const NativeOp &operation, const bool publish) {
  if (at_loop_end(operation)) {
    loop_continue();
  } else if (publish) {
    code_.store_constant(Reg::r12,
                         pc_offset,
                         Width::dword,
                         (operation.pc + operation.instruction.words) & word_mask);
  }
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::word, 0);
  retire_progress(operation);
  code_.raw(increment_retired); // INC R15D (retired instructions).
  if (observable_moves(operation) && (!at_loop_end(operation) || loop_size_ != 0)) {
    code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, dirty));
    code_.memory_immediate(group_cmp, Reg::ax, 0, Width::byte, 0);
    exit_if(Branch::not_equal);
  }
  if (at_loop_end(operation)) {
    if (loop_size_ != 0) {
      loop_backedge();
    } else {
      code_.jump(exits_.at(dirty_accumulators()));
    }
  }
}

std::vector<std::uint8_t> JitEmitter::generate(const std::span<const NativeOp> operations) {
  // Full helpers continue only after checking sequential PC, numeric/loop
  // context, REP state and code validity. The ending itself must be native.
  if (!operations.empty() && at_loop_end(operations.back()) && lowerable(operations.back())) {
    loop_size_ = static_cast<Word>(operations.size());
    loop_pc_ = operations.front().pc;
  }
  packed_retirement_ = admit_packed_retirement(operations);
  retirement_helpers_ = packed_retirement_ && std::ranges::any_of(operations, observable_moves);
  cached_status_ = admit_cached_status();
  cached_inputs_ = admit_cached_inputs(operations);
  code_.raw(entry_landing); // ENDBR64.
  code_.raw(save_nonvolatile);
  code_.mov({.destination = Reg::r12, .source = Reg::di});
  code_.mov({.destination = Reg::r13, .source = Reg::si});
  code_.binary(op_xor, {.destination = Reg::r15, .source = Reg::r15}, Width::dword);
  batch_entry(operations);
  if (cached_status_) {
    initialize_status();
  } else if (!cached_inputs_) {
    code_.load(
        Reg::r14, Reg::r13, offsetof(NativeContext, runtime) + offsetof(JitRuntime, latched));
  }
  initialize_retirement();
  if (mac_batches_) {
    code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
    code_.binary(op_cmp, {.destination = Reg::r15, .source = Reg::ax}, Width::dword);
    exit_if(Branch::equal);
  }
  code_.bind(loop_header_);
  const auto flags = plan_flags(operations);
  for (std::size_t operation_index = 0; operation_index < operations.size(); ++operation_index) {
    const auto &operation = operations[operation_index];
    live_flags_ = flags[operation_index];
    dataflow_ = analyze_dataflow(operation.instruction);
    if (lowerable(operation)) {
      loop_guard(operation);
      if (has_moves(operation)) { moves(operation); }
      for (unsigned index = 0; index < accumulator_dirty_.size(); ++index) {
        if ((dataflow_.move_writes & (Word{1} << index)) != 0) {
          accumulator_dirty_.at(index) = false;
        }
      }
      native(operation);
      retire(operation, publish_pc(operations, operation_index));
    } else {
      helper(runtime_.execute, operation);
      accumulator_dirty_.fill(false);
      code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
      exit_if(Branch::equal);
      code_.raw(increment_retired);
      code_.immediate(group_cmp, Reg::ax, 2, Width::dword);
      exit_if(Branch::not_equal);
    }
  }
  finish();
  // Keep non-linear checks beyond every return, outside the linear hot path.
  emit_ring_guards();
  return code_.bytes();
}
} // namespace dsp56300::detail
