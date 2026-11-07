// SPDX-License-Identifier: GPL-3.0-only
#include "jit/emitter.hpp"

#include "architecture.hpp"
#include "dataflow.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/resident.hpp"
#include "move.hpp"
#include "translation.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
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
constexpr unsigned op_add = 0x01, op_adc = 0x11, op_sub = 0x29, op_xor = 0x31, op_cmp = 0x39,
                   op_and = 0x21;
constexpr unsigned group_add = 0, group_adc = 2, group_and = 4, group_sub = 5, group_cmp = 7;
constexpr unsigned shift_left = 4, shift_arithmetic = 7;
constexpr unsigned condition_overflow = 0x90;
constexpr unsigned low_word_shift = 40, high_word_shift = 16, aligned_shift = 8;
constexpr unsigned product_shift = padding_bits + 1;
constexpr Word unsupported_modes = compatibility_mode | multiply_mode | arithmetic_16;
constexpr std::array<std::uint8_t, 2> call_ax{0xff, 0xd0};
constexpr std::array<std::uint8_t, 3> increment_retired{0x41, 0xff, 0xc7};
constexpr std::array<std::uint8_t, 4> entry_landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr std::array<std::uint8_t, 3> test_round_prefix{0x48, 0x0f, 0xba};
constexpr unsigned test_round_register = 0xe0;
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

Word nop_count(const std::span<const BlockInstruction> operations) {
  Word count = 0;
  for (const auto &operation : operations) {
    if (operation.instruction.operation != Operation::nop || operation.instruction.words != 1 ||
        operation.instruction.cycles != 1 || at_loop_end(operation) ||
        operation.pc != ((operations.front().pc + count) & word_mask)) {
      break;
    }
    ++count;
  }
  return count;
}

// EAX is a zero-extended count. XMM0 holds helper-aware packed retirement;
// helper-free packed retirement derives its exact count from R15 at exit.
void nop_progress(x64::Code &code, const bool packed) {
  if (packed) {
    code.packed_from(x64::Xmm::x2, Reg::ax);
    code.packed(x64::Packed::unpack_low_qword, x64::Xmm::x2, x64::Xmm::x2);
    code.packed(x64::Packed::add_qword, x64::Xmm::x0, x64::Xmm::x2);
    return;
  }
  for (const auto offset : {offsetof(Cpu, instructions), offsetof(Cpu, cycles)}) {
    code.load(Reg::dx, Reg::r12, offset);
    code.binary(op_add, {.destination = Reg::dx, .source = Reg::ax});
    code.store(Reg::r12, offset, Reg::dx);
  }
}

}

BlockEmitter::BlockEmitter(const JitRuntime runtime,
                           const std::array<Word, agu_registers> modifiers,
                           const RegionMode region,
                           const LoopResidency residency,
                           const RangeMode ranges,
                           const WordReuseMode words,
                           const LoopWindowMode window)
    : runtime_(runtime),
      modifiers_(modifiers),
      region_mode_(region),
      residency_mode_(residency),
      range_mode_(ranges),
      word_mode_(words),
      window_mode_(window) {}

bool BlockEmitter::has_moves(const BlockInstruction &operation) {
  return operation.instruction.operation == Operation::parallel &&
         (operation.instruction.opcode & upper_move_mask) != no_move_group;
}

bool BlockEmitter::observable_moves(const BlockInstruction &operation) const {
  return region_mode_ != RegionMode::guarded && has_moves(operation) &&
         !plan_register_move(operation.instruction).valid() &&
         !plan_address_immediate(operation.instruction).valid();
}

bool BlockEmitter::lowerable(const BlockInstruction &operation) {
  const auto &decoded = operation.instruction;
  if (at_loop_end(operation) && (operation.mode & compatibility_mode) != 0) { return false; }
  if (decoded.operation == Operation::nop) { return true; }
  if (decoded.operation == Operation::jump_short && !at_loop_end(operation)) { return true; }
  if ((operation.mode & unsupported_modes) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down) ||
      ((operation.mode & arithmetic_saturation) != 0 && !saturation_supported(operation))) {
    return false;
  }
  if (is_loop_setup(operation)) { return true; }
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
  if (alu == 0) {
    return pure_address_update(decoded) || plan_address_immediate(decoded).valid() ||
           plan_moves(decoded).count != 0 || plan_register_move(decoded).valid();
  }
  if (is_logic(operation)) { return true; }
  if (is_addition(operation)) { return true; }
  if (is_right_shift(operation)) { return true; }
  if ((alu & 0x80U) != 0) { return true; }
  const auto operation_code = alu & alu_mask;
  return operation_code == alu_clear || operation_code == 0x01 || operation_code == alu_x0 ||
         operation_code == alu_y0 || operation_code == alu_x1 || operation_code == alu_y1 ||
         operation_code == alu_abs || operation_code == alu_neg || operation_code == alu_round;
}

void BlockEmitter::current_value(const Reg destination, const unsigned index) {
  if (index < accumulator_dirty_.size() && accumulator_dirty_.at(index)) {
    const auto source = index == 0 ? Reg::bp : Reg::bx;
    if (destination != source) { code_.mov({.destination = destination, .source = source}); }
  } else {
    code_.load(destination, Reg::r12, offsetof(Cpu, reg) + register_offsets.at(index));
  }
}

void BlockEmitter::store_accumulator(const bool destination_b, const Reg source) {
  const auto index = destination_b ? 1U : 0U;
  const auto destination = destination_b ? Reg::bx : Reg::bp;
  if (destination != source) { code_.mov({.destination = destination, .source = source}); }
  accumulator_dirty_.at(index) = true;
}

void BlockEmitter::invalidate_moved_accumulators() {
  if (loop_residency_) { return; }
  for (unsigned index = 0; index < accumulator_dirty_.size(); ++index) {
    if ((dataflow_.move_writes & (Word{1} << index)) != 0) { accumulator_dirty_.at(index) = false; }
  }
}

Word BlockEmitter::dirty_accumulators() const {
  return (accumulator_dirty_[0] ? 1U : 0U) | (accumulator_dirty_[1] ? 2U : 0U);
}

void BlockEmitter::flush_accumulators(const Word mask) {
  for (unsigned index = 0; index < accumulator_dirty_.size(); ++index) {
    if ((mask & (Word{1} << index)) != 0) {
      code_.store(Reg::r12,
                  offsetof(Cpu, reg) + register_offsets.at(index),
                  index == 0 ? Reg::bp : Reg::bx);
    }
  }
}

void BlockEmitter::exit_if(const Branch condition) {
  code_.jump_if(condition, exits_.at(dirty_accumulators()));
}

void BlockEmitter::finish() {
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

void BlockEmitter::input(const Reg destination,
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

void BlockEmitter::helper(const JitHelper function, const BlockInstruction &operation) {
  code_.store_constant(Reg::r13,
                       offsetof(NativeContext, boundary),
                       Width::byte,
                       static_cast<Word>(NativeBoundary::owner));
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

void BlockEmitter::multiply(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  const bool add_old = (opcode & 2U) != 0 && !value_proof_.zero_addend;
  // A nonzero MAC still needs the old destination while forming its product.
  const auto result = add_old ? Reg::ax : result_register(operation);
  if (!gpr_product(operation, result)) {
    const auto operands = product_operands(operation.instruction);
    product_word(result, operation, operands.words[0], operands.scaled);
    if (!operands.square) { product_word(Reg::dx, operation, operands.words[1], false); }
    code_.multiply({.destination = result, .source = operands.square ? result : Reg::dx});
    if (!operands.scaled) { code_.shift(shift_left, result, product_shift); }
  }
  if ((opcode & 4U) != 0) { code_.unary(3, result); }
  if (add_old) {
    value(Reg::dx, (opcode & 8U) != 0 ? 1U : 0U, has_moves(operation));
    code_.binary(op_add, {.destination = result, .source = Reg::dx});
    if (value_proof_.raw_overflow) { code_.set(condition_overflow, Reg::r10); }
  }
  if (!value_proof_.raw_overflow || (opcode & 2U) == 0) {
    if (value_proof_.overflow_value()) {
      code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
    }
  }
  if ((opcode & 1U) != 0) { round(operation.mode, result); }
  if ((operation.mode & arithmetic_saturation) != 0 && (opcode & 2U) == 0 &&
      value_proof_.saturation != SaturationProof::none) {
    // Fresh signed24 fractional products, including RND, stay in [-2^47,2^47].
    // Only the positive endpoint needs SM limiting; MAC sums use the general path.
    constexpr auto boundary = std::uint64_t{1} << (double_word_bits - 1 + padding_bits);
    constexpr unsigned equal = 0x94;
    constexpr unsigned move_equal = 0x44;
    code_.constant(Reg::r11, boundary);
    code_.binary(op_cmp, {.destination = result, .source = Reg::r11});
    code_.set(equal, Reg::r10);
    const auto step = std::uint64_t{1} << ((opcode & 1U) != 0 ? main_shift : padding_bits);
    code_.constant(Reg::r11, boundary - step);
    code_.move_if(move_equal, {.destination = result, .source = Reg::r11});

  } else {
    saturate(operation.mode, (opcode & 1U) != 0);
  }
  store_accumulator((opcode & 8U) != 0, result);
  arithmetic_flags(operation.mode, false, result);
}

void BlockEmitter::round(const Word mode, const Reg result) {
  const unsigned cleared =
      main_shift + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  if (value_proof_.rounding_zero_bits >= cleared - padding_bits) { return; }
  const auto half = std::uint64_t{1} << (cleared - 1);
  if ((mode & rounding_mode) != 0) {
    constexpr auto maximum_immediate_half = std::uint64_t{1} << 31;
    if (half <= maximum_immediate_half) {
      // SUB of a negative immediate adds the positive half with the same OF.
      code_.immediate(group_sub, result, static_cast<Word>(~(half - 1)));
    } else {
      code_.constant(Reg::r11, half);
      code_.binary(op_add, {.destination = result, .source = Reg::r11});
    }
  } else {
    // BT result,imm8 supplies the retained LSB to ADC. Since every input has eight
    // zero padding bits, half-1+CF preserves both the rounded value and raw OF.
    code_.raw(test_round_prefix);
    code_.byte(test_round_register | static_cast<unsigned>(result));
    code_.byte(cleared);
    if (half - 1 <= static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
      code_.immediate(group_adc, result, static_cast<Word>(half - 1));
    } else {
      code_.constant(Reg::r11, half - 1);
      code_.binary(op_adc, {.destination = result, .source = Reg::r11});
    }
  }
  if (value_proof_.rounding_overflow) {
    code_.set(condition_overflow, Reg::dx);
    // The two opposite signed wraps cancel in the fused rounded result.
    code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::dx}, Width::dword);
  }
  code_.constant(Reg::r11, ~std::uint64_t{0} << cleared);
  code_.binary(op_and, {.destination = result, .source = Reg::r11});
}

void BlockEmitter::native(const BlockInstruction &operation) {
  constexpr std::array retirement_only{Operation::nop, Operation::jump_short};
  if (std::ranges::find(retirement_only, operation.instruction.operation) !=
      retirement_only.end()) {
    return;
  }
  if (is_logic(operation)) {
    logic(operation);
    return;
  }
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
  const auto result = result_register(operation);
  if (kind == alu_clear) {
    code_.binary(op_xor, {.destination = result, .source = result}, Width::dword);
    if (value_proof_.overflow_value()) {
      code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
    }
  } else if (kind == alu_round) {
    value(result, destination_b ? 1U : 0U, has_moves(operation));
    if (value_proof_.overflow_value()) {
      code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
    }
    round(operation.mode, result);
  } else if ((kind & 7U) == 1) {
    input(result, (opcode >> 4) & 7, destination_b, has_moves(operation));
    store_accumulator(destination_b, result);
    return;
  } else {
    value(result, destination_b ? 1U : 0U, has_moves(operation));
    if (kind == alu_abs) {
      code_.mov({.destination = Reg::dx, .source = result});
      code_.shift(shift_arithmetic, Reg::dx, host_sign_bit);
      code_.binary(op_xor, {.destination = result, .source = Reg::dx});
      code_.binary(op_sub, {.destination = result, .source = Reg::dx});
    } else {
      code_.unary(3, result);
    }
    if (value_proof_.raw_overflow) {
      code_.set(condition_overflow, Reg::r10);
    } else if (value_proof_.overflow_value()) {
      code_.binary(op_xor, {.destination = Reg::r10, .source = Reg::r10}, Width::dword);
    }
  }
  if (kind != alu_clear) { saturate(operation.mode, kind == alu_round); }
  store_accumulator(destination_b, result);
  arithmetic_flags(operation.mode, false, result);
}

void BlockEmitter::retire(const BlockInstruction &operation, const bool publish) {
  if (repeat_mode_) {
    repeat_retire(operation);
    return;
  }
  if (at_loop_end(operation)) {
    loop_continue();
  } else if (publish) {
    constexpr Word short_target = 0xfff;
    const auto next = operation.instruction.operation == Operation::jump_short
                          ? operation.instruction.opcode & short_target
                          : (operation.pc + operation.instruction.words) & word_mask;
    code_.store_constant(Reg::r12, pc_offset, Width::dword, next);
  }
  if (region_mode_ != RegionMode::guarded || publish) {
    code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::word, 0);
  }
  retire_progress(operation);
  if (!coalesced_accounting_) { code_.raw(increment_retired); } // INC R15D.
  if (observable_moves(operation) && (!at_loop_end(operation) || loop_size_ != 0)) {
    code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, exit_requested));
    code_.memory_immediate(group_cmp, Reg::ax, 0, Width::byte, 0);
    exit_if(Branch::not_equal);
  }
  if (at_loop_end(operation)) {
    materialize_pending_accounting();
    if (loop_size_ != 0) {
      loop_backedge();
    } else {
      code_.jump(exits_.at(dirty_accumulators()));
    }
  }
}

std::vector<std::uint8_t> BlockEmitter::generate_repeat(const BlockInstruction &operation) {
  if (operation.loop_address != inactive_loop || operation.instruction.words != 1 ||
      !repeatable(operation.instruction.operation) || !lowerable(operation)) {
    return {};
  }
  repeat_mode_ = true;
  region_mode_ = RegionMode::ordinary;
  const std::array operations{operation};
  return generate_body(operations);
}

void BlockEmitter::repeat_retire(const BlockInstruction &operation) {
  constexpr auto lc = offsetof(Cpu, reg) + offsetof(Registers, lc);
  code_.store_constant(
      Reg::r12, offsetof(Cpu, repeat_opcode), Width::dword, operation.instruction.opcode);
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_loaded), Width::byte, 1);
  code_.store_constant(Reg::r12, offsetof(Cpu, interrupt_inhibited), Width::byte, 0);
  retire_progress(operation);
  code_.raw(increment_retired);
  code_.memory_immediate(group_sub, Reg::r12, offsetof(Cpu, repeat_remaining), Width::dword, 1);
  x64::Label completed;
  code_.jump_if(Branch::equal, completed);
  code_.load(Reg::ax, Reg::r12, offsetof(Cpu, repeat_remaining), Width::dword);
  code_.store(Reg::r12, lc, Reg::ax, Width::dword);
  code_.store_constant(Reg::r12, pc_offset, Width::dword, operation.pc);
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code_.binary(op_cmp, {.destination = Reg::r15, .source = Reg::ax}, Width::dword);
  exit_if(Branch::above_equal);
  // A later RAM guard can fail at the header's zero-dirty exit. Publish the
  // completed iteration before that edge; the rejected body has no effects.
  flush_accumulators(dirty_accumulators());
  code_.jump(loop_header_);
  code_.bind(completed);
  code_.load(Reg::ax, Reg::r12, offsetof(Cpu, repeat_lc), Width::dword);
  code_.store(Reg::r12, lc, Reg::ax, Width::dword);
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_loaded), Width::byte, 0);
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::byte, 1);
  code_.store_constant(Reg::r12, pc_offset, Width::dword, (operation.pc + 1) & word_mask);
  code_.jump(exits_.at(dirty_accumulators()));
}

void BlockEmitter::budget_guard() {
  materialize_pending_accounting();
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code_.binary(op_cmp, {.destination = Reg::r15, .source = Reg::ax}, Width::dword);
  exit_if(Branch::above_equal);
}

void BlockEmitter::emit_nops(const std::span<const BlockInstruction> operations,
                             const bool publish) {
  const auto count = static_cast<Word>(operations.size());
  if (bounds_ == ExecutionBounds::whole) {
    if (coalesced_accounting_) {
      pending_instructions_ += count - 1;
    } else {
      code_.immediate(group_add, Reg::r15, count - 1, Width::dword);
      if (!packed_retirement_ || retirement_helpers_) {
        code_.constant(Reg::ax, count - 1);
        nop_progress(code_, packed_retirement_);
      }
    }
    retire(operations.back(), publish);
    return;
  }
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code_.binary(op_sub, {.destination = Reg::ax, .source = Reg::r15}, Width::dword);
  code_.constant(Reg::dx, count);
  code_.binary(op_cmp, {.destination = Reg::ax, .source = Reg::dx}, Width::dword);
  constexpr unsigned move_above = 0x47;
  code_.move_if(move_above, {.destination = Reg::ax, .source = Reg::dx});
  code_.mov({.destination = Reg::dx, .source = Reg::ax}, Width::dword);
  code_.immediate(group_add, Reg::dx, operations.front().pc, Width::dword);
  code_.immediate(group_and, Reg::dx, word_mask, Width::dword);
  code_.store(Reg::r12, pc_offset, Reg::dx, Width::dword);
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::word, 0);
  code_.binary(op_add, {.destination = Reg::r15, .source = Reg::ax}, Width::dword);
  if (!packed_retirement_ || retirement_helpers_) { nop_progress(code_, packed_retirement_); }
  code_.immediate(group_cmp, Reg::ax, count, Width::dword);
  exit_if(Branch::not_equal);
}

void BlockEmitter::emit_operations(const std::span<const BlockInstruction> operations) {
  const auto flags = plan_flags(operations);
  const auto effects = word_effects(operations);
  const auto populations = word_mode_ != WordReuseMode::disabled
                               ? plan_word_reuse(effects)
                               : std::vector<Word>(operations.size());
  auto gpr_effects = effects;
  std::ranges::transform(
      operations, gpr_effects, gpr_effects.begin(), [](const auto &operation, auto effect) {
        effect.barrier |= has_moves(operation);
        return effect;
      });
  const auto selections = word_mode_ == WordReuseMode::automatic
                              ? plan_gpr_word_reuse(gpr_effects)
                              : std::vector<Word>(operations.size());
  word_valid_ = 0;
  NativeValueRanges ranges;
  for (std::size_t operation_index = 0; operation_index < operations.size(); ++operation_index) {
    const auto &operation = operations[operation_index];
    if (bounds_ == ExecutionBounds::precise) { budget_guard(); }
    const auto nops = !repeat_mode_ ? nop_count(operations.subspan(operation_index)) : 0;
    if (nops > 1) {
      const auto last = operation_index + nops - 1;
      emit_nops(operations.subspan(operation_index, nops), publish_pc(operations, last));
      operation_index = last;
      continue;
    }
    constexpr Word complete_flags = carry | overflow | zero | negative | unnormalized | extension;
    live_flags_ = bounds_ == ExecutionBounds::precise ? complete_flags : flags[operation_index];
    dataflow_ = analyze_dataflow(operation.instruction);
    const auto &effect = effects[operation_index];
    if (effect.barrier) { word_valid_ = 0; }
    // Address generation may overwrite RCX while unaffected SIMD words survive.
    if (has_moves(operation)) { word_valid_ &= ~Word{15}; }
    word_selected_ = selections[operation_index];
    word_populate_ = populations[operation_index] & ~word_selected_;
    prepare_value_proof(operation, ranges);
    if (lowerable(operation)) {
      if (is_loop_setup(operation)) {
        loop_setup(operation);
        continue;
      }
      loop_guard(operation);
      if (has_moves(operation)) { moves(operation); }
      invalidate_moved_accumulators();
      native(operation);
      retire(operation,
             bounds_ == ExecutionBounds::precise || publish_pc(operations, operation_index));
    } else {
      helper(runtime_.execute, operation);
      accumulator_dirty_.fill(false);
      code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
      exit_if(Branch::equal);
      code_.raw(increment_retired);
      code_.immediate(group_cmp, Reg::ax, 2, Width::dword);
      exit_if(Branch::not_equal);
      // Return value two certifies sequential PC, context and all event gates.
      code_.store_constant(Reg::r13,
                           offsetof(NativeContext, boundary),
                           Width::byte,
                           static_cast<Word>(NativeBoundary::none));
    }
    invalidate_word_reuse(effect);
  }
}

std::vector<std::uint8_t>
BlockEmitter::generate_body(const std::span<const BlockInstruction> operations) {
  // Full helpers continue only after checking sequential PC, numeric/loop
  // context, REP state and code validity. The ending itself must be native.
  if (!operations.empty() && at_loop_end(operations.back()) && lowerable(operations.back())) {
    loop_size_ = static_cast<Word>(operations.size());
    loop_pc_ = operations.front().pc;
  }
  packed_retirement_ = admit_packed_retirement(operations);
  // Accumulated base cycles must fit the signed immediate used by native accounting.
  constexpr auto maximum_coalesced_operations = static_cast<std::size_t>(
      std::numeric_limits<std::int32_t>::max() / std::numeric_limits<std::uint8_t>::max());
  coalesced_accounting_ = region_mode_ == RegionMode::guarded && packed_retirement_ &&
                          operations.size() <= maximum_coalesced_operations;
  loop_window_ = coalesced_accounting_ && region_.loop && window_mode_ == LoopWindowMode::automatic;
  retirement_helpers_ =
      packed_retirement_ &&
      std::ranges::any_of(operations, [this](const auto &op) { return observable_moves(op); });
  cached_status_ = admit_cached_status();
  cached_inputs_ = admit_cached_inputs(operations);
  loop_residency_ = region_mode_ == RegionMode::guarded && loop_size_ != 0 && packed_retirement_ &&
                    residency_mode_ != LoopResidency::disabled;
  if (coalesced_accounting_ && !retirement_helpers_) {
    const auto mode = operations.front().mode;
    constexpr unsigned aligned_growth_shift = 53;
    deferred_growth_shift_ = aligned_growth_shift + ((mode & scale_down) != 0 ? 1U : 0U) -
                             ((mode & scale_up) != 0 ? 1U : 0U);
  }
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
  initialize_loop_window();
  initialize_loop_residency();
  // The selector or residency initializer already proved this first body.
  // Backedges still reach the header unless their retained RAM window suffices.
  const bool admitted = entry_admitted_ || loop_residency_;
  if (admitted && loop_size_ != 0) { code_.jump(loop_body_); }
  if (loop_size_ != 0) { code_.align_entry(); }
  code_.bind(loop_header_);
  if (region_mode_ == RegionMode::guarded && (!admitted || loop_size_ != 0)) {
    guard_region_header();
  }
  if (loop_size_ != 0) { code_.align_entry(); }
  code_.bind(loop_body_);
  emit_operations(operations);
  materialize_pending_accounting();
  finish();
  // Keep non-linear checks beyond every return, outside the linear hot path.
  emit_ring_guards();
  return code_.bytes();
}
} // namespace dsp56300::detail
