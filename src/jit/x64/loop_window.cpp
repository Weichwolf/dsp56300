// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "move.hpp"
#include "region_guards.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::VectorOp;
using x64::Width;
using x64::Xmm;
constexpr unsigned copy_lanes = 4;
constexpr unsigned group_add = 0, group_sub = 5, group_cmp = 7;
constexpr unsigned op_add = 0x01, op_or = 0x09, op_xor = 0x31;
constexpr unsigned shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_not_equal = 0x95, load_word_shift = 40;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
alignas(16) constexpr std::array<Word, copy_lanes> word_masks{
    word_mask, word_mask, word_mask, word_mask};
// Unscaled full-accumulator transfers observe adjacent source bits 22/21 for S.
constexpr Word growth_bit = Word{1} << (word_bits - 3);
alignas(16) constexpr std::array<Word, copy_lanes> growth_masks{
    growth_bit, growth_bit, growth_bit, growth_bit};

bool accumulator_register(const unsigned reg) {
  return reg == a_register || reg == b_register;
}

bool disjoint_banks(const std::array<JitMemory, 2> &memory) {
  const auto x = reinterpret_cast<std::uintptr_t>(memory[0].data);
  const auto y = reinterpret_cast<std::uintptr_t>(memory[1].data);
  return x < y ? memory[0].words <= (y - x) / sizeof(Word)
               : y < x && memory[1].words <= (x - y) / sizeof(Word);
}

struct CopyWindow {
  NativeMoves loads, stores;
};

std::optional<CopyWindow> copy_window(const std::span<const BlockInstruction> operations,
                                      const JitRuntime &runtime,
                                      const std::array<Word, agu_registers> &modifiers) {
  if (operations.size() != 2 || operations.front().mode != 0 || !runtime.windows[0].empty() ||
      !runtime.windows[1].empty() || !disjoint_banks(runtime.memory)) {
    return {};
  }
  CopyWindow result{.loads = plan_moves(operations.front().instruction),
                    .stores = plan_moves(operations.back().instruction)};
  if (result.loads.count != 2 || result.stores.count != 2 ||
      result.loads.transfers[0].reg == result.loads.transfers[1].reg) {
    return {};
  }
  for (unsigned port = 0; port < 2; ++port) {
    const auto &load = result.loads.transfers[port];
    const auto &store = result.stores.transfers[port];
    const auto &instruction = operations[port].instruction;
    if (instruction.words != 1 || instruction.cycles != 1 || !load.load || store.load ||
        load.space != (port == 0 ? Space::x : Space::y) || store.space != load.space ||
        load.address.mode != 4 || store.address.mode != 3 ||
        load.address.index != store.address.index ||
        (modifiers[load.address.index] & modifier_mask) != modifier_mask ||
        !accumulator_register(load.reg) || !accumulator_register(store.reg)) {
      return {};
    }
  }
  return result;
}

void copy_address(x64::Code &code, const NativeMove &load, const JitMemory &memory) {
  const auto address = load.space == Space::x ? Reg::cx : Reg::r8;
  code.load(address, Reg::r12, r_offset + (load.address.index * sizeof(Word)), Width::dword);
  code.shift(shift_left, address, 2);
  code.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(memory.data));
  code.binary(op_add, {.destination = address, .source = Reg::r11});
}

void copy_growth(x64::Code &code, const std::array<Xmm, 2> sources) {
  code.vector_shift(Xmm::x2, sources[0], x64::VectorShift::right, 1, true);
  code.vector(
      VectorOp::bit_xor, {.destination = Xmm::x2, .left = Xmm::x2, .right = sources[0]}, false);
  if (sources[1] != sources[0]) {
    code.vector_shift(Xmm::x7, sources[1], x64::VectorShift::right, 1, true);
    code.vector(
        VectorOp::bit_xor, {.destination = Xmm::x7, .left = Xmm::x7, .right = sources[1]}, false);
    code.vector(
        VectorOp::bit_or, {.destination = Xmm::x2, .left = Xmm::x2, .right = Xmm::x7}, false);
  }
  code.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(growth_masks.data()));
  code.vector_load(Xmm::x10, {.base = Reg::r11}, false);
  code.binary(op_xor, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  code.vector_unary(VectorOp::test, Xmm::x2, Xmm::x10, false);
  code.set(condition_not_equal, Reg::ax); // At least one original transfer sets S.
  code.shift(shift_left, Reg::ax, 7, Width::dword);
  code.binary(op_or, {.destination = Reg::r14, .source = Reg::ax}, Width::dword);
}

void copy_accumulator(x64::Code &code, const NativeMove &load, const Xmm source) {
  code.vector_scalar(Reg::ax, source, 1);
  code.shift(shift_right, Reg::ax, 32);
  code.shift(shift_left, Reg::ax, load_word_shift);
  code.shift(shift_arithmetic, Reg::ax, padding_bits);
  code.mov({.destination = load.reg == a_register ? Reg::bp : Reg::bx, .source = Reg::ax});
}
}

void BlockEmitter::plan_loop_window(const std::span<const BlockInstruction> operations) {
  // A proved copy window has no observers or interior exits. Retain every RAM
  // operation while committing its continuing-loop metadata at the window end.
  coalesced_loop_ = loop_window_ && loop_residency_ && !retirement_helpers_ &&
                    std::ranges::all_of(operations, [](const auto &operation) {
                      return operation.instruction.operation == Operation::parallel &&
                             (operation.instruction.opcode & byte_mask) == 0 &&
                             plan_moves(operation.instruction).count != 0;
                    });
  copy_loop_ = coalesced_loop_ && copy_window(operations, runtime_, modifiers_).has_value();
}

void BlockEmitter::initialize_loop_window() {
  if (!loop_window_) { return; }
  x64::Label rejected;
  code_.store_constant(x64::Reg::sp, window_flag_offset, x64::Width::byte, 0);
  emit_region_window(code_, region_, loop_size_, rejected);
  code_.store_constant(x64::Reg::sp, window_flag_offset, x64::Width::byte, 1);
  code_.bind(rejected);
}

void BlockEmitter::guard_region_header() {
  auto &failed = exits_.at(dirty_accumulators());
  if (!loop_window_) {
    region_guard(code_, x64::Reg::r12, failed);
    return;
  }
  emit_region_loop_guard(code_, x64::Reg::r12, region_, failed);
  x64::Label proved;
  x64::Label checked;
  code_.memory_immediate(group_cmp, x64::Reg::sp, window_flag_offset, x64::Width::byte, 0);
  code_.jump_if(x64::Branch::equal, checked);
  // A retained window proves continuing iterations. Recheck the last one.
  constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
  code_.memory_immediate(group_cmp, x64::Reg::r12, lc_offset, x64::Width::dword, 1);
  code_.jump_if(x64::Branch::not_equal, proved);
  code_.bind(checked);
  emit_region_address_guards(code_, x64::Reg::r12, region_, failed);
  code_.bind(proved);
}

void BlockEmitter::initialize_loop_window_count() {
  if (!coalesced_loop_) { return; }
  using x64::Branch;
  using x64::Reg;
  using x64::Width;
  x64::Label done;
  x64::Label unproved;
  code_.memory_immediate(group_cmp, Reg::sp, window_flag_offset, Width::byte, 0);
  code_.jump_if(Branch::equal, unproved);
  code_.packed_to<32>(Reg::dx, x64::Xmm::x13);
  code_.immediate(group_cmp, Reg::dx, loop_pc_, Width::dword);
  code_.jump_if(Branch::not_equal, unproved);
  code_.load(Reg::ax, Reg::sp, window_count_offset, Width::dword);
  code_.packed_from(x64::Xmm::x7, Reg::ax);
  code_.packed(x64::Packed::unpack_low_qword, x64::Xmm::x13, x64::Xmm::x7);
  code_.jump(done);
  code_.bind(unproved);
  // A retained frame may name a different body even when LA matches.
  code_.store_constant(Reg::sp, window_flag_offset, Width::byte, 0);
  code_.store_constant(Reg::sp, window_count_offset, Width::dword, 0);
  code_.bind(done);
}

void BlockEmitter::loop_window_backedge() {
  using x64::Branch;
  using x64::Reg;
  using x64::Width;
  constexpr unsigned op_sub = 0x29;
  constexpr auto lc = offsetof(Cpu, reg) + offsetof(Registers, lc);
  constexpr auto pc = offsetof(Cpu, reg) + offsetof(Registers, pc);
  x64::Label ordinary;
  x64::Label complete;
  code_.memory_immediate(group_cmp, Reg::sp, window_flag_offset, Width::byte, 0);
  code_.jump_if(Branch::equal, ordinary);
  code_.memory_immediate(group_sub, Reg::sp, window_count_offset, Width::dword, 1);
  code_.jump_if(Branch::not_equal, loop_body_);
  code_.bind(window_exhausted_);
  // The admission excludes the final iteration. Publish its complete prefix
  // before the ordinary header checks the remaining budget, addresses and frames.
  code_.vector_scalar(Reg::ax, x64::Xmm::x13, 1);
  code_.load(Reg::cx, Reg::r12, lc, Width::dword);
  code_.binary(op_sub, {.destination = Reg::cx, .source = Reg::ax}, Width::dword);
  code_.store(Reg::r12, lc, Reg::cx, Width::dword);
  code_.packed_to<32>(Reg::dx, x64::Xmm::x13);
  code_.store(Reg::r12, pc, Reg::dx, Width::dword);
  code_.store_constant(Reg::sp, window_flag_offset, Width::byte, 0);
  code_.jump(complete);
  code_.bind(ordinary);
  loop_continue();
  code_.bind(complete);
  loop_backedge();
}

void BlockEmitter::retire_loop_window(const BlockInstruction &operation, const bool publish) {
  if (publish) {
    code_.store_constant(x64::Reg::r12, offsetof(Cpu, repeat_completed), x64::Width::word, 0);
  }
  retire_progress(operation);
  materialize_pending_accounting();
  loop_window_backedge();
}

void BlockEmitter::emit_loop_copy(const std::span<const BlockInstruction> operations) {
  if (!copy_loop_) { return; }
  const auto plan = copy_window(operations, runtime_, modifiers_);
  if (!plan) { return; }
  x64::Label scalar;
  code_.memory_immediate(group_cmp, Reg::sp, window_count_offset, Width::dword, copy_lanes);
  code_.jump_if(x64::Branch::below, scalar);
  copy_address(code_, plan->loads.transfers[0], runtime_.memory[0]);
  copy_address(code_, plan->loads.transfers[1], runtime_.memory[1]);
  code_.vector_load(Xmm::x8, {.base = Reg::cx}, false);
  code_.vector_load(Xmm::x9, {.base = Reg::r8}, false);
  code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(word_masks.data()));
  code_.vector_load(Xmm::x10, {.base = Reg::r11}, false);
  for (const auto reg : {Xmm::x8, Xmm::x9}) {
    code_.vector(VectorOp::bit_and, {.destination = reg, .left = reg, .right = Xmm::x10}, false);
  }
  const std::array sources{
      plan->stores.transfers[0].reg == plan->loads.transfers[0].reg ? Xmm::x8 : Xmm::x9,
      plan->stores.transfers[1].reg == plan->loads.transfers[0].reg ? Xmm::x8 : Xmm::x9};
  // Entry proves the complete remaining window and disjoint banks. Capture all
  // four original rows before either port's stores; aliases keep the scalar path.
  code_.vector_store({.base = Reg::cx}, sources[0], false);
  code_.vector_store({.base = Reg::r8}, sources[1], false);
  copy_growth(code_, sources);
  copy_accumulator(code_, plan->loads.transfers[0], Xmm::x8);
  copy_accumulator(code_, plan->loads.transfers[1], Xmm::x9);
  for (const auto &load : plan->loads.transfers) {
    code_.memory_immediate(group_add,
                           Reg::r12,
                           r_offset + (load.address.index * sizeof(Word)),
                           Width::dword,
                           copy_lanes);
  }
  code_.store_constant(Reg::r12, offsetof(Cpu, repeat_completed), Width::word, 0);
  code_.immediate(group_add, Reg::r15, copy_lanes * loop_size_, Width::dword);
  code_.memory_immediate(group_sub, Reg::sp, window_count_offset, Width::dword, copy_lanes);
  code_.jump_if(x64::Branch::not_equal, loop_body_);
  code_.jump(window_exhausted_);
  code_.align_entry();
  code_.bind(scalar);
}
} // namespace dsp56300::detail
