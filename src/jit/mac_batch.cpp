// SPDX-License-Identifier: GPL-3.0-only
#include "mac_batch.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "move.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace dsp56300::detail {
namespace {
constexpr Word allowed_modes = scale_down | scale_up | rounding_mode;
constexpr unsigned integer_shift = 55;
constexpr Word changed_flags = overflow | zero | negative | extension | unnormalized;
constexpr Word alu_mask = 0xf3, ordinary_mac = 0xd2;
constexpr unsigned y_index_shift = 3;
constexpr Word index_mask = 7, destination_bit = 64, subtract_bit = 128;

Word available(const JitMemory memory, const Word address) noexcept {
  const auto bound = std::min(memory.words, std::size_t{io_begin});
  return memory.data == nullptr || address >= bound ? 0 : static_cast<Word>(bound - address);
}

bool overlaps_cpu(const Cpu &cpu, const Word *const words, const Word count) noexcept {
  const auto state = std::bit_cast<std::uintptr_t>(&cpu);
  const auto data = std::bit_cast<std::uintptr_t>(words);
  return data <= state ? state - data < std::uintptr_t{count} * sizeof(Word)
                       : data - state < sizeof(Cpu);
}

[[gnu::always_inline]] inline Word batch_count(const Cpu &cpu,
                                               const NativeContext &context,
                                               const BlockInstruction &operation,
                                               const MacBatchPlan plan) noexcept {
  const auto &reg = cpu.reg;
  if (reg.pc != operation.pc || reg.la != operation.pc || (reg.sr & loop_flag) == 0 ||
      (reg.sr & forever_flag) != 0 ||
      (context.runtime.cacheable_program && (reg.sr & cache_enable) != 0) ||
      (reg.sr & numeric_mode_mask) != operation.mode || reg.lc <= mac_batch_width ||
      reg.lc > word_mask || cpu.repeat_remaining != 0 ||
      cpu.processing != ExecutionState::running ||
      (cpu.debug_flags & debug_interrupt_pending) != 0 ||
      (context.exit_requested != nullptr && *context.exit_requested)) {
    return 0;
  }
  const auto frame = reg.stack[reg.sp & (stack_entries - 1)];
  if (frame >> word_bits != operation.pc) { return 0; }
  for (const auto index : plan.indices) {
    if ((reg.m[index] & modifier_mask) != modifier_mask) { return 0; }
  }
  const auto accumulator = plan.destination_b ? reg.b : reg.a;
  if ((accumulator & ~padding_mask) != 0) { return 0; }
  const auto count =
      std::min({context.budget,
                reg.lc - 1,
                available(context.runtime.memory[0], reg.r[plan.indices[0]] & word_mask),
                available(context.runtime.memory[1], reg.r[plan.indices[1]] & word_mask)});
  const auto whole = count & ~(mac_batch_width - 1);
  if (whole == 0 ||
      overlaps_cpu(
          cpu, context.runtime.memory[0].data + (reg.r[plan.indices[0]] & word_mask), whole) ||
      overlaps_cpu(
          cpu, context.runtime.memory[1].data + (reg.r[plan.indices[1]] & word_mask), whole)) {
    return 0;
  }
  return whole;
}

[[gnu::always_inline]] inline void
commit(Cpu &cpu, const MacBatchPlan plan, const MacBatchState state, const Word count) noexcept {
  auto &reg = cpu.reg;
  const auto aligned = (static_cast<std::uint64_t>(state.accumulator) & accumulator_mask)
                       << padding_bits;
  (plan.destination_b ? reg.b : reg.a) = aligned;
  reg.x = (reg.x & ~std::uint64_t{word_mask}) | state.x;
  reg.y = (reg.y & ~std::uint64_t{word_mask}) | state.y;
  for (const auto index : plan.indices) { reg.r[index] = (reg.r[index] & word_mask) + count; }
  reg.lc -= count;
  const auto shift =
      integer_shift + ((reg.sr & scale_down) != 0 ? 1U : 0U) - ((reg.sr & scale_up) != 0 ? 1U : 0U);
  const auto integer = aligned >> shift;
  Word flags = aligned == 0 ? zero : 0;
  if ((aligned >> host_sign_bit) != 0) { flags |= negative; }
  if (integer != 0 && integer != (~std::uint64_t{0} >> shift)) { flags |= extension; }
  if (((aligned >> shift) & 1U) == ((aligned >> (shift - 1)) & 1U)) { flags |= unnormalized; }
  if (state.limited) { flags |= limit; }
  if (state.overflowed) { flags |= overflow; }
  reg.sr = (reg.sr & ~changed_flags) | flags;
  cpu.repeat_completed = false;
  cpu.interrupt_inhibited = false;
  cpu.instructions = wrapped_add(cpu.instructions, count);
  cpu.cycles = wrapped_add(cpu.cycles, count);
}
}

std::optional<MacBatchPlan> plan_mac_batch(const BlockInstruction &operation) {
  const auto &decoded = operation.instruction;
  if (decoded.operation != Operation::parallel || decoded.words != 1 || decoded.cycles != 1 ||
      !at_loop_end(operation) || (decoded.opcode & alu_mask) != ordinary_mac ||
      (operation.mode & ~allowed_modes) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down)) {
    return std::nullopt;
  }
  const auto moves = plan_moves(decoded);
  if (moves.count != 2) { return std::nullopt; }
  for (unsigned port = 0; port < moves.count; ++port) {
    const auto &move = moves.transfers[port];
    if (!move.load || move.reg != (port == 0 ? x0_register : y0_register) ||
        move.address.mode != 3 || move.address.index >= agu_registers) {
      return std::nullopt;
    }
  }
  if (moves.transfers[0].address.index == moves.transfers[1].address.index) { return std::nullopt; }
  return MacBatchPlan{
      .indices = {moves.transfers[0].address.index, moves.transfers[1].address.index},
      .destination_b = (decoded.opcode & 8U) != 0,
      .subtract = (decoded.opcode & 4U) != 0};
}

namespace {
Word execute_mac_batch(Cpu &cpu,
                       NativeContext &context,
                       const BlockInstruction &operation,
                       const Word operands) noexcept {
  const MacBatchPlan plan{
      .indices = {operands & index_mask, (operands >> y_index_shift) & index_mask},
      .destination_b = (operands & destination_bit) != 0,
      .subtract = (operands & subtract_bit) != 0};
  const auto count = batch_count(cpu, context, operation, plan);
  if (count == 0) { return 0; }
  const auto &reg = cpu.reg;
  const MacBatchInput input{
      .x = {context.runtime.memory[0].data + (reg.r[plan.indices[0]] & word_mask), count},
      .y = {context.runtime.memory[1].data + (reg.r[plan.indices[1]] & word_mask), count},
      .subtract = plan.subtract};
  MacBatchState state{.accumulator =
                          std::bit_cast<std::int64_t>(plan.destination_b ? reg.b : reg.a) >>
                          padding_bits,
                      .x = static_cast<Word>(reg.x) & word_mask,
                      .y = static_cast<Word>(reg.y) & word_mask,
                      .limited = (reg.sr & limit) != 0};
  native_mac_batch(state, input);
  commit(cpu, plan, state, count);
  return count;
}
}

std::optional<MacBatchCall> compile_mac_batch(const BlockInstruction &operation) {
  const auto plan = plan_mac_batch(operation);
  if (!plan) { return std::nullopt; }
  return MacBatchCall{.entry = &execute_mac_batch,
                      .operands = plan->indices[0] | (plan->indices[1] << y_index_shift) |
                                  (plan->destination_b ? destination_bit : 0) |
                                  (plan->subtract ? subtract_bit : 0)};
}
} // namespace dsp56300::detail
