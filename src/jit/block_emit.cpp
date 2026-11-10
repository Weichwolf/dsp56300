// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#if defined(__aarch64__)
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#else
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>
#if !defined(__aarch64__)
#include <limits>
#include <stdexcept>
#endif

namespace dsp56300::detail {
namespace {
struct CycleCosts {
  unsigned maximum = instruction_cycle_bound();
  Word complete = 0;
};

CycleCosts cycle_costs(const std::span<const BlockInstruction> operations) {
  if (operations.empty()) { return {}; }
  unsigned maximum = 1;
  std::uint64_t total = 0;
  for (const auto &operation : operations) {
    const auto cycles = std::max(1U, static_cast<unsigned>(operation.instruction.cycles));
    maximum = std::max(maximum, cycles);
    total += cycles;
  }
  // A DO endpoint can repeat inside the entry. Fused nested regions use the
  // default ISA bound because their retained body includes additional code.
#if defined(__aarch64__)
  const bool bounded = std::ranges::all_of(operations, [](const auto &operation) {
    return repeatable(operation.instruction.operation) && !at_loop_end(operation);
  });
#else
  const bool bounded = !at_loop_end(operations.back()) && !BlockEmitter::closed_jump(operations);
#endif
  const bool finite = bounded && std::in_range<std::int32_t>(total);
  return {.maximum = maximum, .complete = finite ? static_cast<Word>(total) : 0U};
}
}

#if defined(__aarch64__)
std::vector<std::uint8_t> admit_cycles(const std::span<const std::uint8_t> body,
                                       const std::span<const BlockInstruction> operations,
                                       std::size_t *body_offset) {
  using namespace a64;
  Code code;
  Label ready;
  Label empty;
  Label partial;
  Label bounded;
  const auto costs = cycle_costs(operations);
  code.load(Reg::x9, Reg::x1, offsetof(NativeContext, cycle_deadline), Access::doubleword);
  code.add(Reg::x10, Reg::x9, 1, Width::x64);
  code.branch_zero(Reg::x10, ready, Width::x64);
  code.load(Reg::x10, Reg::x0, offsetof(Cpu, cycles), Access::doubleword);
  code.compare(Reg::x9, Reg::x10, Width::x64);
  code.branch_far(Condition::below_equal, empty);
  code.alu(Alu::subtract, Reg::x9, Reg::x9, Reg::x10, Width::x64);
  if (costs.complete != 0) {
    code.constant(Reg::x10, costs.complete);
    code.compare(Reg::x9, Reg::x10, Width::x64);
    code.branch(Condition::no_carry, partial);
    code.constant(Reg::x9, operations.size());
    code.branch(bounded);
    code.bind(partial);
  }
  const auto shift = static_cast<unsigned>(std::bit_width(costs.maximum - 1U));
  if (shift != 0) { code.shift(Shift::right, Reg::x9, Reg::x9, shift, Width::x64); }
  code.constant(Reg::x10, 1);
  code.compare(Reg::x9, Reg::x10, Width::x64);
  code.select(Reg::x9, Reg::x10, Reg::x9, Condition::no_carry, Width::x64);
  code.bind(bounded);
  code.load(Reg::x10, Reg::x1, offsetof(NativeContext, budget));
  code.compare(Reg::x9, Reg::x10, Width::x64);
  code.select(Reg::x9, Reg::x10, Reg::x9, Condition::above, Width::x64);
  code.store(Reg::x9, Reg::x1, offsetof(NativeContext, budget));
  code.bind(ready);
  if (body_offset != nullptr) { *body_offset = code.bytes().size(); }
  code.append(body);
  code.bind(empty);
  code.constant(Reg::x0, 0, Width::w32);
  code.return_();
  return code.bytes();
}

std::vector<std::uint8_t> emit_block(const std::span<const BlockInstruction> operations,
                                     const JitRuntime &runtime,
                                     const std::array<Word, agu_registers> &modifiers) {
  a64::Emitter emitter;
  auto bytes = emitter.generate_chained(operations);
  if (operations.size() == 1) { return a64::select_mac_batch(operations.front(), bytes); }
  a64::Emitter guarded;
  auto fast = guarded.generate_guarded(operations, runtime, modifiers, bytes);
  return fast ? std::move(*fast) : bytes;
}

std::vector<std::uint8_t> emit_repeat(const BlockInstruction &operation,
                                      const JitRuntime & /*runtime*/,
                                      const std::array<Word, agu_registers> & /*modifiers*/) {
  a64::Emitter emitter;
  return emitter.generate_repeat(operation);
}

std::vector<std::uint8_t> emit_vector(const BlockInstruction &operation) {
  a64::Emitter emitter;
  return emitter.generate_vector(operation);
}

#else
std::vector<std::uint8_t> admit_cycles(const std::span<const std::uint8_t> body,
                                       const std::span<const BlockInstruction> operations,
                                       std::size_t *body_offset) {
  using namespace x64;
  Code code;
  Label ready;
  Label empty;
  Label partial;
  Label bounded;
  const auto costs = cycle_costs(operations);
  constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
  constexpr unsigned compare = 7;
  constexpr unsigned shift_right = 5;
  constexpr unsigned cmp = 0x39;
  constexpr unsigned sub = 0x29;
  constexpr unsigned xor_register = 0x31;
  constexpr unsigned cmov_below = 0x42;
  constexpr unsigned cmov_above = 0x47;
  constexpr std::uint8_t return_instruction = 0xc3;
  code.raw(landing);
  code.load(Reg::ax, Reg::si, offsetof(NativeContext, cycle_deadline));
  code.immediate(compare, Reg::ax, std::numeric_limits<std::uint32_t>::max());
  code.jump_if(Branch::equal, ready);
  code.load(Reg::dx, Reg::di, offsetof(Cpu, cycles));
  code.binary(cmp, {.destination = Reg::ax, .source = Reg::dx});
  code.jump_if(Branch::below_equal, empty);
  code.binary(sub, {.destination = Reg::ax, .source = Reg::dx});
  // A finite complete block may run when its exact base cost fits. Otherwise
  // bound admission by this entry's slowest operation, rather than the whole ISA.
  if (costs.complete != 0) {
    code.immediate(compare, Reg::ax, costs.complete);
    code.jump_if(Branch::below, partial);
    code.constant(Reg::ax, static_cast<Word>(operations.size()));
    code.jump(bounded);
    code.bind(partial);
  }
  const auto shift = static_cast<unsigned>(std::bit_width(costs.maximum - 1U));
  if (shift != 0) { code.shift(shift_right, Reg::ax, shift); }
  code.constant(Reg::dx, 1);
  code.binary(cmp, {.destination = Reg::ax, .source = Reg::dx});
  code.move_if(cmov_below, {.destination = Reg::ax, .source = Reg::dx});
  code.bind(bounded);
  code.load(Reg::cx, Reg::si, offsetof(NativeContext, budget), Width::dword);
  code.binary(cmp, {.destination = Reg::ax, .source = Reg::cx});
  code.move_if(cmov_above, {.destination = Reg::ax, .source = Reg::cx});
  code.store(Reg::si, offsetof(NativeContext, budget), Reg::ax, Width::dword);
  code.bind(ready);
  code.align_entry();
  if (body_offset != nullptr) { *body_offset = code.bytes().size(); }
  code.raw(body);
  code.bind(empty);
  code.binary(xor_register, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  code.raw({return_instruction});
  return code.bytes();
}

std::vector<std::uint8_t> emit_block(const std::span<const BlockInstruction> operations,
                                     const JitRuntime &runtime,
                                     const std::array<Word, agu_registers> &modifiers) {
  if (operations.empty()) { throw std::invalid_argument("Cannot emit an empty DSP block"); }
  if (operations.size() == 1) {
    BlockEmitter single(runtime, modifiers);
    return single.generate(operations);
  }
  BlockEmitter precise(runtime, modifiers);
  const auto checked = precise.generate(operations, ExecutionBounds::precise);
  BlockEmitter fast(runtime, modifiers);
  if (auto region = fast.generate_guarded(operations, checked)) { return std::move(*region); }
  BlockEmitter whole(runtime, modifiers, RegionMode::ordinary);
  auto full = whole.generate(operations);
  x64::Code code;
  x64::Label interior;
  constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
  constexpr unsigned compare = 7;
  code.raw(landing);
  code.load(x64::Reg::ax, x64::Reg::si, offsetof(NativeContext, budget), x64::Width::dword);
  code.immediate(
      compare, x64::Reg::ax, static_cast<Word>(operations.size() - 1), x64::Width::dword);
  code.jump_if(x64::Branch::below_equal, interior);
  code.align_entry();
  code.raw(full);
  code.align_entry();
  code.bind(interior);
  code.raw(checked);
  return code.bytes();
}

std::vector<std::uint8_t> emit_repeat(const BlockInstruction &operation,
                                      const JitRuntime &runtime,
                                      const std::array<Word, agu_registers> &modifiers) {
  BlockEmitter emitter(runtime, modifiers);
  return emitter.generate_repeat(operation);
}
#endif

}
