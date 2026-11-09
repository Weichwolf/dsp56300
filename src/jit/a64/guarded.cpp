// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/dataflow.hpp"
#include "jit/resident.hpp"
#include "jit/value_ranges.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::detail::a64 {
namespace {
constexpr Word observed_flags = overflow | zero | negative | unnormalized | extension | carry;
constexpr Word arithmetic_changed = observed_flags ^ carry;
constexpr Word condition_mask = 0xffe000;
constexpr Word condition_group = 0x202000;

bool reads_carry(const Word opcode) {
  const auto group = (opcode >> 4) & 7U;
  const auto kind = opcode & 7U;
  return (opcode & 0x80U) == 0 && (group == 2 || group == 3) && (kind == 1 || kind == 5);
}
}

bool Emitter::guardable(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation == Operation::nop) { return true; }
  constexpr Word unsupported = compatibility_mode | multiply_mode | arithmetic_16;
  if ((operation.mode & unsupported) != 0 ||
      (operation.mode & (scale_up | scale_down)) == (scale_up | scale_down)) {
    return false;
  }
  const auto kind = instruction.opcode & 7U;
  if (instruction.operation == Operation::immediate_alu) {
    return kind == 0 || kind == 4 || kind == 5 || kind == 2 || kind == 3 || kind == 6;
  }
  if (instruction.operation != Operation::parallel ||
      (instruction.opcode & condition_mask) == condition_group) {
    return false;
  }
  const auto opcode = instruction.opcode & byte_mask;
  const auto code = opcode & alu_mask;
  if (opcode == 0 || (opcode & 0x80U) != 0) { return true; }
  if (code == alu_transfer || code == alu_x0 || code == alu_y0 || code == alu_x1 ||
      code == alu_y1 || code == alu_clear || code == alu_test || code == alu_round ||
      code == alu_absolute || code == alu_negate || code == alu_complement || code == alu_asr) {
    return true;
  }
  const auto selector = (opcode >> 4) & 7U;
  return (selector >= 4 && (kind == 2 || kind == 3 || kind == 6)) ||
         (selector >= 1 && (kind == 0 || kind == 4)) ||
         (kind == 5 && (selector == 0 || selector >= 4)) || reads_carry(opcode);
}

Word Emitter::written_flags(const BlockInstruction &operation) {
  if (operation.instruction.operation == Operation::nop) { return 0; }
  const auto opcode = operation.instruction.opcode & byte_mask;
  const auto code = opcode & alu_mask;
  const auto selector = (opcode >> 4) & 7U;
  const auto kind = opcode & 7U;
  if (operation.instruction.operation == Operation::immediate_alu) {
    return kind == 2 || kind == 3 || kind == 6 ? overflow | zero | negative : observed_flags;
  }
  if ((opcode & 0x80U) != 0) { return arithmetic_changed; }
  if (opcode == 0 || code == alu_transfer || code == alu_x0 || code == alu_y0 || code == alu_x1 ||
      code == alu_y1) {
    return 0;
  }
  if (code == alu_complement || (selector >= 4 && (kind == 2 || kind == 3 || kind == 6))) {
    return overflow | zero | negative;
  }
  if ((opcode & 0x80U) == 0 && ((selector >= 1 && (kind == 0 || kind == 4)) || kind == 5 ||
                                code == alu_asr || reads_carry(opcode))) {
    return observed_flags;
  }
  return arithmetic_changed;
}

std::vector<Word> Emitter::plan_flags(const std::span<const BlockInstruction> operations) {
  std::vector<Word> result(operations.size(), observed_flags);
  Word live = observed_flags;
  for (auto index = operations.size(); index != 0; --index) {
    const auto &operation = operations[index - 1];
    result[index - 1] = live;
    live &= ~written_flags(operation);
    if (operation.instruction.operation == Operation::parallel &&
        reads_carry(operation.instruction.opcode)) {
      live |= carry;
    }
  }
  return result;
}

void Emitter::guarded_retire(const std::span<const BlockInstruction> operations) {
  const auto count = static_cast<std::uint64_t>(operations.size());
  code_.constant(Reg::x17, count);
  code_.alu(Alu::add, retired, retired, Reg::x17);
  code_.alu(Alu::subtract, remaining, remaining, Reg::x17);
  std::uint64_t cycles = 0;
  for (const auto &operation : operations) { cycles += operation.instruction.cycles; }
  for (const auto &[offset, amount] :
       {std::pair{offsetof(Cpu, instructions), count}, std::pair{offsetof(Cpu, cycles), cycles}}) {
    code_.load(Reg::x9, cpu, offset, Access::doubleword);
    code_.constant(Reg::x17, amount);
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x17, Width::x64);
    code_.store(Reg::x9, cpu, offset, Access::doubleword);
  }
}

void Emitter::guarded_body(const std::span<const BlockInstruction> operations) {
  const auto flags = plan_flags(operations);
  NativeValueRanges ranges;
  for (std::size_t index = 0; index < operations.size(); ++index) {
    const auto &item = operations[index];
    live_flags_ = flags[index];
    value_proof_ = ranges.advance(item, true, false);
    old_inputs_ = analyze_dataflow(item.instruction).old_inputs;
    snapshot();
    flow_ = Flow::sequential;
    operation(item);
  }
  const auto &last = operations.back();
  code_.constant(pc, (last.pc + last.instruction.words) & word_mask);
  code_.store(Reg::zr, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
  code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_completed), Access::byte);
  if (region_.loop) { finish_loop(last); }
  guarded_retire(operations);
}

void Emitter::guarded_finish(const std::span<const std::uint8_t> fallback) {
  code_.bind(done_);
  publish();
  Label returning;
  Label ordinary;
  if (!fallback.empty()) {
    code_.branch_zero_far(retired, returning, Width::w32, true);
    code_.move(Reg::x0, cpu);
    code_.move(Reg::x1, context);
    restore();
    code_.branch(ordinary);
    code_.bind(returning);
  }
  code_.move(Reg::x0, retired, Width::w32);
  restore();
  code_.return_();
  if (!fallback.empty()) {
    code_.bind(ordinary);
    code_.append(fallback);
  }
}

std::optional<std::vector<std::uint8_t>>
Emitter::generate_guarded(const std::span<const BlockInstruction> operations,
                          const JitRuntime &runtime,
                          const std::array<Word, agu_registers> &modifiers,
                          const std::span<const std::uint8_t> fallback) {
  region_ = resident_plan(operations, runtime.memory, modifiers);
  if (!region_.valid || !std::in_range<Word>(operations.size()) ||
      !std::ranges::all_of(operations, guardable)) {
    return std::nullopt;
  }
  entry_ = Entry::guarded;
  runtime_ = runtime;
  begin();
  guard_storage();
  for (unsigned port = 0; port < runtime.memory.size(); ++port) {
    code_.constant(port == 0 ? Reg::x14 : Reg::x15,
                   reinterpret_cast<std::uintptr_t>(runtime.memory[port].data));
  }
  Label header;
  Label body;
  code_.bind(header);
  code_.constant(Reg::x9, operations.size());
  code_.compare(remaining, Reg::x9);
  code_.branch_far(Condition::no_carry, done_);
  guard_loop(done_);
  guard_iterations(operations);
  code_.bind(body);
  guarded_body(operations);
  code_.load(Reg::x9, Reg::sp, Frame::iterations);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract_flags, Reg::x9, Reg::x9, Reg::x17);
  code_.store(Reg::x9, Reg::sp, Frame::iterations);
  code_.branch_far(Condition::not_equal, body);
  if (region_.loop) {
    code_.load(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
    code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
    code_.constant(Reg::x17, operations.front().pc);
    code_.compare(pc, Reg::x17);
    code_.branch_far(Condition::equal, header);
  }
  guarded_finish(fallback);
  return code_.bytes();
}
}
