// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/emitter.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/dataflow.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::detail::a64 {
namespace {
constexpr Word accumulator_banks = bank_bit(ValueBank::a) | bank_bit(ValueBank::b);
constexpr Word data_banks = bank_bit(ValueBank::x) | bank_bit(ValueBank::y);
constexpr std::array saved{std::pair{Reg::x19, Word{0}},
                           std::pair{Reg::x21, accumulator_banks},
                           std::pair{Reg::x23, data_banks},
                           std::pair{Reg::x25, Word{0}},
                           std::pair{Reg::x27, Word{0}}};
constexpr std::array held{std::pair{accumulator_a, offsetof(Registers, a)},
                          std::pair{accumulator_b, offsetof(Registers, b)},
                          std::pair{bank_x, offsetof(Registers, x)},
                          std::pair{bank_y, offsetof(Registers, y)}};
static_assert(offsetof(Cpu, reg) == 0);

}

Word Emitter::active_banks(const std::span<const BlockInstruction> operations) {
  Word result = 0;
  for (const auto &operation : operations) {
    const auto &instruction = operation.instruction;
    if (instruction.operation == Operation::nop || instruction.operation == Operation::bit_memory) {
      continue;
    }
    if (instruction.operation != Operation::parallel || (instruction.opcode & byte_mask) != 0) {
      return accumulator_banks | data_banks;
    }
    const auto flow = analyze_dataflow(instruction);
    result |= flow.move_reads | flow.move_writes;
  }
  return result;
}

void Emitter::begin() {
  code_.push(Reg::fp, Reg::lr);
  code_.add(Reg::fp, Reg::sp, 0, Width::x64);
  for (const auto &[reg, banks] : saved) {
    if (banks != 0 && (banks & active_banks_) == 0) { continue; }
    code_.push(reg, static_cast<Reg>(static_cast<unsigned>(reg) + 1));
  }
  code_.subtract(Reg::sp, Reg::sp, Frame::pairs * 2 * sizeof(std::uint64_t), Width::x64);
  code_.move(cpu, Reg::x0);
  code_.move(context, Reg::x1);
  for (std::size_t index = 0; index < held.size(); ++index) {
    if ((active_banks_ & (Word{1} << index)) != 0) {
      code_.load(held[index].first, cpu, held[index].second, Access::doubleword);
    }
  }
  code_.load(status, cpu, offsetof(Registers, sr));
  code_.load(pc, cpu, offsetof(Registers, pc));
  code_.constant(retired, 0);
  code_.load(remaining, context, offsetof(NativeContext, budget));
  if (deferred_instructions_) {
    code_.load(Reg::x17, cpu, offsetof(Cpu, instructions), Access::doubleword);
    code_.store(Reg::x17, Reg::sp, Frame::instructions, Access::doubleword);
  }
  if (cycle_cost_ != 0) {
    code_.load(Reg::x17, cpu, offsetof(Cpu, cycles), Access::doubleword);
    code_.store(Reg::x17, Reg::sp, Frame::cycles, Access::doubleword);
  }
}

void Emitter::snapshot() {
  for (std::size_t index = 0; index < held.size(); ++index) {
    if ((old_inputs_ & (Word{1} << index)) == 0) { continue; }
    code_.store(held[index].first, Reg::sp, index * sizeof(std::uint64_t), Access::doubleword);
  }
}

void Emitter::read_old(const Reg result, const unsigned bank) {
  if ((old_inputs_ & (Word{1} << bank)) == 0) {
    code_.move(result, held.at(bank).first);
  } else {
    code_.load(result, Reg::sp, bank * sizeof(std::uint64_t), Access::doubleword);
  }
}

void Emitter::restore() {
  code_.add(Reg::sp, Reg::sp, Frame::pairs * 2 * sizeof(std::uint64_t), Width::x64);
  for (const auto &[reg, banks] : std::views::reverse(saved)) {
    if (banks != 0 && (banks & active_banks_) == 0) { continue; }
    code_.pop(reg, static_cast<Reg>(static_cast<unsigned>(reg) + 1));
  }
  code_.pop(Reg::fp, Reg::lr);
}

void Emitter::publish() {
  if (deferred_instructions_) {
    code_.load(Reg::x17, Reg::sp, Frame::instructions, Access::doubleword);
    code_.alu(Alu::add, Reg::x17, Reg::x17, retired, Width::x64);
    code_.store(Reg::x17, cpu, offsetof(Cpu, instructions), Access::doubleword);
  }
  if (cycle_cost_ != 0) {
    code_.load(Reg::x17, Reg::sp, Frame::cycles, Access::doubleword);
    unsigned shift = 0;
    for (auto cost = cycle_cost_; cost != 0; cost >>= 1) {
      if ((cost & 1U) != 0) { code_.add(Reg::x17, Reg::x17, retired, shift, Width::x64); }
      ++shift;
    }
    code_.store(Reg::x17, cpu, offsetof(Cpu, cycles), Access::doubleword);
  }
  for (std::size_t index = 0; index < held.size(); ++index) {
    if ((active_banks_ & (Word{1} << index)) != 0) {
      code_.store(held[index].first, cpu, held[index].second, Access::doubleword);
    }
  }
  code_.store(status, cpu, offsetof(Registers, sr));
  code_.store(pc, cpu, offsetof(Registers, pc));
}

void Emitter::mask(const Reg target, const Reg value, const std::uint64_t bits, const Width width) {
  if (code_.logical(Alu::bit_and, target, value, bits, width)) { return; }
  code_.constant(Reg::x17, bits);
  code_.alu(Alu::bit_and, target, value, Reg::x17, width);
}

Reg Emitter::destination(const Word opcode) {
  return (opcode & 8U) != 0 ? accumulator_b : accumulator_a;
}

void Emitter::retire(const BlockInstruction &operation) {
  code_.add(retired, retired, 1);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, remaining, remaining, Reg::x17);
  for (const auto &[offset, count] :
       {std::pair{offsetof(Cpu, instructions), 1U},
        std::pair{offsetof(Cpu, cycles), unsigned{operation.instruction.cycles}}}) {
    if (deferred_instructions_ && offset == offsetof(Cpu, instructions)) { continue; }
    if (cycle_cost_ != 0 && offset == offsetof(Cpu, cycles)) { continue; }
    code_.load(Reg::x9, cpu, offset, Access::doubleword);
    code_.add(Reg::x9, Reg::x9, static_cast<std::uint16_t>(count), Width::x64);
    code_.store(Reg::x9, cpu, offset, Access::doubleword);
  }
}

void Emitter::finish_instruction(const BlockInstruction &operation) {
  if (entry_ == Entry::vector) {
    finish_vector(operation);
    retire(operation);
    code_.branch(done_);
    return;
  }
  if (entry_ == Entry::repeat) {
    finish_repeat(operation);
    return;
  }
  if (!is_repeat(operation.instruction.operation)) {
    code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_completed), Access::byte);
  }
  if (entry_ != Entry::chained || flow_ != Flow::sequential || at_loop_end(operation)) {
    finish_loop(operation);
  }
  retire(operation);
}

bool Emitter::prepare_instruction(const BlockInstruction &operation) {
  if (entry_ == Entry::repeat) { code_.bind(repeat_header_); }
  code_.branch_zero_far(remaining, done_);
  if (entry_ == Entry::repeat) {
    code_.load(Reg::x17, cpu, offsetof(Cpu, repeat_remaining));
    code_.branch_zero_far(Reg::x17, done_);
    if (operation.instruction.words != 1 || !repeatable(operation.instruction.operation)) {
      code_.constant(Reg::x0, operation.instruction.opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
      return false;
    }
  }
  if (entry_ == Entry::vector) { return prepare_vector(operation); }
  code_.constant(pc, (operation.pc + operation.instruction.words) & word_mask);
  code_.store(Reg::zr, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
  return true;
}

std::vector<std::uint8_t> Emitter::generate(const std::span<const BlockInstruction> operations) {
  if (operations.empty()) { throw std::invalid_argument("Cannot emit an empty A64 DSP block"); }
  const bool chained = entry_ == Entry::chained;
  active_banks_ = active_banks(operations);
  deferred_instructions_ = entry_ == Entry::repeat || operations.size() >= 4;
  cycle_cost_ = operations.front().instruction.cycles;
  const bool repeating = entry_ == Entry::repeat || (chained && at_loop_end(operations.back()));
  if ((!repeating && operations.size() < 2) ||
      !std::ranges::all_of(operations, [this](const auto &operation) {
        return operation.instruction.cycles == cycle_cost_;
      })) {
    cycle_cost_ = 0;
  }
  Label header;
  Label dispatch;
  begin();
  code_.bind(header);
  for (const auto &item : operations) {
    if (!prepare_instruction(item)) { continue; }
    old_inputs_ = analyze_dataflow(item.instruction).old_inputs;
    snapshot();
    flow_ = Flow::sequential;
    if (entry_ == Entry::vector) {
      vector_operation(item);
    } else {
      operation(item);
    }
    finish_instruction(item);
    code_.load(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
    code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
    if (flow_ == Flow::owner || flow_ == Flow::context) {
      code_.branch(done_);
    } else if (!chained || flow_ == Flow::control || at_loop_end(item)) {
      code_.constant(Reg::x17, (item.pc + item.instruction.words) & word_mask);
      code_.compare(pc, Reg::x17);
      code_.branch_far(Condition::not_equal, chained ? dispatch : done_);
    }
  }
  if (chained) {
    code_.bind(dispatch);
    code_.constant(Reg::x17, operations.front().pc);
    code_.compare(pc, Reg::x17);
    code_.branch_far(Condition::equal, header);
  }
  code_.bind(done_);
  publish();
  code_.move(Reg::x0, retired, Width::w32);
  restore();
  code_.return_();
  if (branch_range_ == BranchRange::full && code_.size() < Code::near_branch_bytes) {
    Emitter compact(BranchRange::near);
    compact.entry_ = entry_;
    return compact.generate(operations);
  }
  return code_.bytes();
}

std::vector<std::uint8_t>
Emitter::generate_chained(const std::span<const BlockInstruction> operations) {
  entry_ = Entry::chained;
  return generate(operations);
}

std::vector<std::uint8_t> Emitter::generate_repeat(const BlockInstruction &operation) {
  entry_ = Entry::repeat;
  return generate(std::span<const BlockInstruction>(&operation, 1));
}

std::vector<std::uint8_t> Emitter::generate_vector(const BlockInstruction &operation) {
  entry_ = Entry::vector;
  return generate(std::span<const BlockInstruction>(&operation, 1));
}
}
