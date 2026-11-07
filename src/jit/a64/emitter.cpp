// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/emitter.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"

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
constexpr std::array saved{Reg::x19, Reg::x21, Reg::x23, Reg::x25, Reg::x27};
constexpr std::array held{std::pair{accumulator_a, offsetof(Registers, a)},
                          std::pair{accumulator_b, offsetof(Registers, b)},
                          std::pair{bank_x, offsetof(Registers, x)},
                          std::pair{bank_y, offsetof(Registers, y)}};
static_assert(offsetof(Cpu, reg) == 0);
}

void Emitter::begin() {
  code_.push(Reg::fp, Reg::lr);
  code_.add(Reg::fp, Reg::sp, 0, Width::x64);
  for (const auto reg : saved) {
    code_.push(reg, static_cast<Reg>(static_cast<unsigned>(reg) + 1));
  }
  for (std::size_t pair = 0; pair < Frame::pairs; ++pair) { code_.push(Reg::zr, Reg::x17); }
  code_.move(cpu, Reg::x0);
  code_.move(context, Reg::x1);
  for (const auto &[reg, offset] : held) { code_.load(reg, cpu, offset, Access::doubleword); }
  code_.load(status, cpu, offsetof(Registers, sr));
  code_.load(pc, cpu, offsetof(Registers, pc));
  code_.constant(retired, 0);
  code_.load(remaining, context, offsetof(NativeContext, budget));
}

void Emitter::snapshot() {
  for (std::size_t index = 0; index < held.size(); ++index) {
    code_.store(held[index].first, Reg::sp, index * sizeof(std::uint64_t), Access::doubleword);
  }
}

void Emitter::publish() {
  for (const auto &[reg, offset] : held) { code_.store(reg, cpu, offset, Access::doubleword); }
  code_.store(status, cpu, offsetof(Registers, sr));
  code_.store(pc, cpu, offsetof(Registers, pc));
}

void Emitter::mask(const Reg target, const Reg value, const std::uint64_t bits, const Width width) {
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
  finish_loop(operation);
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
  begin();
  for (const auto &item : operations) {
    if (!prepare_instruction(item)) { continue; }
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
    } else {
      code_.constant(Reg::x17, (item.pc + item.instruction.words) & word_mask);
      code_.compare(pc, Reg::x17);
      code_.branch_far(Condition::not_equal, done_);
    }
  }
  code_.bind(done_);
  publish();
  code_.move(Reg::x0, retired, Width::w32);
  for (std::size_t pair = 0; pair < Frame::pairs; ++pair) { code_.pop(Reg::zr, Reg::x17); }
  for (const auto reg : std::views::reverse(saved)) {
    code_.pop(reg, static_cast<Reg>(static_cast<unsigned>(reg) + 1));
  }
  code_.pop(Reg::fp, Reg::lr);
  code_.return_();
  return code_.bytes();
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
