// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/emitter.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "translation.hpp"

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
  code_.store(Reg::zr, cpu, offsetof(Cpu, repeat_completed), Access::byte);
}

std::vector<std::uint8_t> Emitter::generate(const std::span<const BlockInstruction> operations) {
  if (operations.empty()) { throw std::invalid_argument("Cannot emit an empty A64 DSP block"); }
  begin();
  for (const auto &item : operations) {
    if (item.loop_address != inactive_loop) {
      throw std::invalid_argument("A64 DSP hardware-loop lowering is incomplete");
    }
    code_.branch_zero_far(remaining, done_);
    code_.constant(pc, (item.pc + item.instruction.words) & word_mask);
    code_.store(Reg::zr, cpu, offsetof(Cpu, interrupt_inhibited), Access::byte);
    snapshot();
    flow_ = Flow::sequential;
    operation(item);
    retire(item);
    code_.load(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
    code_.branch_zero_far(Reg::x17, done_, Width::w32, true);
    if (flow_ == Flow::owner || flow_ == Flow::context) {
      code_.branch(done_);
    } else if (flow_ == Flow::control) {
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
}
