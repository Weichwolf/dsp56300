// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail::a64 {
inline constexpr Reg cpu = Reg::x19, context = Reg::x20, accumulator_a = Reg::x21,
                     accumulator_b = Reg::x22, bank_x = Reg::x23, bank_y = Reg::x24,
                     status = Reg::x25, pc = Reg::x26, retired = Reg::x27, remaining = Reg::x28;

enum AluByte : std::uint8_t {
  alu_transfer = 1,
  alu_test = 3,
  alu_round = 0x11,
  alu_clear = 0x13,
  alu_complement = 0x17,
  alu_absolute = 0x26,
  alu_negate = 0x36,
  alu_x0 = 0x41,
  alu_y0 = 0x51,
  alu_x1 = 0x61,
  alu_y1 = 0x71,
  alu_mask = 0xf7
};

class Emitter {
public:
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const BlockInstruction> operations);

private:
  void begin();
  void snapshot();
  void publish();
  void retire(const BlockInstruction &operation);
  void operation(const BlockInstruction &operation);
  void source(Reg value, unsigned selector, bool destination_b);
  void product_word(Reg value, unsigned selector);
  void arithmetic(const BlockInstruction &operation);
  void multiply(const BlockInstruction &operation);
  void unary(const BlockInstruction &operation);
  void logic(const BlockInstruction &operation);
  void round(Word mode);
  void saturate(Word mode, bool rounded);
  void arithmetic_flags(Word mode, bool carry_changed);
  void logical_flags(Reg value);
  void
  complete(const BlockInstruction &operation, bool compare, bool carried, bool rounded = false);
  void mask(Reg target, Reg value, std::uint64_t bits, Width width = Width::x64);
  [[nodiscard]] static Reg destination(Word opcode);
  Code code_;
  Label done_;
};
}
