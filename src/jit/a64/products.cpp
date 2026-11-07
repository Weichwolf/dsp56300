// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::ordinary_product(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto selector = (instruction.opcode >> 4) & 3U;
  const bool power = instruction.operation == Operation::product_power;
  constexpr std::array immediate_sources{0U, 2U, 1U, 3U};
  constexpr std::array power_sources{3U, 0U, 2U, 1U};
  product_word(Reg::x0, (power ? power_sources : immediate_sources).at(selector));
  const auto coefficient =
      power ? std::int64_t{1} << (word_bits - 1 - ((instruction.opcode >> 8) & five_bit_mask))
            : signed_word(instruction.extension);
  code_.constant(Reg::x1, static_cast<std::uint64_t>(coefficient));
}

void Emitter::mixed_product(const BlockInstruction &operation) {
  const auto opcode = operation.instruction.opcode;
  constexpr std::array<std::array<unsigned, 2>, 16> pairs{{{0, 0},
                                                           {2, 2},
                                                           {1, 0},
                                                           {3, 2},
                                                           {0, 3},
                                                           {2, 0},
                                                           {1, 2},
                                                           {3, 1},
                                                           {1, 1},
                                                           {3, 3},
                                                           {0, 1},
                                                           {2, 3},
                                                           {3, 0},
                                                           {0, 2},
                                                           {2, 1},
                                                           {1, 3}}};
  const auto pair = pairs.at(opcode & 15U);
  product_word(Reg::x0, pair[0]);
  product_word(Reg::x1, pair[1]);
  if ((opcode & (1U << 6)) != 0) { mask(Reg::x0, Reg::x0, word_mask); }
  const bool signed_pair = operation.instruction.operation == Operation::product_double &&
                           (opcode & ((1U << 8) | (1U << 6))) == 0;
  if (!signed_pair) { mask(Reg::x1, Reg::x1, word_mask); }
}

bool Emitter::product(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const bool ordinary = kind == Operation::product_immediate || kind == Operation::product_power;
  const bool double_product = kind == Operation::product_double;
  if (!ordinary && !double_product && kind != Operation::product_mixed) { return false; }
  if (!numeric_mode(operation, true)) { return true; }
  const auto opcode = operation.instruction.opcode;
  if (kind == Operation::product_power) {
    const auto power = (opcode >> 8) & five_bit_mask;
    constexpr unsigned maximum_power = 23;
    if (power == 0 || power > maximum_power) {
      code_.constant(Reg::x0, opcode);
      fault(operation,
            power == 0 ? FaultKind::unsupported_mode : FaultKind::invalid_instruction,
            Reg::x0);
      return true;
    }
  }
  const auto signs = ((opcode >> 7) & 2U) | ((opcode >> 6) & 1U);
  if (double_product && signs == 1) {
    code_.constant(Reg::x0, opcode);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return true;
  }
  if (ordinary) {
    ordinary_product(operation);
  } else {
    mixed_product(operation);
  }
  product_complete(operation);
  return true;
}

void Emitter::product_complete(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const auto opcode = operation.instruction.opcode;
  const bool ordinary = kind == Operation::product_immediate || kind == Operation::product_power;
  const bool double_product = kind == Operation::product_double;
  const bool signed_pair = (opcode & ((1U << 8) | (1U << 6))) == 0;
  code_.multiply(Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.shift(Shift::left, Reg::x0, Reg::x0, padding_bits + 1, Width::x64);
  if ((opcode & (ordinary ? 4U : 16U)) != 0) {
    code_.alu(Alu::subtract, Reg::x0, Reg::zr, Reg::x0, Width::x64);
  }
  const auto target = (opcode & (ordinary ? 8U : 32U)) != 0 ? accumulator_b : accumulator_a;
  const bool accumulate =
      ordinary ? (opcode & 2U) != 0 : double_product || (opcode & (1U << 8)) == 0;
  code_.constant(Reg::x1, 0);
  if (accumulate) { code_.move(Reg::x1, target); }
  if (double_product) {
    code_.shift(Shift::arithmetic, Reg::x1, Reg::x1, word_bits, Width::x64);
    mask(Reg::x1, Reg::x1, padding_mask);
  }
  code_.alu(Alu::add_flags, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  code_.set(Reg::x3, Condition::overflow);
  const bool rounded = ordinary && (opcode & 1U) != 0;
  if (rounded) { round(operation.mode); }
  if (ordinary || (double_product && signed_pair)) { saturate(operation.mode, rounded); }
  arithmetic_flags(operation.mode, false);
  code_.move(target, Reg::x0);
}
}
