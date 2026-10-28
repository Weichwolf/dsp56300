// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr std::array<std::uint8_t, 4> power_sources{3, 0, 1, 2};
constexpr std::array<std::array<std::uint8_t, 2>, 16> pairs{{{0, 0},
                                                             {1, 1},
                                                             {2, 0},
                                                             {3, 1},
                                                             {0, 3},
                                                             {1, 0},
                                                             {2, 1},
                                                             {3, 2},
                                                             {2, 2},
                                                             {3, 3},
                                                             {0, 2},
                                                             {1, 3},
                                                             {3, 0},
                                                             {0, 1},
                                                             {1, 2},
                                                             {2, 3}}};
constexpr unsigned maximum_power = 23;

struct Product {
  std::int64_t left = 0, right = 0;
  bool accumulate = false, subtract = false, rounded = false, destination_b = false;
};

Word input(const Registers &state, const unsigned selector) {
  const auto bank = (selector & 1U) == 0 ? state.x : state.y;
  return static_cast<Word>(bank >> ((selector & 2U) == 0 ? 0U : word_bits)) & word_mask;
}

Product immediate_product(const Instruction &decoded, const Registers &state) {
  const auto opcode = decoded.opcode;
  const auto selector = (opcode >> 4) & 3U;
  const bool power = decoded.operation == Operation::product_power;
  const auto source = power ? power_sources[selector] : selector;
  const auto coefficient = power ? std::int64_t{1}
                                       << (word_bits - 1 - ((opcode >> 8) & five_bit_mask))
                                 : signed_word(decoded.extension);
  return {.left = signed_word(input(state, source)),
          .right = coefficient,
          .accumulate = (opcode & 2U) != 0,
          .subtract = (opcode & 4U) != 0,
          .rounded = (opcode & 1U) != 0,
          .destination_b = (opcode & 8U) != 0};
}

Product mixed_product(const Instruction &decoded, const Registers &state) {
  const auto opcode = decoded.opcode;
  const auto pair = pairs[opcode & 15U];
  const Word left = input(state, pair[0]);
  const Word right = input(state, pair[1]);
  const bool double_product = decoded.operation == Operation::product_double;
  const bool signed_pair = double_product && (opcode & ((1U << 8) | (1U << 6))) == 0;
  const bool unsigned_pair = (opcode & (1U << 6)) != 0;
  return {.left = unsigned_pair ? static_cast<std::int64_t>(left) : signed_word(left),
          .right = signed_pair ? static_cast<std::int64_t>(signed_word(right)) : right,
          .accumulate = double_product || (opcode & (1U << 8)) == 0,
          .subtract = (opcode & (1U << 4)) != 0,
          .destination_b = (opcode & (1U << 5)) != 0};
}
}

void Core::Impl::execute_product(const Instruction &decoded) {
  if (!numeric_mode_supported()) { return; }
  const auto opcode = decoded.opcode;
  if (decoded.operation == Operation::product_power) {
    const unsigned power = (opcode >> 8) & five_bit_mask;
    if (power == 0 || power > maximum_power) {
      // Power zero needs separate qualification; do not guess its sign.
      fail(power == 0 ? FaultKind::unsupported_mode : FaultKind::invalid_instruction,
           Space::program,
           current_pc,
           opcode);
      return;
    }
  }
  const unsigned sign_mode = ((opcode >> 7) & 2U) | ((opcode >> 6) & 1U);
  if (decoded.operation == Operation::product_double && sign_mode == 1) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const bool ordinary = decoded.operation == Operation::product_immediate ||
                        decoded.operation == Operation::product_power;
  const auto product =
      ordinary ? immediate_product(decoded, cpu.reg) : mixed_product(decoded, cpu.reg);
  auto &destination = product.destination_b ? cpu.reg.b : cpu.reg.a;
  auto initial = product.accumulate ? destination : 0;
  if (decoded.operation == Operation::product_double) {
    initial = static_cast<std::uint64_t>(std::bit_cast<std::int64_t>(initial) >> word_bits) &
              padding_mask;
  }
  const auto value = 2 * product.left * product.right * (product.subtract ? -1 : 1);
  const auto term = (static_cast<std::uint64_t>(value) & accumulator_mask) << padding_bits;
  destination = wrapped_add(initial, term);
  const bool overflowed = ((initial ^ destination) & (term ^ destination)) >> host_sign_bit != 0;
  arithmetic_flags(destination, overflowed);
  if (product.rounded) {
    round(destination);
    if (overflowed) { cpu.reg.sr |= overflow | limit; }
  }
}
} // namespace dsp56300
