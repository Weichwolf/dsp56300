// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "instruction.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail {
enum class BitLocation : std::uint8_t { reg, short_address, effective, port_high, port_low };

struct BitOperand {
  BitLocation location = BitLocation::reg;
  Word selector = 0;
  Space space = Space::x;
};

struct BitAccess {
  BitOperand operand{};
  BitAction action = BitAction::test;
  unsigned bit = 0;
};

inline constexpr Word bit_prefix_mask = 0x0f0000, bit_low_prefix = 0x010000;
inline constexpr Word branch_low_prefix = 0x040000;
inline constexpr Word bit_selector_mask = 0x3f;

constexpr BitOperand bit_operand(const Instruction &decoded) noexcept {
  const auto opcode = decoded.opcode;
  const bool relative = decoded.operation == Operation::bit_branch;
  BitOperand result{.selector = (opcode >> 8) & bit_selector_mask,
                    .space = (opcode & (1U << 6)) != 0 ? Space::y : Space::x};
  if ((opcode & bit_prefix_mask) == (relative ? branch_low_prefix : bit_low_prefix)) {
    result.location = BitLocation::port_low;
    return result;
  }
  const auto group = (opcode >> 14) & 3;
  if (relative) {
    if (group == 2) {
      result.location =
          (opcode & (1U << 7)) != 0 ? BitLocation::short_address : BitLocation::effective;
    } else {
      result.location = (opcode & (1U << 7)) != 0 ? BitLocation::reg : BitLocation::port_high;
    }
  } else {
    constexpr std::array locations{BitLocation::short_address,
                                   BitLocation::effective,
                                   BitLocation::port_high,
                                   BitLocation::reg};
    result.location = locations[group];
  }
  return result;
}

constexpr BitAction bit_action(const Word opcode) noexcept {
  const bool low = (opcode & bit_prefix_mask) == bit_low_prefix;
  const unsigned group = (((opcode >> (low ? 14 : 16)) & 1) * 2) + ((opcode >> 5) & 1);
  constexpr std::array actions{
      BitAction::clear, BitAction::set, BitAction::change, BitAction::test};
  return actions[group];
}

constexpr bool bit_call(const Instruction &decoded) noexcept {
  const bool relative = decoded.operation == Operation::bit_branch;
  const bool low =
      (decoded.opcode & bit_prefix_mask) == (relative ? branch_low_prefix : bit_low_prefix);
  const unsigned low_shift = relative ? 7 : 14;
  return ((decoded.opcode >> (low ? low_shift : 16)) & 1) != 0;
}
} // namespace dsp56300::detail
