// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <cstdint>
#include <string_view>

namespace dsp56300::detail {
enum class Operation : std::uint8_t {
  unsupported,
  parallel,
  nop,
  jump_short,
  jump_effective,
  call_short,
  call_effective,
  return_subroutine,
  return_interrupt,
  and_control,
  or_control,
  repeat_immediate,
  loop_immediate,
  end_loop
};

struct Instruction {
  Word opcode = 0, extension = 0;
  Operation operation = Operation::unsupported;
  std::uint8_t words = 1;
  std::uint8_t cycles = 1;
  bool operator==(const Instruction &) const = default;
};

struct Encoding {
  Word mask = 0, value = 0;

  [[nodiscard]] constexpr bool matches(const Word opcode) const noexcept {
    return (opcode & mask) == value;
  }
};

consteval Encoding encoding(const std::string_view pattern) {
  constexpr unsigned instruction_bits = 24;
  if (pattern.size() != instruction_bits) { throw "Invalid DSP instruction encoding width"; }
  Encoding result;
  for (const char bit : pattern) {
    result.mask <<= 1;
    result.value <<= 1;
    if (bit == '0' || bit == '1') { result.mask |= 1; }
    if (bit == '1') { result.value |= 1; }
  }
  return result;
}

[[nodiscard]] Instruction decode(Word opcode) noexcept;
} // namespace dsp56300::detail
