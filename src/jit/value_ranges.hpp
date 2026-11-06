// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "jit.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail {
enum class RangeMode : std::uint8_t { automatic, disabled };
enum class SaturationProof : std::uint8_t { none, canonical, general };

struct NativeValueProof {
  std::array<unsigned, 2> transfer_widths{accumulator_bits, accumulator_bits};
  unsigned width = accumulator_bits;
  unsigned input_zero_bits = 0, rounding_zero_bits = 0;
  bool raw_overflow = true, rounding_overflow = true;
  bool zero_addend = false;
  SaturationProof saturation = SaturationProof::general;

  [[nodiscard]] bool overflow_value() const noexcept {
    return raw_overflow || rounding_overflow || saturation != SaturationProof::none;
  }
};

// Track signed width, low zero bits and conservative positive headroom.
// Width zero is exactly zero; roots admit every architectural raw56 value.
// Unknown effects erase every fact.
class NativeValueRanges {
public:
  void reset() noexcept {
    widths_.fill(accumulator_bits);
    zeros_.fill(0);
    headroom_.fill(0);
  }

  [[nodiscard]] NativeValueProof
  advance(const BlockInstruction &operation, bool native, bool observer);

private:
  void moves(const Instruction &instruction);
  std::array<unsigned, 2> widths_{accumulator_bits, accumulator_bits};
  std::array<unsigned, 2> zeros_{}, headroom_{};
};
} // namespace dsp56300::detail
