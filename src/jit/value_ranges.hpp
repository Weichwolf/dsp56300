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
  bool raw_overflow = true, rounding_overflow = true;
  bool zero_addend = false;
  SaturationProof saturation = SaturationProof::general;

  [[nodiscard]] bool overflow_value() const noexcept {
    return raw_overflow || rounding_overflow || saturation != SaturationProof::none;
  }
};

// Width zero is exactly zero; otherwise every signed w-bit value is admitted.
// Roots and unknown effects always admit the complete architectural domain.
class NativeValueRanges {
public:
  void reset() noexcept { widths_.fill(accumulator_bits); }

  [[nodiscard]] NativeValueProof advance(const NativeOp &operation, bool native, bool observer);

private:
  void moves(const Instruction &instruction);
  std::array<unsigned, 2> widths_{accumulator_bits, accumulator_bits};
};
} // namespace dsp56300::detail
