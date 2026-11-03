// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>

namespace dsp56300 {
using namespace detail;

bool Core::Impl::numeric_alu_supported(const Word opcode) {
  if ((cpu.reg.sr & arithmetic_saturation) == 0) { return numeric_mode_supported(); }
  constexpr Word alu_mask = 0xf7;
  constexpr Word tfr = 1;
  constexpr Word tfr_x0 = 0x41;
  constexpr Word tfr_y0 = 0x51;
  constexpr Word tfr_x1 = 0x61;
  constexpr Word tfr_y1 = 0x71;
  constexpr Word clear = 0x13;
  constexpr Word rounded = 0x11;
  constexpr Word test = 0x03;
  constexpr Word maximum = 0x1d;
  constexpr Word maximum_magnitude = 0x15;
  constexpr Word absolute = 0x26;
  constexpr Word negate = 0x36;
  constexpr Word shift_right = 0x22;
  constexpr Word shift_left = 0x32;
  constexpr Word complement = 0x17;
  constexpr Word logical_right = 0x23;
  constexpr Word logical_left = 0x33;
  constexpr Word rotate_right = 0x27;
  constexpr Word rotate_left = 0x37;
  constexpr Word add_right = 0x02;
  constexpr Word subtract_right = 0x06;
  constexpr Word add_left = 0x12;
  constexpr Word subtract_left = 0x16;
  const auto operation = opcode & alu_mask;
  const auto selector = (opcode >> 4) & 7;
  const auto kind = opcode & 7;
  const bool sum = selector >= 1 && (kind == 0 || kind == 4);
  const bool carry_sum = (selector == 2 || selector == 3) && (kind == 1 || kind == 5);
  const bool binary_logic = selector >= 4 && (kind == 2 || kind == 3 || kind == 6);
  const bool bfu = binary_logic || operation == complement || operation == logical_right ||
                   operation == logical_left || operation == rotate_right ||
                   operation == rotate_left;
  const bool compare = (selector == 0 || selector >= 4) && (kind == 5 || kind == 7);
  const bool selection =
      operation == test || operation == maximum || operation == maximum_magnitude;
  const bool shifted = operation == add_right || operation == subtract_right ||
                       operation == add_left || operation == subtract_left;
  const bool supported =
      opcode == 0 || (opcode & 0x80U) != 0 || sum || carry_sum || compare || selection ||
      operation == tfr || operation == tfr_x0 || operation == tfr_y0 || operation == tfr_x1 ||
      operation == tfr_y1 || operation == clear || operation == rounded || operation == absolute ||
      operation == negate || operation == shift_right || operation == shift_left || shifted || bfu;
  return numeric_mode_supported(supported ? arithmetic_saturation : 0);
}

bool Core::Impl::limit_arithmetic(std::uint64_t &value, const bool rounded) const {
  if ((cpu.reg.sr & arithmetic_saturation) == 0) { return false; }
  // Table 3-1 observes exactly EXT[7], EXT[0], MSP[23], not all extension bits.
  constexpr unsigned low_extension_bit = 48 + padding_bits;
  constexpr unsigned high_msp_bit = 47 + padding_bits;
  const auto sign = value >> host_sign_bit;
  if (((value >> low_extension_bit) & 1) == sign && ((value >> high_msp_bit) & 1) == sign) {
    return false;
  }
  constexpr std::uint64_t positive = 0x007fffffffffff00;
  constexpr std::uint64_t negative_limit = 0xff80000000000000;
  constexpr std::uint64_t rounded_positive = 0x007fffff00000000;
  value = sign != 0 ? negative_limit : positive;
  if (rounded && sign == 0) { value = rounded_positive; }
  return true;
}

void Core::Impl::saturate_arithmetic(std::uint64_t &value, const bool rounded) {
  if (limit_arithmetic(value, rounded)) { arithmetic_flags(value, true); }
}
} // namespace dsp56300
