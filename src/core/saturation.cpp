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
  constexpr Word absolute = 0x26;
  constexpr Word negate = 0x36;
  const auto operation = opcode & alu_mask;
  const auto selector = (opcode >> 4) & 7;
  const auto kind = opcode & 7;
  const bool sum = selector >= 1 && (kind == 0 || kind == 4);
  const bool carry_sum = (selector == 2 || selector == 3) && (kind == 1 || kind == 5);
  const bool supported = opcode == 0 || (opcode & 0x80U) != 0 || sum || carry_sum ||
                         operation == tfr || operation == tfr_x0 || operation == tfr_y0 ||
                         operation == tfr_x1 || operation == tfr_y1 || operation == clear ||
                         operation == rounded || operation == absolute || operation == negate;
  return numeric_mode_supported(supported ? arithmetic_saturation : 0);
}

void Core::Impl::saturate_arithmetic(std::uint64_t &value, const bool rounded) {
  if ((cpu.reg.sr & arithmetic_saturation) == 0) { return; }
  // Table 3-1 observes exactly EXT[7], EXT[0], MSP[23], not all extension bits.
  constexpr unsigned low_extension_bit = 48 + padding_bits;
  constexpr unsigned high_msp_bit = 47 + padding_bits;
  const auto sign = value >> host_sign_bit;
  if (((value >> low_extension_bit) & 1) == sign && ((value >> high_msp_bit) & 1) == sign) {
    return;
  }
  constexpr std::uint64_t positive = 0x007fffffffffff00;
  constexpr std::uint64_t negative_limit = 0xff80000000000000;
  constexpr std::uint64_t rounded_positive = 0x007fffff00000000;
  value = sign != 0 ? negative_limit : positive;
  if (rounded && sign == 0) { value = rounded_positive; }
  arithmetic_flags(value, true);
}
} // namespace dsp56300
