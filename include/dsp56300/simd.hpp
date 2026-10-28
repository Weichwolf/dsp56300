// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace dsp56300 {
// Host lane/vector geometry, independent of the DSP's 24/48/56-bit precision.
// A valid shape describes storage; it does not prove ISA availability or speed.
template <unsigned LaneBits, unsigned VectorBits>
concept SimdShape = (LaneBits == 8 || LaneBits == 16 || LaneBits == 32 || LaneBits == 64) &&
                    (VectorBits == 128 || VectorBits == 256) && VectorBits % LaneBits == 0;

template <unsigned LaneBits, unsigned VectorBits = 128>
  requires SimdShape<LaneBits, VectorBits>
struct SimdLayout {
  static constexpr unsigned lane_bits = LaneBits;
  static constexpr unsigned vector_bits = VectorBits;
  static constexpr unsigned lanes = VectorBits / LaneBits;
};
} // namespace dsp56300
