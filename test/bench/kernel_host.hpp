// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "kernel_reference.hpp"

namespace core_kernels {
inline constexpr unsigned dsp_fraction_bits = 23;
using DspSample = Sample<dsp_fraction_bits>;
void host_fir(std::span<const DspSample> input,
              std::span<const DspSample, taps> coefficients,
              std::span<DspSample> output) noexcept;
void host_biquads(std::span<const DspSample> input,
                  const Coefficients<dsp_fraction_bits> &coefficients,
                  State<dsp_fraction_bits> &state,
                  std::span<DspSample> output) noexcept;
}
