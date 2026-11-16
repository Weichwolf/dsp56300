// SPDX-License-Identifier: GPL-3.0-only
#include "kernel_host.hpp"

#include "kernel_reference.hpp"

#include <span>

namespace core_kernels {
void host_fir(const std::span<const DspSample> input,
              const std::span<const DspSample, taps> coefficients,
              const std::span<DspSample> output) noexcept {
  fir<dsp_fraction_bits>(input, coefficients, output);
}

void host_biquads(const std::span<const DspSample> input,
                  const Coefficients<dsp_fraction_bits> &coefficients,
                  State<dsp_fraction_bits> &state,
                  const std::span<DspSample> output) noexcept {
  biquads<dsp_fraction_bits>(input, coefficients, state, output);
}
}
