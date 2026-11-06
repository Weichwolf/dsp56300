// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <span>

namespace dsp56300::test::fir_host {
enum class HostIsa : unsigned char { scalar, avx2 };

[[nodiscard]] bool avx2_supported() noexcept;

struct Result {
  Registers registers;
  Word output = 0;
};

// Signed-24 products, wrapping signed-56 MACs, sticky overflow, convergent RND,
// scaled/limited output, dual linear AGUs and the retired DO stack frames.
[[nodiscard]] Result execute(const Registers &initial,
                             std::span<const Word> data,
                             std::span<const Word> coefficients,
                             HostIsa host_isa = HostIsa::scalar);
} // namespace dsp56300::test::fir_host
