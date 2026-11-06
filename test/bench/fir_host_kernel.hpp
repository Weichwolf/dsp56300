// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::test::fir_host {
struct Accumulator {
  std::int64_t sum = 0;
  std::uint64_t overflow = 0;
};

struct Kernel {
  Accumulator accumulated;
  std::size_t index = 0;
  std::int64_t left = 0, right = 0;
  std::uint8_t overflow = 0;
};

void vector_products(Kernel &state, std::span<const Word> data, std::span<const Word> coefficients);
} // namespace dsp56300::test::fir_host
