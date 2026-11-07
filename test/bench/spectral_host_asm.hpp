// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "fft_host.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::test::fft::host {
// Window count is a multiple of four; all input words use signed24 values.
struct Window {
  const Word *real, *imaginary, *weights;
  std::int64_t *output_real, *output_imaginary;
  std::size_t count;
};

// Every qword input is a canonical signed24 value; output arrays hold Word data.
struct Power {
  const std::int64_t *real, *imaginary, *output_real, *output_imaginary;
  Word *stored_real, *stored_imaginary, *stored_output_real, *stored_output_imaginary, *power;
  std::size_t count, position, ring_words;
};

void window_asm(const Window &job, bool rm);
std::size_t power_asm(const Power &job, bool rm);
void transform_asm(Data &data, std::span<const Stage> plan, bool rm);
}
