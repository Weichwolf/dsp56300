// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <cstddef>

namespace dsp56300::test::numeric_host {
struct Job {
  const Word *input, *coefficients;
  Word *state, *output, *progress;
  std::size_t first, frames, total;
};

// Complete authored banks, including input periods, in-place history and output rings.
void iir_asm(const Job &job, unsigned bits, bool rm);
void nonlinear_asm(const Job &job, unsigned bits, bool rm);
}
