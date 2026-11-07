// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "fir_host_kernel.hpp"
#include "support.hpp"

#include <cstddef>
#include <span>

namespace dsp56300::test::fir_host {
namespace {
template <bool TrackOverflow>
void vector_kernel(Kernel &state,
                   const std::span<const Word> data,
                   const std::span<const Word> coefficients) {
  constexpr std::size_t lanes = 2;
  const auto count = data.size() - (data.size() % lanes);
  asm volatile(R"(
    movi v6.2d,#0
    movi v5.2d,#0
    1:
      add %[left],%[data],%[index],lsl #2
      add %[right],%[coefficients],%[index],lsl #2
      ldr d0,[%[left]]
      ldr d1,[%[right]]
      shl v0.2s,v0.2s,#8
      shl v1.2s,v1.2s,#8
      sshr v0.2s,v0.2s,#8
      sshr v1.2s,v1.2s,#8
      smull v2.2d,v0.2s,v1.2s
      .if %c[track]
        shl v2.2d,v2.2d,#9
        ext v3.16b,v6.16b,v2.16b,#8
        add v2.2d,v2.2d,v3.2d
        dup v5.2d,%[sum]
        add v3.2d,v5.2d,v2.2d
        eor v0.16b,v3.16b,v5.16b
        eor v1.16b,v3.16b,v2.16b
        and v0.16b,v0.16b,v1.16b
        ushr v0.2d,v0.2d,#63
        umaxv s0,v0.4s
        umov %w[left],v0.s[0]
        orr %[sticky],%[sticky],%[left]
        umov %[sum],v3.d[1]
      .else
        add v5.2d,v5.2d,v2.2d
      .endif
      add %[index],%[index],#2
      cmp %[index],%[count]
      b.lo 1b
    .if !%c[track]
      addp d5,v5.2d
      fmov %[sum],d5
      lsl %[sum],%[sum],#9
    .endif
  )"
               : [sum] "+&r"(state.accumulated.sum),
                 [sticky] "+&r"(state.accumulated.overflow),
                 [index] "+&r"(state.index),
                 [left] "=&r"(state.left),
                 [right] "=&r"(state.right)
               : [data] "r"(data.data()),
                 [coefficients] "r"(coefficients.data()),
                 [count] "r"(count),
                 [track] "i"(TrackOverflow)
               : "cc", "memory", "v0", "v1", "v2", "v3", "v5", "v6");
}
}

void vector_products(Kernel &state,
                     const std::span<const Word> data,
                     const std::span<const Word> coefficients) {
  constexpr auto safe_words = std::size_t{1} << (accumulator_bits - (2 * word_bits));
  if (state.index == 0 && state.accumulated.sum == 0 && data.size() < safe_words) {
    vector_kernel<false>(state, data, coefficients);
  } else {
    vector_kernel<true>(state, data, coefficients);
  }
}
}
