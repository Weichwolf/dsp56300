// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "fir_host_kernel.hpp"
#include "support.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::test::fir_host {
namespace {
template <bool TrackOverflow>
// Preserve an ABI call boundary around VZEROUPPER and its vector-register clobbers.
[[gnu::target("avx2"), gnu::noinline]] void vector_kernel(
    Kernel &state, const std::span<const Word> data, const std::span<const Word> coefficients) {
  constexpr std::size_t lanes = 4;
  const auto count = data.size() - (data.size() % lanes);
  if (state.index >= count) { return; }
  constexpr auto sign = std::uint64_t{1} << 63;
  constexpr unsigned low_pair_sum = 0x55; // Replicate prefix lane 1 into all four lanes.
  constexpr unsigned upper_pair = 0xf0;   // Take the upper 128 bits from that replicated sum.
  constexpr unsigned final_sum = 0xff;    // Replicate prefix lane 3 for the next group.
  // Four exact prefix sums expose intermediate overflows even if cancellation
  // brings the group's last value back into range. The first out-of-range
  // unwrapped prefix proves a real sequential overflow; sticky L needs any one.
  asm volatile(".if %c[track]\n\t"
               "vmovq %[sum], %%xmm5\n\t"
               "vpbroadcastq %%xmm5, %%ymm5\n\t"
               "vpxor %%ymm6, %%ymm6, %%ymm6\n\t"
               "movq %[sign], %[temporary]\n\t"
               "vmovq %[temporary], %%xmm7\n\t"
               "vpbroadcastq %%xmm7, %%ymm7\n\t"
               ".else\n\t"
               "vpxor %%ymm5, %%ymm5, %%ymm5\n\t"
               ".endif\n\t"
               "1:\n\t"
               "vpmovsxdq (%[data],%[index],4), %%ymm0\n\t"
               "vpmovsxdq (%[coefficients],%[index],4), %%ymm1\n\t"
               "vpslld %[extension], %%ymm0, %%ymm0\n\t"
               "vpslld %[extension], %%ymm1, %%ymm1\n\t"
               "vpsrad %[extension], %%ymm0, %%ymm0\n\t"
               "vpsrad %[extension], %%ymm1, %%ymm1\n\t"
               "vpmuldq %%ymm1, %%ymm0, %%ymm2\n\t"
               ".if %c[track]\n\t"
               "vpsllq %[product_shift], %%ymm2, %%ymm2\n\t"
               "vpslldq %[lane_bytes], %%ymm2, %%ymm3\n\t"
               "vpaddq %%ymm2, %%ymm3, %%ymm3\n\t"
               "vpermq %[low_pair_sum], %%ymm3, %%ymm4\n\t"
               "vblendps %[upper_pair], %%ymm4, %%ymm6, %%ymm4\n\t"
               "vpaddq %%ymm3, %%ymm4, %%ymm2\n\t"
               "vpaddq %%ymm5, %%ymm2, %%ymm3\n\t"
               "vpxor %%ymm3, %%ymm5, %%ymm0\n\t"
               "vpxor %%ymm3, %%ymm2, %%ymm1\n\t"
               "vpand %%ymm1, %%ymm0, %%ymm0\n\t"
               "vptest %%ymm7, %%ymm0\n\t"
               "setne %[overflow]\n\t"
               "movzbl %[overflow], %k[temporary]\n\t"
               "orq %[temporary], %[sticky]\n\t"
               "vpermq %[final_sum], %%ymm3, %%ymm5\n\t"
               ".else\n\t"
               "vpaddq %%ymm2, %%ymm5, %%ymm5\n\t"
               ".endif\n\t"
               "addq %[lanes], %[index]\n\t"
               "cmpq %[count], %[index]\n\t"
               "jb 1b\n\t"
               ".if %c[track]\n\t"
               "vmovq %%xmm5, %[sum]\n\t"
               ".else\n\t"
               "vextracti128 $1, %%ymm5, %%xmm0\n\t"
               "vpaddq %%xmm5, %%xmm0, %%xmm0\n\t"
               "vpsrldq %[lane_bytes], %%xmm0, %%xmm1\n\t"
               "vpaddq %%xmm1, %%xmm0, %%xmm0\n\t"
               "vmovq %%xmm0, %[sum]\n\t"
               "shlq %[product_shift], %[sum]\n\t"
               ".endif\n\t"
               "vzeroupper"
               : [sum] "+&r"(state.accumulated.sum),
                 [sticky] "+&r"(state.accumulated.overflow),
                 [index] "+&r"(state.index),
                 [temporary] "=&r"(state.left),
                 [overflow] "=&q"(state.overflow)
               : [data] "r"(data.data()),
                 [coefficients] "r"(coefficients.data()),
                 [count] "r"(count),
                 [sign] "i"(sign),
                 [track] "i"(TrackOverflow),
                 [lanes] "i"(lanes),
                 [lane_bytes] "i"(sizeof(std::uint64_t)),
                 [low_pair_sum] "i"(low_pair_sum),
                 [upper_pair] "i"(upper_pair),
                 [final_sum] "i"(final_sum),
                 [extension] "i"(32 - word_bits),
                 [product_shift] "i"(64 - accumulator_bits + 1)
               : "cc",
                 "memory",
                 "ymm0",
                 "ymm1",
                 "ymm2",
                 "ymm3",
                 "ymm4",
                 "ymm5",
                 "ymm6",
                 "ymm7",
                 "ymm8",
                 "ymm9",
                 "ymm10",
                 "ymm11",
                 "ymm12",
                 "ymm13",
                 "ymm14",
                 "ymm15");
}
} // namespace

void vector_products(Kernel &state,
                     const std::span<const Word> data,
                     const std::span<const Word> coefficients) {
  // At most 255 arbitrary signed-24 fractional products fit signed-56 from CLR.
  // A fresh short reduction therefore cannot create intermediate overflow.
  constexpr auto safe_words = std::size_t{1} << (accumulator_bits - (2 * word_bits));
  if (state.index == 0 && state.accumulated.sum == 0 && data.size() < safe_words) {
    vector_kernel<false>(state, data, coefficients);
  } else {
    vector_kernel<true>(state, data, coefficients);
  }
}
} // namespace dsp56300::test::fir_host
