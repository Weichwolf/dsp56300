// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>

#if defined(__aarch64__)
#define DSP56300_HOST_VECTOR [[gnu::target("neon")]]
#else
#define DSP56300_HOST_VECTOR [[gnu::target("avx2")]]
#endif

namespace dsp56300::test {
#if defined(__aarch64__)
inline constexpr auto host_vector_name = "neon";
inline constexpr std::size_t host_vector_words = 2;

inline bool host_vector_available() noexcept {
  return true;
}
#else
inline constexpr auto host_vector_name = "avx2";
inline constexpr std::size_t host_vector_words = 4;

inline bool host_vector_available() noexcept {
  return __builtin_cpu_supports("avx2");
}
#endif
}
