// SPDX-License-Identifier: GPL-3.0-only
// Single-thread FFT reference for the paired JIT/host stream comparison.
#pragma once
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>
#include <vector>

namespace dsp56300::test::fft::host {
inline constexpr unsigned fractional_bits = 23, rounding_half_bits = 1, lane_count = 4;
inline constexpr std::int64_t scale = std::int64_t{1} << fractional_bits;

struct Stage {
  std::size_t half = 0;
  std::vector<std::int64_t> real, imaginary;
};

struct Data {
  std::vector<std::int64_t> real, imaginary, output_real, output_imaginary;
};

struct Job {
  Data *data = nullptr;
  const Stage *stage = nullptr;
  bool rm = false;
};

struct PlanInput {
  std::span<const Word> real, imaginary;
};

inline std::vector<Stage> make_plan(const PlanInput input) {
  const auto real = input.real;
  const auto imaginary = input.imaginary;
  const auto size = 2 * real.size();
  std::vector<Stage> result;
  for (auto half = real.size(); half != 0; half /= 2) {
    Stage stage;
    stage.half = half;
    stage.real.reserve(half);
    stage.imaginary.reserve(half);
    for (std::size_t index = 0; index < half; ++index) {
      const auto offset = index * size / (2 * half);
      stage.real.push_back(signed24(real[offset]));
      stage.imaginary.push_back(signed24(imaginary[offset]));
    }
    result.push_back(std::move(stage));
  }
  return result;
}

struct Round {
  std::int64_t value = 0;
  unsigned bits = 0;
  bool rm = false;
};

inline std::int64_t rounded(const Round input) noexcept {
  const auto floor = input.value >> input.bits;
  const auto residue =
      static_cast<std::uint64_t>(input.value) & ((std::uint64_t{1} << input.bits) - 1);
  const auto half = std::uint64_t{1} << (input.bits - 1);
  const bool increment = residue > half || (residue == half && (input.rm || (floor & 1) != 0));
  return floor + static_cast<std::int64_t>(increment);
}

inline std::int64_t limited(const std::int64_t value) noexcept {
  return std::clamp(value, -scale, scale - 1);
}

inline void scalar_butterfly(Job &job, const std::size_t flat) noexcept {
  const auto half = job.stage->half;
  const auto column = flat % half;
  const auto left = (2 * (flat / half) * half) + column;
  const auto right = left + half;
  auto &data = *job.data;
  const auto ar = data.real[left];
  const auto ai = data.imaginary[left];
  const auto br = data.real[right];
  const auto bi = data.imaginary[right];
  const auto dr = limited(rounded({.value = ar - br, .bits = rounding_half_bits, .rm = job.rm}));
  const auto di = limited(rounded({.value = ai - bi, .bits = rounding_half_bits, .rm = job.rm}));
  const auto wr = job.stage->real[column];
  const auto wi = job.stage->imaginary[column];
  data.real[left] = limited(rounded({.value = ar + br, .bits = rounding_half_bits, .rm = job.rm}));
  data.imaginary[left] =
      limited(rounded({.value = ai + bi, .bits = rounding_half_bits, .rm = job.rm}));
  data.real[right] =
      limited(rounded({.value = (dr * wr) - (di * wi), .bits = fractional_bits, .rm = job.rm}));
  data.imaginary[right] =
      limited(rounded({.value = (dr * wi) + (di * wr), .bits = fractional_bits, .rm = job.rm}));
}

inline void scalar_work(Job &job, const std::size_t begin, const std::size_t end) noexcept {
  for (auto index = begin; index < end; ++index) { scalar_butterfly(job, index); }
}

// Compiler vector expressions keep arithmetic independent of x86 intrinsics.
// The AVX2 variant is selected only after the host feature check.
using Vector = std::int64_t __attribute__((ext_vector_type(lane_count)));
using Narrow = std::int32_t __attribute__((ext_vector_type(lane_count)));

template <unsigned Bits, bool Rm>
[[gnu::target("avx2")]] inline Vector round_vector(const Vector value) noexcept {
  constexpr auto mask = (std::int64_t{1} << Bits) - 1;
  constexpr auto half = std::int64_t{1} << (Bits - 1);
  const Vector one{1, 1, 1, 1};
  const Vector halfway{half, half, half, half};
  const Vector masks{mask, mask, mask, mask};
  const auto floor = value >> Bits;
  const auto remainder = value & masks;
  const auto greater = remainder > halfway;
  const auto equal = remainder == halfway;
  const auto parity = Rm ? one : (floor & one);
  return floor + ((greater & one) | (equal & parity));
}

[[gnu::target("avx2")]] inline Vector limit_vector(const Vector value) noexcept {
  const Vector low{-scale, -scale, -scale, -scale};
  const Vector high{scale - 1, scale - 1, scale - 1, scale - 1};
  const auto top = value > high ? high : value;
  return top < low ? low : top;
}

[[gnu::target("avx2")]] inline Vector load(const std::int64_t *const data) noexcept {
  Vector result{};
  std::memcpy(&result, data, sizeof(result));
  return result;
}

[[gnu::target("avx2")]] inline void store(std::int64_t *const data, const Vector value) noexcept {
  std::memcpy(data, &value, sizeof(value));
}

struct Product {
  Vector left{}, right{};
};

[[gnu::target("avx2")]] inline Vector product(const Product input) noexcept {
  // Inputs are limited signed24 words. Explicit widening lets LLVM select
  // four signed32-to-signed64 products instead of general 64-bit multiplication.
  const auto a = __builtin_convertvector(__builtin_convertvector(input.left, Narrow), Vector);
  const auto b = __builtin_convertvector(__builtin_convertvector(input.right, Narrow), Vector);
  return a * b;
}

template <bool Rm>
[[gnu::target("avx2")]] inline void vector_butterfly(Job &job, const std::size_t flat) noexcept {
  const auto half = job.stage->half;
  const auto column = flat % half;
  const auto left = (2 * (flat / half) * half) + column;
  const auto right = left + half;
  auto &data = *job.data;
  const auto ar = load(data.real.data() + left);
  const auto ai = load(data.imaginary.data() + left);
  const auto br = load(data.real.data() + right);
  const auto bi = load(data.imaginary.data() + right);
  const auto dr = limit_vector(round_vector<rounding_half_bits, Rm>(ar - br));
  const auto di = limit_vector(round_vector<rounding_half_bits, Rm>(ai - bi));
  const auto wr = load(job.stage->real.data() + column);
  const auto wi = load(job.stage->imaginary.data() + column);
  const auto real = product({.left = dr, .right = wr}) - product({.left = di, .right = wi});
  const auto imaginary = product({.left = dr, .right = wi}) + product({.left = di, .right = wr});
  store(data.real.data() + left, limit_vector(round_vector<rounding_half_bits, Rm>(ar + br)));
  store(data.imaginary.data() + left, limit_vector(round_vector<rounding_half_bits, Rm>(ai + bi)));
  store(data.real.data() + right, limit_vector(round_vector<fractional_bits, Rm>(real)));
  store(data.imaginary.data() + right, limit_vector(round_vector<fractional_bits, Rm>(imaginary)));
}

template <bool Rm>
[[gnu::target("avx2")]] inline void
vector_work(Job &job, const std::size_t begin, const std::size_t end) noexcept {
  auto index = begin;
  while (index < end) {
    const auto column = index % job.stage->half;
    if (column + lane_count <= job.stage->half && index + lane_count <= end) {
      vector_butterfly<Rm>(job, index);
      index += lane_count;
    } else {
      scalar_butterfly(job, index);
      ++index;
    }
  }
}

[[gnu::target("avx2")]] inline void
avx2_work(Job &job, const std::size_t begin, const std::size_t end) noexcept {
  if (job.rm) {
    vector_work<true>(job, begin, end);
  } else {
    vector_work<false>(job, begin, end);
  }
}

using Work = void (*)(Job &, std::size_t, std::size_t) noexcept;

inline void
transform(Data &data, const std::span<const Stage> plan, const bool rm, const bool avx2) {
  const Work work = avx2 ? avx2_work : scalar_work;
  for (const auto &stage : plan) {
    Job job{.data = &data, .stage = &stage, .rm = rm};
    work(job, 0, data.real.size() / 2);
  }
  const auto size = data.real.size();
  for (std::size_t index = 0; index < size; ++index) {
    const auto reverse = __builtin_bitreverse32(static_cast<Word>(index)) >>
                         (32 - static_cast<unsigned>(std::countr_zero(size)));
    data.output_real[index] = data.real[reverse];
    data.output_imaginary[index] = data.imaginary[reverse];
  }
}
} // namespace dsp56300::test::fft::host
