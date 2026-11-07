// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/memory.hpp"
#include "fft_host.hpp"
#include "fft_program.hpp"
#include "fixture_memory.hpp"
#include "host_features.hpp"
#include "spectral_host_asm.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
std::int64_t signed_word(Word value) {
  value &= word_mask;
  return value < sign24 ? value : std::int64_t{value} - (std::int64_t{1} << word_bits);
}

struct Quantization {
  std::int64_t value;
  unsigned bits;
  bool rm;
};

std::int64_t quantized(const Quantization input) {
  const auto value = input.value;
  const auto bits = input.bits;
  const auto rm = input.rm;
  const auto unit = std::int64_t{1} << bits;
  auto quotient = value / unit;
  auto residue = value % unit;
  if (residue < 0) {
    --quotient;
    residue += unit;
  }
  if (residue > unit / 2 || (residue == unit / 2 && (rm || quotient % 2 != 0))) { ++quotient; }
  return std::clamp(quotient, -std::int64_t{sign24}, std::int64_t{sign24 - 1});
}

constexpr std::array<Word, 16> points{0,
                                      1,
                                      3,
                                      5,
                                      sign24,
                                      sign24 - 1,
                                      word_mask,
                                      0x400000,
                                      0xc00000,
                                      0x200000,
                                      0xe00000,
                                      2048,
                                      6144,
                                      0xfff800,
                                      0x123456,
                                      0xabfedcba};

void window_check(bool rm) {
  const std::size_t size = points.size() * points.size() * points.size();
  std::vector<Word> x(size);
  std::vector<Word> y(size);
  std::vector<Word> weights(size);
  constexpr std::int64_t guard = 0x13579b;
  std::vector<std::int64_t> a(size + 2, guard);
  std::vector<std::int64_t> b(size + 2, guard);
  for (std::size_t i = 0; i < size; ++i) {
    x[i] = points[i % points.size()];
    y[i] = points[(i / points.size()) % points.size()];
    weights[i] = points[i / (points.size() * points.size())];
  }
  fft::host::window_asm({.real = x.data(),
                         .imaginary = y.data(),
                         .weights = weights.data(),
                         .output_real = a.data() + 1,
                         .output_imaginary = b.data() + 1,
                         .count = size},
                        rm);
  for (std::size_t i = 0; i < size; ++i) {
    require(a[i + 1] == quantized({.value = signed_word(x[i]) * signed_word(weights[i]),
                                   .bits = word_bits - 1,
                                   .rm = rm}),
            "Window real rounding and limiting");
    require(b[i + 1] == quantized({.value = signed_word(y[i]) * signed_word(weights[i]),
                                   .bits = word_bits - 1,
                                   .rm = rm}),
            "Window imaginary rounding and limiting");
  }
  require(a.front() == guard && a.back() == guard && b.front() == guard && b.back() == guard,
          "Window guards");
}

void power_check(bool rm) {
  const std::size_t size = points.size() * points.size();
  std::vector<std::int64_t> a(size);
  std::vector<std::int64_t> b(size);
  std::vector<std::int64_t> c(size);
  std::vector<std::int64_t> d(size);
  constexpr Word guard = 0xabcdef12;
  constexpr std::size_t ring_size = 7;
  constexpr std::size_t first = 6;
  std::vector<Word> aa(size + 2, guard);
  std::vector<Word> bb(size + 2, guard);
  std::vector<Word> cc(size + 2, guard);
  std::vector<Word> dd(size + 2, guard);
  std::vector<Word> ring(ring_size + 2, guard);
  auto expected_ring = ring;
  for (std::size_t i = 0; i < size; ++i) {
    a[i] = signed_word(points[i % points.size()]);
    b[i] = signed_word(points[(i + 3) % points.size()]);
    c[i] = signed_word(points[i / points.size()]);
    d[i] = signed_word(points[i % points.size()]);
  }
  auto position = first;
  for (std::size_t i = 0; i < size; ++i) {
    expected_ring[position + 1] =
        static_cast<Word>(
            quantized({.value = (c[i] * c[i]) + (d[i] * d[i]), .bits = word_bits, .rm = rm})) &
        word_mask;
    position = (position + 1) % ring_size;
  }
  const auto cursor = fft::host::power_asm({.real = a.data(),
                                            .imaginary = b.data(),
                                            .output_real = c.data(),
                                            .output_imaginary = d.data(),
                                            .stored_real = aa.data() + 1,
                                            .stored_imaginary = bb.data() + 1,
                                            .stored_output_real = cc.data() + 1,
                                            .stored_output_imaginary = dd.data() + 1,
                                            .power = ring.data() + 1,
                                            .count = size,
                                            .position = first,
                                            .ring_words = ring_size},
                                           rm);
  require(cursor == position && ring == expected_ring, "Power rounding, saturation and ring order");
  for (std::size_t i = 0; i < size; ++i) {
    require(aa[i + 1] == (static_cast<Word>(a[i]) & word_mask) &&
                bb[i + 1] == (static_cast<Word>(b[i]) & word_mask) &&
                cc[i + 1] == (static_cast<Word>(c[i]) & word_mask) &&
                dd[i + 1] == (static_cast<Word>(d[i]) & word_mask),
            "Power memory planes");
  }
  for (const auto *plane : {&aa, &bb, &cc, &dd}) {
    require(plane->front() == guard && plane->back() == guard, "Power output guards");
  }
}

void run(const char *const path) {
  auto items = read_fixture(path);
  for (const auto &item : items) {
    const auto block = std::ranges::find_if(item.before, [](const auto &b) {
      return b.space == Space::x && b.address == fft::data_base;
    });
    require(block != item.before.end(), "FFT inputs");
    const auto count = block->words.size();
    auto memory = initial_memory(item);
    auto expected = memory;
    patch(expected, item.after);
    fft::host::Data data;
    data.real.resize(count);
    data.imaginary.resize(count);
    data.output_real.resize(count);
    data.output_imaginary.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      data.real[i] = signed24(memory[1][fft::data_base + i]);
      data.imaginary[i] = signed24(memory[2][fft::data_base + i]);
    }
    const auto plan = fft::host::make_plan(
        {.real = std::span<const Word>(memory[1]).subspan(fft::twiddle_base, count / 2),
         .imaginary = std::span<const Word>(memory[2]).subspan(fft::twiddle_base, count / 2)});
    fft::host::transform_asm(data, plan, (item.initial.sr & mode_rm) != 0);
    for (std::size_t i = 0; i < count; ++i) {
      require(data.real[i] == signed24(expected[1][fft::data_base + i]),
              item.name + " working real");
      require(data.imaginary[i] == signed24(expected[2][fft::data_base + i]),
              item.name + " working imaginary");
      require(data.output_real[i] == signed24(expected[1][fft::output_base + i]),
              item.name + " output real");
      require(data.output_imaginary[i] == signed24(expected[2][fft::output_base + i]),
              item.name + " output imaginary");
    }
  }
  for (const bool rm : {false, true}) {
    window_check(rm);
    power_check(rm);
  }
  std::cout << items.size() << " independent ASM FFT cases and " << checks
            << " numerical/guard checks passed\n";
}
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_spectral_host_test FFT.d56"); }
    dsp56300::test::require(dsp56300::test::host_vector_available(),
                            "ASM spectral qualification needs the native host SIMD ISA");
    dsp56300::test::run(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
