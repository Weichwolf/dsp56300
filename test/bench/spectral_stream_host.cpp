// SPDX-License-Identifier: GPL-3.0-only
// Optimized numerical window/FFT/power reference, never an emulator backend.
#include "binary_fixture.hpp"
#include "dsp56300/memory.hpp"
#include "fft_native.hpp"
#include "fft_program.hpp"
#include "fixture_memory.hpp"
#include "spectral_stream_program.hpp"
#include "stream_host.hpp"
#include "stream_host_math.hpp"
#include "support.hpp"

#include <chrono>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
using namespace spectral_stream;

class NativeSpectrum {
public:
  NativeSpectrum(const BinaryCase &item, const bool avx2)
      : memory_(initial_memory(item)),
        total_(item.initial.n[7]),
        rm_((item.initial.sr & mode_rm) != 0),
        plan_(fft::native::make_plan(
            {.real = std::span<const Word>(memory_[1]).subspan(fft::twiddle_base, size / 2),
             .imaginary = std::span<const Word>(memory_[2]).subspan(fft::twiddle_base, size / 2)})),
        pool_(1, avx2) {
    require(item.before.front().words == program().code.words && total_ > 0 &&
                total_ <= stream_hops && (item.initial.sr & (mode_up | mode_down)) == 0,
            "Native spectrum requires the authored bounded ordinary-scaling stream");
    data_.real.resize(size);
    data_.imaginary.resize(size);
    data_.output_real.resize(size);
    data_.output_imaginary.resize(size);
  }

  void run(const Word count) {
    require(count <= total_ - step_, "Native spectrum cannot exceed the requested stream");
    for (Word index = 0; index < count; ++index, ++step_) {
      for (Word channel = 0; channel < channels; ++channel) { spectrum(channel); }
      memory_[1][progress_address] = total_ - step_;
    }
  }

  void verify(const BinaryCase &item) const {
    auto expected = initial_memory(item);
    patch(expected, item.after);
    require(step_ == total_ && memory_ == expected,
            "Native spectrum matches independent FFT, power, cursors and all P/X/Y memory");
  }

private:
  void spectrum(const Word channel) {
    const auto cursor = memory_[1][cursor_base + channel];
    for (Word index = 0; index < size; ++index) {
      const auto weight = signed24(memory_[1][window_base + index]);
      data_.real[index] = host_math::rounded(
          signed24(memory_[1][cursor + index]) * weight, fft::native::fractional_bits, rm_);
      data_.imaginary[index] = host_math::rounded(
          signed24(memory_[2][cursor + index]) * weight, fft::native::fractional_bits, rm_);
    }
    pool_.run(data_, plan_, rm_);
    auto power_cursor = memory_[1][power_cursor_address];
    for (Word index = 0; index < size; ++index) {
      memory_[1][fft::data_base + index] = static_cast<Word>(data_.real[index]) & word_mask;
      memory_[2][fft::data_base + index] = static_cast<Word>(data_.imaginary[index]) & word_mask;
      const auto real = data_.output_real[index];
      const auto imaginary = data_.output_imaginary[index];
      memory_[1][fft::output_base + index] = static_cast<Word>(real) & word_mask;
      memory_[2][fft::output_base + index] = static_cast<Word>(imaginary) & word_mask;
      memory_[1][power_cursor] = host_math::word(
          host_math::rounded((real * real) + (imaginary * imaginary), word_bits, rm_));
      power_cursor = power_base + ((power_cursor - power_base + 1) % power_words);
    }
    memory_[1][cursor_base + channel] = cursor + hop;
    memory_[1][power_cursor_address] = power_cursor;
  }

  FixtureMemory memory_;
  Word total_, step_ = 0;
  bool rm_;
  std::vector<fft::native::Stage> plan_;
  fft::native::Pool pool_;
  fft::native::Data data_;
};

double measure(const BinaryCase &item, const bool avx2) {
  NativeSpectrum host(item, avx2);
  host.run(hops_per_second);
  const auto begin = Clock::now();
  host.run(hops_per_second);
  const auto elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
  host.verify(item);
  return elapsed;
}
}

HostStreamTimes measure_spectral_host(const BinaryCase &item) {
  require(item.initial.n[7] == stream_hops, "Native spectrum timing covers two input seconds");
  HostStreamTimes result{.scalar_seconds = measure(item, false),
                         .scope = "fixed-point-algorithm-and-memory"};
  if (__builtin_cpu_supports("avx2")) { result.avx2_seconds = measure(item, true); }
  return result;
}

void verify_spectral_host(const std::vector<BinaryCase> &cases) {
  for (const bool avx2 : {false, true}) {
    if (avx2 && !__builtin_cpu_supports("avx2")) { continue; }
    for (const auto &item : cases) {
      NativeSpectrum host(item, avx2);
      host.run(item.initial.n[7]);
      host.verify(item);
    }
  }
  std::cout << cases.size() << " independent native spectral stream cases passed per ISA\n";
}
} // namespace dsp56300::test
