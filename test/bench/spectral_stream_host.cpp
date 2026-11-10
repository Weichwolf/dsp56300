// SPDX-License-Identifier: GPL-3.0-only
// Optimized numerical window/FFT/power reference, never an emulator backend.
#include "binary_fixture.hpp"
#include "dsp56300/memory.hpp"
#include "fft_host.hpp"
#include "fft_program.hpp"
#include "fixture_memory.hpp"
#include "host_features.hpp"
#include "spectral_host_asm.hpp"
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

enum class Kernel : unsigned char { scalar, vector, assembly };

template <Kernel Version> class NativeSpectrum {
public:
  explicit NativeSpectrum(const BinaryCase &item)
      : memory_(initial_memory(item)),
        total_(item.initial.n[7]),
        rm_((item.initial.sr & mode_rm) != 0),
        plan_(fft::host::make_plan(
            {.real = std::span<const Word>(memory_[1]).subspan(fft::twiddle_base, size / 2),
             .imaginary =
                 std::span<const Word>(memory_[2]).subspan(fft::twiddle_base, size / 2)})) {
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
  void window(const Word cursor) {
    if constexpr (Version == Kernel::assembly) {
      fft::host::window_asm({.real = memory_[1].data() + cursor,
                             .imaginary = memory_[2].data() + cursor,
                             .weights = memory_[1].data() + window_base,
                             .output_real = data_.real.data(),
                             .output_imaginary = data_.imaginary.data(),
                             .count = size},
                            rm_);
      return;
    }
    for (Word index = 0; index < size; ++index) {
      const auto weight = signed24(memory_[1][window_base + index]);
      data_.real[index] = host_math::rounded(
          signed24(memory_[1][cursor + index]) * weight, fft::host::fractional_bits, rm_);
      data_.imaginary[index] = host_math::rounded(
          signed24(memory_[2][cursor + index]) * weight, fft::host::fractional_bits, rm_);
    }
  }

  void output() {
    auto power_cursor = memory_[1][power_cursor_address];
    if constexpr (Version == Kernel::assembly) {
      require(power_cursor >= power_base && power_cursor - power_base < power_words,
              "Native spectrum power ring cursor");
      const auto position =
          fft::host::power_asm({.real = data_.real.data(),
                                .imaginary = data_.imaginary.data(),
                                .output_real = data_.output_real.data(),
                                .output_imaginary = data_.output_imaginary.data(),
                                .stored_real = memory_[1].data() + fft::data_base,
                                .stored_imaginary = memory_[2].data() + fft::data_base,
                                .stored_output_real = memory_[1].data() + fft::output_base,
                                .stored_output_imaginary = memory_[2].data() + fft::output_base,
                                .power = memory_[1].data() + power_base,
                                .count = size,
                                .position = power_cursor - power_base,
                                .ring_words = power_words},
                               rm_);
      memory_[1][power_cursor_address] = power_base + static_cast<Word>(position);
      return;
    }
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
    memory_[1][power_cursor_address] = power_cursor;
  }

  void spectrum(const Word channel) {
    const auto cursor = memory_[1][cursor_base + channel];
    window(cursor);
    if constexpr (Version == Kernel::assembly) {
      fft::host::transform_asm(data_, plan_, rm_);
    } else {
      fft::host::transform(data_, plan_, rm_, Version == Kernel::vector);
    }
    output();
    memory_[1][cursor_base + channel] = cursor + hop;
  }

  FixtureMemory memory_;
  Word total_, step_ = 0;
  bool rm_;
  std::vector<fft::host::Stage> plan_;
  fft::host::Data data_;
};

template <Kernel Version> double measure(const BinaryCase &item) {
  NativeSpectrum<Version> host(item);
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
  HostStreamTimes result{.scalar_seconds = measure<Kernel::scalar>(item),
                         .scope = "fixed-point-algorithm-and-memory",
                         .implementation = "compiler-and-handwritten-spectrum-asm"};
  result.cpp_seconds = result.scalar_seconds;
  if (host_vector_available()) {
    result.vector_seconds = measure<Kernel::vector>(item);
    result.asm_seconds = measure<Kernel::assembly>(item);
  }
  return result;
}

void verify_spectral_host(const std::vector<BinaryCase> &cases) {
  const auto verify = [&cases]<Kernel Version>() {
    for (const auto &item : cases) {
      NativeSpectrum<Version> host(item);
      host.run(item.initial.n[7]);
      host.verify(item);
    }
  };
  verify.operator()<Kernel::scalar>();
  if (host_vector_available()) {
    verify.operator()<Kernel::vector>();
    verify.operator()<Kernel::assembly>();
  }
  std::cout << cases.size()
            << " independent native spectral stream cases passed per implementation\n";
}
} // namespace dsp56300::test
