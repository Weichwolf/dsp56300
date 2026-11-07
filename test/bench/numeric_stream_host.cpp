// SPDX-License-Identifier: GPL-3.0-only
// Independent native recurrences: exact word results without an ISA interpreter.
#include "binary_fixture.hpp"
#include "dsp56300/memory.hpp"
#include "fixture_memory.hpp"
#include "iir_program.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "numeric_host_asm.hpp"
#include "stream_host.hpp"
#include "stream_host_math.hpp"
#include "support.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
constexpr std::int64_t state_unit = std::int64_t{1} << word_bits;

class NativeBank {
public:
  NativeBank(const std::string_view name, const BinaryCase &item, const bool use_asm = false)
      : memory_(initial_memory(item)),
        total_(item.initial.n[7]),
        nonlinear_(name == "nonlinear"),
        use_asm_(use_asm),
        bits_(word_bits + ((item.initial.sr & mode_down) != 0 ? 1U : 0U) -
              ((item.initial.sr & mode_up) != 0 ? 1U : 0U)),
        rm_((item.initial.sr & mode_rm) != 0) {
    require(name == "iir" || nonlinear_, "Native bank name");
    const auto code = nonlinear_ ? nonlinear_stream::program().words : iir_stream::program().words;
    require(item.before.front().words == code && total_ != 0 && total_ <= iir_stream::stream_frames,
            "Native bank requires the authored program and bounded frame count");
  }

  void run(const Word count) {
    require(count <= total_ - frame_, "Native bank cannot exceed the requested stream");
    if (use_asm_) {
      run_asm(count);
      frame_ += count;
      return;
    }
    for (Word index = 0; index < count; ++index, ++frame_) {
      if (nonlinear_) {
        nonlinear_frame();
      } else {
        iir_frame();
      }
    }
  }

  void verify(const BinaryCase &item) const {
    auto expected = initial_memory(item);
    patch(expected, item.after);
    require(frame_ == total_ && memory_ == expected,
            "Native recurrence matches independent history, outputs and all P/X/Y memory");
  }

private:
  void run_asm(const Word count) {
    const numeric_host::Job job{
        .input = memory_[1].data() +
                 (nonlinear_ ? nonlinear_stream::input_base : iir_stream::input_base),
        .coefficients = memory_[2].data() + (nonlinear_ ? nonlinear_stream::coefficient_base
                                                        : iir_stream::coefficient_base),
        .state = memory_[1].data() +
                 (nonlinear_ ? nonlinear_stream::state_base : iir_stream::state_base),
        .output = memory_[1].data() +
                  (nonlinear_ ? nonlinear_stream::output_base : iir_stream::output_base),
        .progress = nonlinear_ ? &memory_[1][nonlinear_stream::progress_address] : nullptr,
        .first = frame_,
        .frames = count,
        .total = total_};
    if (nonlinear_) {
      numeric_host::nonlinear_asm(job, bits_, rm_);
    } else {
      numeric_host::iir_asm(job, bits_, rm_);
    }
  }

  [[nodiscard]] Word quantize(const std::int64_t raw) const noexcept {
    return host_math::word(host_math::rounded(raw, bits_, rm_));
  }

  void iir_frame() {
    using namespace iir_stream;
    std::int64_t sum = 0;
    for (Word channel = 0; channel < channels; ++channel) {
      auto value =
          signed24(memory_[1][input_base + ((frame_ % signal_period) * channels) + channel]);
      for (Word stage = 0; stage < sections; ++stage) {
        const auto slot = state_base + (2 * ((channel * sections) + stage));
        const auto weight = coefficient_base + (stage * iir::coefficients_per_section);
        const auto *const coefficients = memory_[2].data() + weight;
        const auto output = quantize(host_math::mac(signed24(memory_[1][slot]) * state_unit,
                                                    {.a = value, .b = signed24(coefficients[0])}));
        auto first = host_math::mac(signed24(memory_[1][slot + 1]) * state_unit,
                                    {.a = value, .b = signed24(coefficients[1])});
        first = host_math::mac(first, {.a = -signed24(output), .b = signed24(coefficients[2])});
        auto second = host_math::mac(0, {.a = value, .b = signed24(coefficients[3])});
        second = host_math::mac(second, {.a = -signed24(output), .b = signed24(coefficients[4])});
        memory_[1][slot] = quantize(first);
        memory_[1][slot + 1] = quantize(second);
        value = signed24(output);
      }
      sum += value;
    }
    memory_[1][output_base + (frame_ % output_words)] = quantize((sum * state_unit) >> 7);
  }

  void nonlinear_frame() {
    using namespace nonlinear_stream;
    constexpr auto boundary = (std::int64_t{1} << (word_bits - 1)) * state_unit;
    const auto quantum = std::int64_t{1} << bits_;
    for (Word channel = 0; channel < channels; ++channel) {
      auto value = memory_[1][input_base + ((frame_ % signal_period) * channels) + channel];
      for (Word stage = 0; stage < stages; ++stage) {
        const auto slot = state_base + (channel * stages) + stage;
        const auto gain = signed24(memory_[2][coefficient_base + (2 * stage)]);
        const auto feedback = signed24(memory_[2][coefficient_base + (2 * stage) + 1]);
        auto raw = host_math::mac(0, {.a = signed24(value), .b = gain});
        raw = std::clamp(raw, -boundary, boundary - 1);
        raw = host_math::mac(raw, {.a = signed24(memory_[1][slot]), .b = feedback});
        raw = host_math::rounded(raw, bits_, rm_) * quantum;
        raw = raw >= boundary ? boundary - state_unit : std::max(raw, -boundary);
        value = host_math::word(raw >> bits_);
        memory_[1][slot] = value;
      }
      memory_[1][output_base + (((frame_ * channels) + channel) % output_words)] = value;
    }
    memory_[1][progress_address] = total_ - frame_;
  }

  FixtureMemory memory_;
  Word total_, frame_ = 0;
  bool nonlinear_, use_asm_;
  unsigned bits_;
  bool rm_;
};

double numeric_time(const std::string_view name, const BinaryCase &item, const bool use_asm) {
  NativeBank host(name, item, use_asm);
  host.run(iir_stream::sample_rate);
  const auto begin = Clock::now();
  host.run(iir_stream::sample_rate);
  const auto elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
  host.verify(item);
  return elapsed;
}

}

HostStreamTimes measure_numeric_host(const std::string_view name, const BinaryCase &item) {
  require(item.initial.n[7] == iir_stream::stream_frames, "Native timing covers two input seconds");
  return {.scalar_seconds = numeric_time(name, item, false),
          .asm_seconds = numeric_time(name, item, true),
          .scope = "fixed-point-algorithm-and-memory",
          .implementation = "handwritten-scalar-asm-and-cpp"};
}

void verify_numeric_host(const std::string_view name, const std::vector<BinaryCase> &cases) {
  for (const auto &item : cases) {
    NativeBank host(name, item);
    host.run(item.initial.n[7]);
    host.verify(item);
    NativeBank native(name, item, true);
    native.run(item.initial.n[7]);
    native.verify(item);
  }
  std::cout << cases.size() << " independent native " << name << " stream cases passed\n";
}
} // namespace dsp56300::test
