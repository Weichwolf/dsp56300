// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fir_host.hpp"
#include "fir_stream_program.hpp"
#include "fixture_memory.hpp"
#include "stream_host.hpp"
#include "support.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using Clock = std::chrono::steady_clock;
using namespace fir_stream;
constexpr Word changed_flags = flag_v | flag_z | flag_n | flag_e | flag_u;

class HostStream {
public:
  explicit HostStream(const BinaryCase &item)
      : memory_(initial_memory(item)), state_(item.initial), initial_(item.initial) {
    const auto &code = code_;
    require(item.before.front().words == code.words && initial_.sp == 0 && initial_.sc == 0 &&
                initial_.n[7] != 0 && initial_.n[7] <= stream_frames,
            "Host stream requires the authored bounded FIR bank and original outer frame");
    state_.stack[1] = (std::uint64_t{initial_.la} << word_bits) | initial_.lc;
    state_.stack[2] = (std::uint64_t{code.frame_body} << word_bits) | initial_.sr;
    state_.sr |= loop_flag;
    state_.sp = state_.sc = 2;
    state_.la = code.frame_end;
    state_.lc = initial_.n[7];
    state_.pc = code.frame_body;
  }

  void run(const Word count, const fir_host::HostIsa isa) {
    require(count <= state_.lc, "Host stream cannot execute beyond its retained outer loop");
    const auto &code = code_;
    for (Word frame = 0; frame < count; ++frame) {
      state_.stack[3] = (std::uint64_t{code.frame_end} << word_bits) | state_.lc;
      state_.stack[4] = (std::uint64_t{code.channel_body} << word_bits) | state_.sr;
      for (Word channel = 0; channel < channels; ++channel) {
        state_.stack[5] = (std::uint64_t{code.channel_end} << word_bits) | (channels - channel);
        state_.stack[6] = (std::uint64_t{code.mac_pc} << word_bits) | (state_.sr & ~changed_flags) |
                          flag_z | flag_u;
        const auto cursor = memory_[1][cursor_base + channel];
        const auto data = std::span<const Word>(memory_[1]).subspan(cursor, taps);
        const auto weights = std::span<const Word>(memory_[2]).subspan(coefficient_base, taps);
        const auto output = fir_host::filter(state_, data, weights, isa);
        memory_[1][state_.r[1]] = output;
        state_.r[1] = output_base + ((state_.r[1] - output_base + 1) % output_words);
        state_.r[0] = cursor + 1;
        memory_[1][cursor_base + channel] = state_.r[0];
        state_.x = (state_.x & ~std::uint64_t{word_mask}) | data.back();
        state_.y = (state_.y & ~std::uint64_t{word_mask}) | weights.back();
      }
      state_.r[2] = state_.r[3] = cursor_base + channels;
      state_.r[4] = coefficient_base;
      state_.x = (state_.x & word_mask) | (std::uint64_t{state_.lc} << word_bits);
      memory_[1][progress_address] = state_.lc;
      --state_.lc;
    }
    if (state_.lc == 0) {
      state_.sr = (state_.sr & ~loop_flag) | (initial_.sr & loop_flag);
      state_.la = initial_.la;
      state_.lc = initial_.lc;
      state_.sp = initial_.sp;
      state_.sc = initial_.sc;
      state_.pc = code.frame_end + 1;
    }
  }

  void verify(const BinaryCase &item) const {
    auto expected = initial_memory(item);
    patch(expected, item.after);
    require(memory_ == expected && state_ == item.expected,
            "Native host FIR bank matches independent complete registers and every P/X/Y word");
  }

private:
  FixtureMemory memory_;
  Registers state_, initial_;
  Program code_ = program();
};

double measure(const BinaryCase &item, const fir_host::HostIsa isa) {
  HostStream host(item);
  host.run(sample_rate, isa);
  const auto begin = Clock::now();
  host.run(sample_rate, isa);
  const auto elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
  host.verify(item);
  return elapsed;
}
}

HostStreamTimes measure_fir_host(const BinaryCase &item) {
  require(item.initial.n[7] == stream_frames, "Matched stream timing covers two input seconds");
  HostStreamTimes result{.scalar_seconds = measure(item, fir_host::HostIsa::scalar),
                         .cpp_seconds = measure(item, fir_host::HostIsa::cpp)};
  if (fir_host::vector_supported()) {
    result.vector_seconds = measure(item, fir_host::HostIsa::vector);
  }
  return result;
}

void verify_fir_host(const std::vector<BinaryCase> &cases) {
  for (const auto &item : cases) {
    for (const auto isa :
         {fir_host::HostIsa::scalar, fir_host::HostIsa::vector, fir_host::HostIsa::cpp}) {
      if (isa == fir_host::HostIsa::vector && !fir_host::vector_supported()) { continue; }
      HostStream host(item);
      host.run(item.initial.n[7], isa);
      host.verify(item);
    }
  }
  std::cout << cases.size() << " independent complete host FIR stream cases passed\n";
}
}
