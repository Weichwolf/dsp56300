// SPDX-License-Identifier: GPL-3.0-only
// Original overlapping Hann/FFT/mean-IQ-power stream, with explicit loop metadata.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fft_program.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::test::spectral_stream {
inline constexpr Word sample_rate = 48000, size = 1024, hop = 480, channels = 15;
inline constexpr Word hops_per_second = sample_rate / hop, stream_hops = 2 * hops_per_second;
inline constexpr Word history_words = size - hop, input_base = 0x10000, input_stride = 96624;
inline constexpr Word window_base = 0x3000, power_base = 0x4000, power_words = 16 * size;
inline constexpr Word cursor_base = 8, power_cursor_address = 24, progress_address = 25;
inline constexpr Word outer_do = 0x06df00;
inline constexpr Word window_steps = 7, power_steps = 6;
inline constexpr std::array<Word, window_steps> window_body{
    0x47da00, 0xf09800, 0x2000c0, 0x2000b8, 0x200011, 0x200019, 0xbb3900};
inline constexpr std::array<Word, power_steps> power_body{
    0xf09800, 0x200080, 0x200092, 0x200022, 0x200011, 0x565900};

struct Program {
  fft::Program code, transform;
  Progress hop_progress;
  Word hop_body = 0, hop_end = 0, channel_body = 0, channel_end = 0, power_body = 0;
};

inline Word pc(const fft::Program &code) {
  return code.entry + static_cast<Word>(code.words.size());
}

inline void window(Program &program) {
  auto &code = program.code;
  constexpr Word load_cursor = 0x60db00;
  constexpr Word copy_cursor = 0x221400;
  code.words.insert(code.words.end(), {load_cursor, copy_cursor});
  for (const Word index : {1U, 5U}) {
    fft::immediate(code, {.selector = fft::address_register_base + index, .value = fft::data_base});
  }
  fft::immediate(code, {.selector = fft::address_register_base + 2, .value = window_base});
  fft::immediate(code, {.selector = fft::offset_register_base, .value = history_words});
  const auto ending = fft::loop(code, size);
  code.words.insert(code.words.end(), window_body.begin(), window_body.end());
  code.words[ending] = pc(code) - 1;
  constexpr Word rewind_input = 0x204000;
  constexpr Word save_cursor = 0x605f00;
  code.words.insert(code.words.end(), {rewind_input, save_cursor});
}

inline void power(Program &program) {
  auto &code = program.code;
  for (const Word index : {0U, 4U}) {
    fft::immediate(code,
                   {.selector = fft::address_register_base + index, .value = fft::output_base});
  }
  constexpr Word load_output_cursor = 0x618000 | (power_cursor_address << 8);
  code.words.push_back(load_output_cursor);
  const auto ending = fft::loop(code, size);
  program.power_body = pc(code);
  code.words.insert(code.words.end(), power_body.begin(), power_body.end());
  code.words[ending] = pc(code) - 1;
  constexpr Word save_output_cursor = 0x610000 | (power_cursor_address << 8);
  code.words.push_back(save_output_cursor);
}

inline Program program() {
  Program result;
  auto &code = result.code;
  code.words = {outer_do, 0};
  result.hop_body = pc(code);
  for (const Word index : {3U, 7U}) {
    fft::immediate(code, {.selector = fft::address_register_base + index, .value = cursor_base});
  }
  const auto channel_ending = fft::loop(code, channels);
  result.channel_body = pc(code);
  window(result);
  result.transform = fft::make_at({.size = size, .entry = pc(code)});
  const auto &words = result.transform.words;
  code.words.insert(code.words.end(), words.begin(), words.end());
  power(result);
  result.channel_end = pc(code) - 1;
  code.words[channel_ending] = result.channel_end;
  constexpr Word read_progress = 0x0444bf;
  constexpr Word store_progress = 0x440000 | (progress_address << 8);
  code.words.insert(code.words.end(), {read_progress, store_progress});
  result.hop_end = pc(code) - 1;
  code.words[1] = result.hop_end;
  constexpr Word channel_overhead = 14;
  constexpr Word channel_extra_cycles = 15;
  constexpr Word hop_overhead = 5;
  constexpr Word hop_extra_cycles = 7;
  const auto samples = std::uint64_t{size} * (window_steps + power_steps);
  result.hop_progress = {
      .instructions = hop_overhead + (channels * (result.transform.progress.instructions + samples +
                                                  channel_overhead)),
      .estimated_cycles = hop_overhead + hop_extra_cycles +
                          (channels * (result.transform.progress.estimated_cycles + samples +
                                       channel_overhead + channel_extra_cycles))};
  return result;
}
} // namespace dsp56300::test::spectral_stream
