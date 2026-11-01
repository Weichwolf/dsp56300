// SPDX-License-Identifier: GPL-3.0-only
// Original continuous eleven-channel pipelined FIR with persistent input windows.
#pragma once
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <vector>

namespace dsp56300::test::fir_stream {
inline constexpr Word sample_rate = 48000, block_frames = 64, stream_frames = 2 * sample_rate;
inline constexpr Word channels = 11, taps = 507, input_stride = 96528;
inline constexpr Word input_base = 0x1000, coefficient_base = 0x400, output_base = 0x800;
inline constexpr Word cursor_base = 8, progress_address = 24, output_words = 1024;
inline constexpr Word channel_instructions = taps + 8;
inline constexpr Word frame_instructions = (channels * channel_instructions) + 5;
inline constexpr Word frame_cycles = frame_instructions + (4 * channels) + 7;
inline constexpr Word loop_flag = Word{1} << 15;
inline constexpr Word outer_do = 0x06df00, loop_prefix = 0x060080;
inline constexpr Word pipelined_mac = 0xf098d2, rounded = 0x200011;

struct Program {
  std::vector<Word> words;
  Word frame_body = 0, channel_body = 0, channel_end = 0, frame_end = 0, mac_pc = 0;
};

inline Program program() {
  constexpr Word set_read_cursor = 0x62f400;
  constexpr Word set_write_cursor = 0x63f400;
  constexpr Word load_input_cursor = 0x60da00;
  constexpr Word preload_clear = 0xf09813;
  constexpr Word last_mac = 0x2000d2;
  constexpr Word store_output = 0x565900;
  constexpr Word rewind_input = 0x204000;
  constexpr Word store_input_cursor = 0x605b00;
  constexpr Word rewind_coefficients = 0x204400;
  constexpr Word read_progress = 0x0445bf;
  constexpr Word store_progress = 0x450000 | (progress_address << 8);
  Program result{.words = {outer_do, 0}, .frame_body = start + 2};
  result.words.insert(result.words.end(),
                      {set_read_cursor,
                       cursor_base,
                       set_write_cursor,
                       cursor_base,
                       loop_prefix | (channels << 8),
                       0});
  const auto channel_end_slot = result.words.size() - 1;
  result.channel_body = start + static_cast<Word>(result.words.size());
  const auto count = taps - 1;
  result.words.insert(result.words.end(),
                      {load_input_cursor,
                       preload_clear,
                       loop_prefix | ((count & status_byte) << 8) | (count >> 8),
                       0});
  const auto tap_end_slot = result.words.size() - 1;
  result.mac_pc = start + static_cast<Word>(result.words.size());
  result.words[tap_end_slot] = result.mac_pc;
  result.words.insert(result.words.end(),
                      {pipelined_mac,
                       last_mac,
                       rounded,
                       store_output,
                       rewind_input,
                       store_input_cursor,
                       rewind_coefficients});
  result.channel_end = start + static_cast<Word>(result.words.size()) - 1;
  result.words[channel_end_slot] = result.channel_end;
  result.words.insert(result.words.end(), {read_progress, store_progress});
  result.frame_end = start + static_cast<Word>(result.words.size()) - 1;
  result.words[1] = result.frame_end;
  return result;
}
} // namespace dsp56300::test::fir_stream
