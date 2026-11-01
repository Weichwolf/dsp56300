// SPDX-License-Identifier: GPL-3.0-only
// Original driven feedback cascade with continuously retained quantized history.
#pragma once
#include "dsp56300/core.hpp"
#include "iir_program.hpp"
#include "iir_stream_program.hpp"
#include "support.hpp"

#include <utility>
#include <vector>

namespace dsp56300::test::nonlinear_stream {
inline constexpr Word sample_rate = 48000, block_frames = 64, stream_frames = 2 * sample_rate;
inline constexpr Word channels = 204, stages = 4, signal_period = 32;
inline constexpr Word input_base = 0x10000, state_base = 0x1000, output_base = 0x2000;
inline constexpr Word coefficient_base = 32, progress_address = 24, output_words = 1024;
inline constexpr Word input_words = channels * signal_period, state_words = channels * stages;
inline constexpr Word coefficient_words = stages * 2;
inline constexpr Word channel_instructions = (stages * 6) + 3;
inline constexpr Word frame_instructions = (channels * channel_instructions) + 7;
inline constexpr Word frame_cycles = frame_instructions + channels + 9;
inline constexpr Word sm = Word{1} << 20, loop_flag = Word{1} << 15;
inline constexpr Word set_sm = 0x0af974, clear_sm = 0x0af954;

struct Program {
  std::vector<Word> words;
  Word frame_body = 0, channel_body = 0, channel_end = 0, frame_end = 0;
};

inline Program program() {
  constexpr Word outer_do = 0x06df00;
  constexpr Word do_prefix = 0x060080;
  constexpr Word read_progress = 0x0445bf;
  constexpr Word store_progress = 0x450000 | (progress_address << 8);
  constexpr Word mpy_x0_y0 = 0xd0;
  constexpr Word macr_x1_y1 = 0x2000f3;
  constexpr unsigned read_cursor = 18;
  constexpr unsigned write_cursor = 19;
  constexpr unsigned coefficient_cursor = 20;
  iir_stream::Program code{.words = {outer_do, 0, set_sm}};
  iir_stream::immediate(code, {.reg = read_cursor, .value = state_base});
  iir_stream::immediate(code, {.reg = write_cursor, .value = state_base});
  code.words.insert(code.words.end(), {do_prefix | (channels << 8), 0});
  const auto ending_slot = code.words.size() - 1;
  const auto body = start + static_cast<Word>(code.words.size());
  code.words.push_back(iir_stream::memory({.reg = 4, .cursor = 0}));
  iir_stream::immediate(code, {.reg = coefficient_cursor, .value = coefficient_base});
  for (Word stage = 0; stage < stages; ++stage) {
    code.words.insert(code.words.end(),
                      {iir_stream::memory({.reg = 5, .cursor = 2}),
                       iir_stream::memory({.reg = 6, .cursor = 4, .y = true}),
                       iir_stream::memory({.reg = 7, .cursor = 4, .y = true, .alu = mpy_x0_y0}),
                       macr_x1_y1,
                       iir::move(14, 4).word,
                       iir_stream::memory({.reg = 4, .cursor = 3, .load = false})});
  }
  code.words.push_back(iir_stream::memory({.reg = 4, .cursor = 1, .load = false}));
  const auto ending = start + static_cast<Word>(code.words.size()) - 1;
  code.words[ending_slot] = ending;
  code.words.insert(code.words.end(), {clear_sm, read_progress, store_progress});
  const auto frame_end = start + static_cast<Word>(code.words.size()) - 1;
  code.words[1] = frame_end;
  return {.words = std::move(code.words),
          .frame_body = start + 2,
          .channel_body = body,
          .channel_end = ending,
          .frame_end = frame_end};
}
} // namespace dsp56300::test::nonlinear_stream
