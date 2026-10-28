// SPDX-License-Identifier: GPL-3.0-only
// Original continuous multichannel filter bank; no per-block host state changes.
#pragma once
#include "dsp56300/core.hpp"
#include "iir_program.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace dsp56300::test::iir_stream {
inline constexpr Word sample_rate = 48000, block_frames = 64, stream_frames = 2 * sample_rate;
inline constexpr Word channels = 92, sections = 4, signal_period = 256;
inline constexpr Word input_base = 0x10000, state_base = 0x1000, output_base = 0x2000;
inline constexpr Word coefficient_base = 32, output_words = 1024;
inline constexpr Word input_words = channels * signal_period, state_words = channels * sections * 2;
inline constexpr Word coefficient_words = sections * iir::coefficients_per_section;
inline constexpr Word channel_instructions = 2 + (sections * 15);
inline constexpr Word frame_instructions = (channels * channel_instructions) + 13;
inline constexpr Word frame_cycles = frame_instructions + 6;
inline constexpr Word outer_do = 0x06df00;

struct Program {
  std::vector<Word> words;
  Word frame_body = 0, channel_body = 0, channel_end = 0, frame_end = 0;
};

struct Transfer {
  unsigned reg = 0, cursor = 0;
  bool load = true, y = false;
  Word alu = 0;
};

inline Word memory(const Transfer input) {
  constexpr Word indirect_postincrement = 0x5800;
  return iir::single(input.reg) | indirect_postincrement | (input.cursor << 8) |
         (input.load ? 0x8000 : 0) | (input.y ? 0x080000 : 0) | input.alu;
}

struct RegisterValue {
  unsigned reg = 0;
  Word value = 0;
};

inline void immediate(Program &program, const RegisterValue input) {
  constexpr Word immediate_extension = 0xf400;
  program.words.insert(program.words.end(),
                       {iir::single(input.reg) | immediate_extension, input.value});
}

inline void section(Program &program) {
  constexpr Word tfr_x1 = 0x61;
  constexpr Word tfr_y1 = 0x71;
  constexpr Word mac_x0 = 0xd2;
  constexpr Word subtract_x1 = 0xe6;
  constexpr Word round = 0x200011;
  constexpr Word clear = 0x13;
  program.words.insert(program.words.end(),
                       {memory({.reg = 5, .cursor = 2}),
                        memory({.reg = 7, .cursor = 2}),
                        memory({.reg = 6, .cursor = 4, .y = true, .alu = tfr_x1}),
                        memory({.reg = 6, .cursor = 4, .y = true, .alu = mac_x0}),
                        round,
                        iir::move(14, 5).word | tfr_y1,
                        memory({.reg = 6, .cursor = 4, .y = true, .alu = mac_x0}),
                        memory({.reg = 6, .cursor = 4, .y = true, .alu = subtract_x1}),
                        round,
                        memory({.reg = 14, .cursor = 3, .load = false, .alu = clear}),
                        memory({.reg = 6, .cursor = 4, .y = true, .alu = mac_x0}),
                        parallel_none | subtract_x1,
                        round,
                        memory({.reg = 14, .cursor = 3, .load = false}),
                        iir::move(5, 4).word});
}

inline Program program() {
  Program result{.words = {outer_do, 0}, .frame_body = start + 2};
  constexpr unsigned state_read = 18;
  immediate(result, {.reg = state_read, .value = state_base});
  constexpr unsigned state_write = 19;
  immediate(result, {.reg = state_write, .value = state_base});
  constexpr Word clear_b = 0x20001b;
  result.words.push_back(clear_b);
  result.words.insert(result.words.end(), {iir::do_prefix | (channels << 8), 0});
  const auto channel_end_slot = result.words.size() - 1;
  result.channel_body = start + static_cast<Word>(result.words.size());
  result.words.push_back(memory({.reg = 4, .cursor = 0}));
  for (Word stage = 0; stage < sections; ++stage) { section(result); }
  constexpr Word mix = 0x200048;
  constexpr Word shift_b = 0x20002a;
  constexpr Word round_b = 0x200019;
  result.words.push_back(mix);
  result.channel_end = start + static_cast<Word>(result.words.size()) - 1;
  result.words[channel_end_slot] = result.channel_end;
  constexpr unsigned normalization_shifts = 7;
  result.words.insert(result.words.end(), normalization_shifts, shift_b);
  result.words.push_back(round_b);
  result.words.push_back(memory({.reg = 15, .cursor = 1, .load = false}));
  result.frame_end = start + static_cast<Word>(result.words.size()) - 1;
  result.words[1] = result.frame_end;
  return result;
}
} // namespace dsp56300::test::iir_stream
