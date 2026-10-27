// SPDX-License-Identifier: GPL-3.0-only
// Project-authored scaled radix-2 DIF program and reverse-carry output permutation.
#pragma once
#include "dsp56300/core.hpp"
#include "fft_twiddles.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace dsp56300::test::fft {
inline constexpr Word data_base = 0x400, twiddle_base = 0x1000, output_base = 0x2000;
inline constexpr Word loop_flag = Word{1} << 15;
inline constexpr unsigned offset_register_base = 24, address_register_base = 16;
inline constexpr Word stage_instructions = 11, stage_cycles = 25;
inline constexpr Word group_instructions = 7, group_cycles = 13;
inline constexpr Word copy_instructions = 11, copy_cycles = 23;
inline constexpr std::array<Word, 29> butterfly{
    0xc08000, 0xc5a100, 0x200041, 0x200060, 0x200059, 0x200078, 0x200022, 0x20002a,
    0x200011, 0x200019, 0xbb1800, 0x200041, 0x200064, 0x200059, 0x20007c, 0x200022,
    0x20002a, 0x200011, 0x200019, 0x21c400, 0x21e600, 0xd5ca00, 0x2000a0, 0x2000b6,
    0x2000c8, 0x2000ea, 0x200011, 0x200019, 0xbb3900};

struct Stage {
  Word length = 0, groups = 0, step = 0, outer_end = 0, inner_body = 0;
};

struct Program {
  std::vector<Word> words;
  std::vector<Stage> stages;
  Progress progress;
  Word copy_body = 0;
};

struct RegisterValue {
  unsigned selector = 0;
  Word value = 0;
};

inline void immediate(Program &program, const RegisterValue input) {
  constexpr Word prefix = 0x40f400;
  constexpr Word high_register_mask = 0x18;
  constexpr unsigned high_register_shift = 17;
  program.words.push_back(prefix | ((input.selector & 7U) << 16) |
                          ((input.selector & high_register_mask) << high_register_shift));
  program.words.push_back(input.value);
}

inline std::size_t loop(Program &program, const Word count) {
  constexpr Word prefix = 0x060080;
  constexpr Word low_count_mask = 0xff;
  program.words.push_back(prefix | ((count & low_count_mask) << 8) | (count >> 8));
  program.words.push_back(0);
  return program.words.size() - 1;
}

inline void stage(Program &program, Stage input) {
  const auto half = input.length / 2;
  for (const unsigned index : {0U, 4U, 1U, 5U}) {
    immediate(program,
              {.selector = address_register_base + index,
               .value = data_base + (index == 1 || index == 5 ? half : 0)});
    immediate(program, {.selector = offset_register_base + index, .value = half});
  }
  for (const unsigned index : {2U, 6U}) {
    immediate(program, {.selector = offset_register_base + index, .value = input.step});
  }
  const auto outer = loop(program, input.groups);
  immediate(program, {.selector = address_register_base + 2, .value = twiddle_base});
  immediate(program, {.selector = address_register_base + 6, .value = twiddle_base});
  const auto inner = loop(program, half);
  input.inner_body = start + static_cast<Word>(program.words.size());
  program.words.insert(program.words.end(), butterfly.begin(), butterfly.end());
  program.words[inner] = start + static_cast<Word>(program.words.size()) - 1;
  constexpr Word update = 0x204800;
  for (const Word index : {0U, 1U, 4U, 5U}) { program.words.push_back(update | (index << 8)); }
  input.outer_end = start + static_cast<Word>(program.words.size()) - 1;
  program.words[outer] = input.outer_end;
  program.stages.push_back(input);
  // Ten two-word setup moves, one outer DO, and G*(two moves, inner DO,
  // H butterflies, four post-updates). Every butterfly word costs one base unit.
  program.progress.instructions +=
      stage_instructions + input.groups * (group_instructions + half * butterfly.size());
  program.progress.estimated_cycles +=
      stage_cycles + input.groups * (group_cycles + half * butterfly.size());
}

inline Program make_program(const Word size) {
  if (size < 16 || size > maximum_size || !std::has_single_bit(size)) {
    throw std::invalid_argument("FFT size must be a power of two in [16,1024]");
  }
  Program program;
  for (Word length = size; length > 1; length /= 2) {
    stage(program, {.length = length, .groups = size / length, .step = size / length});
  }
  for (const unsigned index : {0U, 4U, 1U, 5U}) {
    immediate(program,
              {.selector = address_register_base + index,
               .value = index == 0 || index == 4 ? data_base : output_base});
  }
  for (const unsigned index : {0U, 4U}) {
    immediate(program, {.selector = offset_register_base + index, .value = size / 2});
    constexpr Word clear_modifier = 0x0500a0;
    program.words.push_back(clear_modifier | index);
  }
  const auto copy = loop(program, size);
  program.copy_body = start + static_cast<Word>(program.words.size());
  constexpr Word load_reversed = 0xd08800;
  constexpr Word store_linear = 0xb03900;
  program.words.insert(program.words.end(), {load_reversed, store_linear});
  program.words[copy] = start + static_cast<Word>(program.words.size()) - 1;
  for (const Word index : {0U, 4U}) {
    constexpr Word load_modifier = 0x05f420;
    program.words.insert(program.words.end(), {load_modifier | index, word_mask});
  }
  program.progress.instructions += copy_instructions + 2 * size;
  program.progress.estimated_cycles += copy_cycles + 2 * size;
  return program;
}
} // namespace dsp56300::test::fft
