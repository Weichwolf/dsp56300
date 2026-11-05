// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace dsp56300::test::long_load {
inline constexpr unsigned words = 128, address = 32, offset = 3;
inline constexpr unsigned word_bits = 24, double_word_bits = 48;
inline constexpr std::uint64_t mask56 = 0xffffffffffffff;
inline constexpr Word default_status = 0x300;
inline constexpr std::uint64_t default_value = 0x923456abcdef;

struct Input {
  unsigned selector = 4, mode = 4, index = 0;
  Word status = default_status;
  std::uint64_t value = default_value;
  bool short_address = false;
};

[[nodiscard]] Word opcode(Input input);
[[nodiscard]] Registers seed(Input input);
[[nodiscard]] Word effective_address(Input input);
void expected_load(Registers &state, Input input);
void expected_alu(Registers &state, const Registers &old, Word instruction);
void boundary_cases();
void repeated_cases();

inline constexpr std::array domains{std::string_view{"audio-mac"},
                                    std::string_view{"image-add"},
                                    std::string_view{"communications-xor"}};

struct Timing {
  double seconds = 0;
  std::uint64_t instructions = 0, native = 0, moves = 0, helpers = 0;
};

[[nodiscard]] Timing workload(std::string_view domain, bool use_long, unsigned repetitions);
} // namespace dsp56300::test::long_load
