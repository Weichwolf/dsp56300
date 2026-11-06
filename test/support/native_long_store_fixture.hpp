// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace dsp56300::test::native_long {
inline constexpr unsigned words = 128, address = 32, offset = 3;
inline constexpr Word sentinel = 0x13579b;
inline constexpr Word default_status = 0x300;
inline constexpr std::uint64_t default_value = 0x00123456765432;
inline constexpr std::uint64_t mask56 = 0xffffffffffffff;

struct Input {
  unsigned selector = 4, mode = 4, index = 0;
  Word status = default_status;
  std::uint64_t value = default_value;
  bool short_address = false;
};

struct Result {
  Registers state;
  std::array<Word, 2> data;
};

[[nodiscard]] Word opcode(Input input);
[[nodiscard]] Registers seed(Input input);
[[nodiscard]] Word effective_address(Input input);
[[nodiscard]] Result expected(Input input);

inline constexpr std::array domains{std::string_view{"audio-mac"},
                                    std::string_view{"image-add"},
                                    std::string_view{"communications-xor"}};

struct Timing {
  double seconds = 0;
  std::uint64_t instructions = 0;
};

[[nodiscard]] Timing workload(std::string_view domain, bool long_store, unsigned repetitions);
void boundary_cases();
} // namespace dsp56300::test::native_long
