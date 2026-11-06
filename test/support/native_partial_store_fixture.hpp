// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace dsp56300::test::native_partial {
inline constexpr unsigned memory_words = 128, address = 0x20, offset = 3;
inline constexpr Word sentinel = 0x13579b, masked_status = 0x300;
inline constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
inline constexpr std::array selectors{8U, 9U, 10U, 11U, 12U, 13U};
inline constexpr std::array modes{0U, 0x400U, 0x800U, 0x200000U, 0x100000U, 0x100400U};
inline constexpr std::array values{std::uint64_t{0},
                                   std::uint64_t{0x007fffff000000},
                                   std::uint64_t{0xff800000ffffff},
                                   std::uint64_t{0x7fffffffffffff},
                                   std::uint64_t{0x80000000000000},
                                   std::uint64_t{0x00123456765432}};

struct Store {
  unsigned selector = 12, mode = 4, index = 0;
  Space space = Space::x;
  Word status = masked_status;
  std::uint64_t a = values.back();
};

[[nodiscard]] Word opcode(Store input);
[[nodiscard]] Registers seed(Store input);
[[nodiscard]] Word partial_word(const Registers &state, unsigned selector);
[[nodiscard]] Word effective_address(Store input);
[[nodiscard]] Registers after(Store input);

inline constexpr std::array domains{std::string_view{"audio-mac"},
                                    std::string_view{"image-add"},
                                    std::string_view{"communications-xor"}};

struct Timing {
  double seconds = 0;
  std::uint64_t instructions = 0;
};

// Shared independently modeled streams for functional checks and paired timing.
[[nodiscard]] Timing workload(std::string_view domain, bool partial, unsigned repetitions);
}
