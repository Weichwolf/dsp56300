// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace dsp56300::test {
inline constexpr std::size_t memory_words = 1024;
inline constexpr Word start = 0x100;
inline constexpr unsigned word_bits = 24, accumulator_bits = 56;
inline constexpr std::uint64_t sign56 = std::uint64_t{1} << (accumulator_bits - 1);
inline constexpr Word sign24 = Word{1} << (word_bits - 1);
inline constexpr Word parallel_none = 0x200000;
inline constexpr Word flag_c = 1, flag_v = 2, flag_z = 4, flag_n = 8;
inline constexpr Word flag_u = 16, flag_e = 32, flag_l = 64, flag_s = 128;
inline constexpr Word mode_down = Word{1} << 10, mode_up = Word{1} << 11;
inline constexpr Word mode_rm = Word{1} << 21;
inline constexpr Word status_byte = 0xff;
inline std::size_t checks = 0;

constexpr std::string_view backend_name(const Backend backend) {
  switch (backend) {
    case Backend::reference: return "reference";
    case Backend::cached: return "cached";
    case Backend::jit: return "jit";
    default: return "invalid";
  }
}

inline void require(const bool condition, const std::string_view message) {
  ++checks;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

constexpr std::int64_t signed24(const Word value) noexcept {
  return value < sign24 ? value : static_cast<std::int64_t>(value) - (std::int64_t{1} << word_bits);
}

constexpr std::int64_t signed56(const std::uint64_t value) noexcept {
  return value < sign56 ? static_cast<std::int64_t>(value)
                        : static_cast<std::int64_t>(value) - (std::int64_t{1} << accumulator_bits);
}

inline Registers initial_state() {
  Registers state;
  state.pc = start;
  state.m.fill(word_mask);
  return state;
}

} // namespace dsp56300::test
