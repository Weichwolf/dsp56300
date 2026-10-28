// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <bit>
#include <cstdint>

namespace dsp56300::detail {
struct AddressForm {
  unsigned mode = 0;
  unsigned index = 0;
  Word extra = 0;
};

struct AddressUpdate {
  unsigned index = 0;
  std::int32_t offset = 0;
};

struct ShiftControl {
  unsigned count = 1;
  bool destination_b = false, left = false;
};

struct AddOperands {
  std::uint64_t left = 0, right = 0, carry_in = 0;
  bool destination_b = false, subtract = false, compare = false, shift_overflow = false;
};

inline constexpr unsigned word_bits = 24;
inline constexpr unsigned double_word_bits = 48;
inline constexpr unsigned accumulator_bits = 56;
inline constexpr unsigned host_word_bits = 32;
inline constexpr unsigned host_sign_bit = 63;
inline constexpr unsigned padding_bits = 8;
inline constexpr std::uint64_t padding_mask = 0xffffffffffffff00;
inline constexpr Word sign24_bit = Word{1} << (word_bits - 1);
inline constexpr std::int32_t minimum_word = -static_cast<std::int32_t>(sign24_bit);
inline constexpr std::int32_t maximum_word = static_cast<std::int32_t>(sign24_bit - 1);
inline constexpr Word carry = 1, overflow = 2, zero = 4, negative = 8;
inline constexpr Word unnormalized = 16, extension = 32, limit = 64, growth = 128;
inline constexpr unsigned scaling_shift = 10;
inline constexpr Word scale_down = Word{1} << 10, scale_up = Word{1} << 11;
inline constexpr Word forever_flag = Word{1} << 16;
inline constexpr Word loop_flag = Word{1} << 15, rounding_mode = Word{1} << 21;
inline constexpr Word compatibility_mode = Word{1} << 13;
inline constexpr Word multiply_mode = Word{1} << 14;
inline constexpr Word arithmetic_16 = Word{1} << 17, arithmetic_saturation = Word{1} << 20;
inline constexpr Word cache_enable = Word{1} << 19, burst_enable = Word{1} << 10;
inline constexpr Word io_begin = 0xffff80;
inline constexpr Word numeric_mode_mask = scale_up | scale_down | compatibility_mode |
                                          multiply_mode | arithmetic_16 | arithmetic_saturation |
                                          rounding_mode;
inline constexpr unsigned stack_word_shift = 24;

constexpr std::int32_t signed_word(const Word word) noexcept {
  return std::bit_cast<std::int32_t>(word << padding_bits) >> padding_bits;
}

constexpr std::uint64_t aligned_word(const Word word) noexcept {
  return static_cast<std::uint64_t>(signed_word(word)) << host_word_bits;
}

constexpr std::uint64_t wrapped_add(const std::uint64_t left, const std::uint64_t right) noexcept {
  std::uint64_t result = 0;
  static_cast<void>(__builtin_add_overflow(left, right, &result));
  return result;
}

constexpr std::uint64_t wrapped_sub(const std::uint64_t left, const std::uint64_t right) noexcept {
  std::uint64_t result = 0;
  static_cast<void>(__builtin_sub_overflow(left, right, &result));
  return result;
}

constexpr Word address_add(const Word address, const std::int32_t offset) noexcept {
  return static_cast<Word>(static_cast<std::int64_t>(address) + offset) & word_mask;
}

// A/B are left aligned internally. Low padding bits stay zero after every operation.
struct alignas(64) Cpu {
  Registers reg;
  std::uint64_t instructions = 0, cycles = 0;
  Word repeat_remaining = 0, repeat_pc = 0, repeat_lc = 0, repeat_opcode = 0;
  bool repeat_loaded = false, repeat_completed = false;
  bool interrupt_inhibited = false;
};
} // namespace dsp56300::detail
