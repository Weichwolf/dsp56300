// SPDX-License-Identifier: GPL-3.0-only
// Independent per-bit truth tables, serial middle-word shifts and raw56 assembly.
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::bfu_model {
inline constexpr Word sm = Word{1} << 20;
enum class Function : std::uint8_t {
  bit_and,
  bit_or,
  bit_xor,
  complement,
  left,
  right,
  rotate_left,
  rotate_right
};
enum class Form : std::uint8_t {
  parallel,
  short_immediate,
  long_immediate,
  shift_immediate,
  shift_register
};

struct Input {
  Function function = Function::bit_and;
  Form form = Form::parallel;
  Word value = 0;
  unsigned source = 0, count = 1;
  bool destination_b = false;
};

inline constexpr std::array controls{12U, 13U, 4U, 5U, 6U, 7U};
inline constexpr std::array control_codes{2U, 3U, 4U, 6U, 5U, 7U};
inline constexpr std::array source_codes{4U, 6U, 5U, 7U};
inline constexpr std::array<Word, 4> logic_codes{6, 2, 3, 0x17};
inline constexpr std::array<Word, 4> shift_codes{0x33, 0x23, 0x37, 0x27};

inline bool logical(const Function function) {
  return function <= Function::complement;
}

inline bool left(const Function function) {
  return function == Function::left || function == Function::rotate_left;
}

inline bool rotating(const Function function) {
  return function >= Function::rotate_left;
}

inline unsigned words(const Input input) {
  return input.form == Form::long_immediate ? 2U : 1U;
}

inline Word opcode(const Input input) {
  const auto function = static_cast<unsigned>(input.function);
  if (input.form == Form::parallel) {
    Word code = logical(input.function) ? logic_codes.at(function) : shift_codes.at(function - 4);
    if (input.function < Function::complement) { code |= source_codes.at(input.source) * 16; }
    return parallel_none | code | (input.destination_b ? 8U : 0U);
  }
  if (input.form == Form::short_immediate || input.form == Form::long_immediate) {
    if (input.function >= Function::complement) {
      throw std::logic_error("No immediate unary form");
    }
    constexpr Word short_code = 0x014080;
    constexpr Word long_code = 0x0140c0;
    const Word fields = logic_codes.at(function) | (input.destination_b ? 8U : 0U);
    return input.form == Form::long_immediate ? long_code | fields
                                              : short_code | (input.value * 256) | fields;
  }
  constexpr Word left_immediate = 0x0c1e80;
  constexpr Word right_immediate = 0x0c1ec0;
  constexpr Word left_register = 0x0c1e10;
  constexpr Word right_register = 0x0c1e30;
  const Word destination = input.destination_b ? 1U : 0U;
  if (input.form == Form::shift_immediate) {
    return (left(input.function) ? left_immediate : right_immediate) | (input.count * 2) |
           destination;
  }
  return (left(input.function) ? left_register : right_register) |
         (control_codes.at(input.source) * 2) | destination;
}

inline Word control(const Registers &old, const unsigned selector) {
  const std::array values{
      old.a >> word_bits, old.b >> word_bits, old.x, old.x >> word_bits, old.y, old.y >> word_bits};
  return static_cast<Word>(values.at(selector) & word_mask);
}

using Bits = std::array<bool, word_bits>;

inline Bits truth(const Bits &before, const Word source, const Function function) {
  constexpr std::array<unsigned, 4> tables{8, 14, 6, 3};
  const auto table = tables.at(static_cast<unsigned>(function));
  Bits result{};
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const auto pair = (before[bit] ? 2U : 0U) + ((source >> bit) % 2);
    result[bit] = ((table >> pair) % 2) != 0;
  }
  return result;
}

inline bool serial(Bits &bits, const Word status, const Input input, const unsigned count) {
  if (count > word_bits || (rotating(input.function) && count != 1)) {
    throw std::logic_error("Undefined BFU shift trace");
  }
  bool previous = (status & flag_c) != 0;
  for (unsigned step = 0; step < count; ++step) {
    const bool carried = left(input.function) ? bits.back() : bits.front();
    Bits next{};
    for (unsigned bit = 0; bit + 1 < word_bits; ++bit) {
      if (left(input.function)) {
        next[bit + 1] = bits[bit];
      } else {
        next[bit] = bits[bit + 1];
      }
    }
    next[left(input.function) ? 0 : word_bits - 1] = rotating(input.function) && previous;
    bits = next;
    previous = carried;
  }
  return count != 0 && previous;
}

inline void apply(Registers &state, const Registers &old, const Input input) {
  const auto raw = input.destination_b ? old.b : old.a;
  Bits bits{};
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    bits[bit] = ((raw >> (word_bits + bit)) % 2) != 0;
  }
  bool carried = false;
  if (logical(input.function)) {
    const std::array values{old.x, old.x >> word_bits, old.y, old.y >> word_bits};
    const auto word = input.form == Form::parallel
                          ? static_cast<Word>(values.at(input.source) & word_mask)
                          : input.value;
    bits = truth(bits, word, input.function);
  } else {
    constexpr Word count_mask = 31;
    const auto count =
        input.form == Form::shift_register ? control(old, input.source) & count_mask : input.count;
    carried = serial(bits, old.sr, input, count);
  }
  std::uint64_t result = 0;
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const bool middle = bit >= word_bits && bit < word_bits * 2;
    const bool value = middle ? bits[bit - word_bits] : ((raw >> bit) % 2) != 0;
    if (value) { result += std::uint64_t{1} << bit; }
  }
  (input.destination_b ? state.b : state.a) = result;
  Word flags = 0;
  bool zero = true;
  for (const bool bit : bits) { zero = zero && !bit; }
  if (zero) { flags |= flag_z; }
  if (bits.back()) { flags |= flag_n; }
  const auto changed = flag_n | flag_z | flag_v | (logical(input.function) ? 0U : flag_c);
  state.sr = (state.sr & ~changed) | flags | (carried ? flag_c : 0U);
}
} // namespace dsp56300::test::bfu_model
