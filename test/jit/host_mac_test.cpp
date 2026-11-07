// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "jit/mac_batch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {
using dsp56300::Word;
using dsp56300::detail::MacBatchInput;
using dsp56300::detail::MacBatchState;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

constexpr std::int64_t signed_word(const Word value) {
  constexpr Word sign = 0x800000;
  constexpr Word mask = 0xffffff;
  constexpr std::int64_t range = 0x1000000;
  return std::int64_t{value & mask} - ((value & sign) != 0 ? range : 0);
}

MacBatchState reference(MacBatchState state, const MacBatchInput input) {
  constexpr std::int64_t sign = std::int64_t{1} << 55U;
  constexpr std::uint64_t mask = (std::uint64_t{1} << 56U) - 1;
  constexpr Word word_mask = 0xffffff;
  for (std::size_t index = 0; index < input.x.size(); ++index) {
    const auto product = 2 * signed_word(state.x) * signed_word(state.y);
    const auto result = state.accumulator + (input.subtract ? -product : product);
    state.overflowed = result < -sign || result >= sign;
    state.limited = state.limited || state.overflowed;
    const auto raw = static_cast<std::uint64_t>(result) & mask;
    state.accumulator = static_cast<std::int64_t>(raw) - ((raw & sign) != 0 ? 2 * sign : 0);
    state.x = input.x[index] & word_mask;
    state.y = input.y[index] & word_mask;
  }
  return state;
}

void verify(const MacBatchState initial, const MacBatchInput input) {
  const auto expected = reference(initial, input);
  auto actual = initial;
  dsp56300::detail::native_mac_batch(actual, input);
  require(actual.accumulator == expected.accumulator && actual.x == expected.x &&
              actual.y == expected.y && actual.limited == expected.limited &&
              actual.overflowed == expected.overflowed,
          "Host MAC batch differs from independent ordered signed56 arithmetic");
}

void cases() {
  require(dsp56300::detail::native_mac_available(), "Host MAC backend is unavailable");
  constexpr std::size_t words = 128;
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  constexpr std::array<Word, 8> values{
      0, 1, 0xffffff, 0x800000, 0x7fffff, 0x123456, 0xff800000, 0xff7fffff};
  constexpr auto sign = std::int64_t{1} << 55U;
  constexpr std::array accumulators{std::int64_t{0}, -sign, sign - 1, -sign + 1, sign - 2};
  constexpr std::array<std::size_t, 7> counts{0, 4, 8, 16, 20, 64, words};
  for (const auto left : values) {
    for (const auto right : values) {
      x.fill(left);
      y.fill(right);
      for (const auto accumulator : accumulators) {
        for (const auto count : counts) {
          for (const bool subtract : {false, true}) {
            for (const bool limited : {false, true}) {
              verify({.accumulator = accumulator,
                      .x = right,
                      .y = left,
                      .limited = limited,
                      .overflowed = true},
                     {.x = std::span(x).first(count),
                      .y = std::span(y).first(count),
                      .subtract = subtract});
            }
          }
        }
      }
    }
  }
  for (std::size_t index = 0; index < words; ++index) {
    x[index] = values[index % values.size()];
    y[index] = values[(index * 3) % values.size()];
  }
  for (const auto accumulator : accumulators) {
    for (const bool subtract : {false, true}) {
      verify({.accumulator = accumulator, .x = values[3], .y = values[3]},
             {.x = x, .y = y, .subtract = subtract});
    }
  }
}
}

int main() {
  try {
    cases();
    std::cout << "Host MAC products, ordered overflow, sticky limit and retained inputs passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
