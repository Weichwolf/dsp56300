// SPDX-License-Identifier: GPL-3.0-only
// Independent extended SM product fields and raw signed/mixed arithmetic.
#pragma once
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace dsp56300::test::saturation_products {
inline constexpr Word immediate_base = 0x0141c0, power_base = 0x0100c0;
inline constexpr Word mixed_mac = 0x012680, mixed_mpy = 0x012780, double_base = 0x012480;
inline constexpr std::array<std::array<Word, 4>, 4> pair_code{
    {{0, 10, 13, 4}, {2, 8, 6, 15}, {5, 14, 1, 11}, {12, 7, 3, 9}}};
inline constexpr std::array immediate_code{0U, 2U, 1U, 3U}, power_code{1U, 3U, 2U, 0U};

inline Word data(const Registers &state, const unsigned selector) {
  const auto bank = selector < 2 ? state.x : state.y;
  return static_cast<Word>(bank >> (selector % 2 == 0 ? 0U : word_bits)) & word_mask;
}

struct WordWrite {
  unsigned selector;
  Word value;
};

inline void put(Registers &state, const WordWrite input) {
  const auto selector = input.selector;
  const auto value = input.value;
  auto &bank = selector < 2 ? state.x : state.y;
  const auto shift = selector % 2 == 0 ? 0U : word_bits;
  bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
}

struct Operation {
  unsigned left = 0, right = 2, signs = 1;
  Word power = 0, coefficient = sign24, variant = 0;
  bool destination_b = false, double_product = false, accumulate = true;
};

inline void store(std::vector<BinaryCase> &cases,
                  const Registers &initial,
                  sequence_model::Arithmetic result,
                  std::vector<Word> words,
                  const Operation operation,
                  const bool bypass) {
  const auto limited =
      saturation_model::apply(static_cast<std::uint64_t>(result.full) & accumulator_mask,
                              {.status = bypass ? initial.sr & ~saturation_model::mode : initial.sr,
                               .rounded = (operation.variant & 1U) != 0});
  BinaryCase item;
  item.initial = item.expected = initial;
  (operation.destination_b ? item.expected.b : item.expected.a) = limited.value;
  result.full = signed56(limited.value);
  result.overflow = result.overflow || limited.limited;
  sequence_model::flags(item.expected, result);
  item.expected.pc += static_cast<Word>(words.size());
  item.progress.estimated_cycles = words.size();
  item.before = {{.words = std::move(words), .address = start}};
  item.name = bypass ? "SM mixed unsigned product bypasses arithmetic saturation"
                     : "SM extended signed product saturates only after arithmetic/rounding";
  cases.push_back(std::move(item));
}

inline void
mixed(std::vector<BinaryCase> &cases, const Registers &initial, const Operation operation) {
  const auto before = operation.destination_b ? initial.b : initial.a;
  auto value = operation.accumulate ? signed56(before) : 0;
  if (operation.double_product) {
    const auto divisor = std::int64_t{1} << word_bits;
    const auto original = signed56(before);
    value = original / divisor - (original % divisor < 0 ? 1 : 0);
  }
  const auto raw_left = data(initial, operation.left);
  const auto raw_right = data(initial, operation.right);
  const auto left = operation.signs == 2 ? raw_left : signed24(raw_left);
  const auto right = operation.signs == 0 ? signed24(raw_right) : raw_right;
  const auto term = 2 * left * right;
  value += (operation.variant & 4U) != 0 ? -term : term;
  const sequence_model::Arithmetic result{.full = value,
                                          .overflow = sequence_model::overflowed(value)};
  Word opcode = operation.accumulate ? mixed_mac : mixed_mpy;
  if (operation.double_product) { opcode = double_base; }
  opcode |= pair_code.at(operation.left).at(operation.right) |
            (operation.destination_b ? 0x20U : 0U) | ((operation.variant & 4U) != 0 ? 0x10U : 0U) |
            (operation.signs == 2 ? 0x40U : 0U) |
            (operation.double_product && operation.signs != 0 ? 0x100U : 0U);
  store(cases,
        initial,
        result,
        {opcode},
        operation,
        !operation.double_product || operation.signs != 0);
}

inline void
ordinary(std::vector<BinaryCase> &cases, const Registers &initial, const Operation operation) {
  const auto coefficient = operation.power == 0
                               ? signed24(operation.coefficient)
                               : std::int64_t{1} << (word_bits - 1 - operation.power);
  auto full = 2 * signed24(data(initial, operation.left)) * coefficient;
  if ((operation.variant & 4U) != 0) { full = -full; }
  if ((operation.variant & 2U) != 0) {
    full += signed56(operation.destination_b ? initial.b : initial.a);
  }
  sequence_model::Arithmetic result{.full = full, .overflow = sequence_model::overflowed(full)};
  if ((operation.variant & 1U) != 0) { sequence_model::rounded(result, initial.sr); }
  const bool power = operation.power != 0;
  const Word opcode = (power ? power_base | (operation.power << 8) : immediate_base) |
                      ((power ? power_code : immediate_code).at(operation.left) << 4) |
                      (operation.destination_b ? 8U : 0U) | operation.variant;
  std::vector words{opcode};
  if (!power) { words.push_back(operation.coefficient); }
  store(cases, initial, result, std::move(words), operation, false);
}
} // namespace dsp56300::test::saturation_products
