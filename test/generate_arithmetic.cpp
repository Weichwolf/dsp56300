// SPDX-License-Identifier: GPL-3.0-only
// Independent integer expectations: this generator never executes or links a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word mac = 0xd2, negate = 0x36, adc_x = 0x21, sbc_x = 0x25, rnd = 0x11;
constexpr unsigned fractional_bits = 47;
constexpr auto maximum56 = static_cast<std::int64_t>(sign56 - 1);
constexpr auto minimum56 = -static_cast<std::int64_t>(sign56);
constexpr std::array<Word, 7> words{0, 1, 2, sign24 - 1, sign24, word_mask - 1, word_mask};
constexpr std::array<std::uint64_t, 7> accumulators{0,
                                                    1,
                                                    sign56 - 1,
                                                    sign56,
                                                    accumulator_mask,
                                                    std::uint64_t{1} << fractional_bits,
                                                    accumulator_mask - 1};

struct FlagInput {
  std::uint64_t raw = 0;
  Word mode = 0;
  bool overflowed = false;
};

Word flags(const FlagInput input) {
  unsigned fractional = fractional_bits;
  if ((input.mode & mode_down) != 0) { ++fractional; }
  if ((input.mode & mode_up) != 0) { --fractional; }
  const auto top = input.raw >> fractional;
  const auto maximum_top = accumulator_mask >> fractional;
  Word result = input.raw == 0 ? flag_z : 0;
  if (input.raw >= sign56) { result |= flag_n; }
  if (top != 0 && top != maximum_top) { result |= flag_e; }
  if (((input.raw >> fractional) & 1) == ((input.raw >> (fractional - 1)) & 1)) {
    result |= flag_u;
  }
  if (input.overflowed) { result |= flag_v | flag_l; }
  return result;
}

struct ArithmeticCase {
  Registers initial;
  std::int64_t full = 0;
  Word opcode = 0, status = 0;
};

void append(std::vector<BinaryCase> &cases, const ArithmeticCase &input) {
  BinaryCase item;
  item.name = "independent integer arithmetic opcode " + std::to_string(input.opcode);
  item.initial = input.initial;
  item.expected = input.initial;
  item.expected.pc = start + 1;
  item.expected.a = static_cast<std::uint64_t>(input.full) & accumulator_mask;
  item.expected.sr = input.status;
  item.before.push_back({.words = {parallel_none | input.opcode}, .address = start});
  cases.push_back(std::move(item));
}

void multiply_cases(std::vector<BinaryCase> &cases) {
  for (const auto a : accumulators) {
    for (const auto x : words) {
      for (const auto y : words) {
        auto state = initial_state();
        state.a = a;
        state.x = x;
        state.y = y;
        state.sr = flag_c | flag_s;
        const auto full = signed56(a) + (2 * signed24(x) * signed24(y));
        const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
        const auto status =
            flags({.raw = raw, .overflowed = full < minimum56 || full > maximum56}) | flag_c |
            flag_s;
        append(cases, {.initial = state, .full = full, .opcode = mac, .status = status});
      }
    }
  }
}

void negate_cases(std::vector<BinaryCase> &cases) {
  for (const auto a : accumulators) {
    for (const auto carry : {Word{0}, flag_c}) {
      auto state = initial_state();
      state.a = a;
      state.sr = carry;
      const auto full = -signed56(a);
      const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
      const auto status = flags({.raw = raw, .overflowed = a == sign56}) | carry;
      append(cases, {.initial = state, .full = full, .opcode = negate, .status = status});
    }
  }
}

struct CarryCase {
  std::uint64_t a = 0, x = 0;
  Word carry = 0;
  bool subtract = false;
};

void carry_case(std::vector<BinaryCase> &cases, const CarryCase input) {
  auto state = initial_state();
  state.a = input.a;
  state.x = input.x;
  state.sr = input.carry;
  constexpr auto long_sign = std::uint64_t{1} << (word_bits * 2 - 1);
  const auto signed_x = input.x < long_sign ? static_cast<std::int64_t>(input.x)
                                            : static_cast<std::int64_t>(input.x) -
                                                  (std::int64_t{1} << (word_bits * 2));
  const auto full = signed56(input.a) + (input.subtract ? -signed_x : signed_x) +
                    (input.subtract ? -static_cast<std::int64_t>(input.carry) : input.carry);
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto unsigned_source = static_cast<std::uint64_t>(signed_x) & accumulator_mask;
  const bool carried = input.subtract ? input.a < unsigned_source + input.carry
                                      : input.a + unsigned_source + input.carry > accumulator_mask;
  const auto status = flags({.raw = raw, .overflowed = full < minimum56 || full > maximum56}) |
                      (carried ? flag_c : 0);
  append(
      cases,
      {.initial = state, .full = full, .opcode = input.subtract ? sbc_x : adc_x, .status = status});
}

void carry_cases(std::vector<BinaryCase> &cases) {
  constexpr std::array<std::uint64_t, 3> long_words{0, 1, long_mask};
  for (const auto subtract : {false, true}) {
    for (const auto a : accumulators) {
      for (const auto x : long_words) {
        for (const auto carry : {Word{0}, flag_c}) {
          carry_case(cases, {.a = a, .x = x, .carry = carry, .subtract = subtract});
        }
      }
    }
  }
}

struct RoundInput {
  std::int64_t value = 0;
  unsigned bits = 0;
  bool rm = false;
};

std::int64_t rounded(const RoundInput input) {
  const auto divisor = std::int64_t{1} << input.bits;
  auto whole = input.value / divisor;
  auto remainder = input.value % divisor;
  if (remainder < 0) {
    --whole;
    remainder += divisor;
  }
  if (remainder > divisor / 2 || (remainder == divisor / 2 && (input.rm || whole % 2 != 0))) {
    ++whole;
  }
  return whole * divisor;
}

struct RoundCase {
  std::int64_t whole = 0;
  Word mode = 0;
  int offset = 0;
  bool rm = false;
};

void round_case(std::vector<BinaryCase> &cases, const RoundCase input) {
  const unsigned bits =
      word_bits + (input.mode == mode_down ? 1U : 0U) - (input.mode == mode_up ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  const auto value = (input.whole * unit) + (unit / 2) + input.offset;
  auto state = initial_state();
  state.a = static_cast<std::uint64_t>(value) & accumulator_mask;
  state.sr = input.mode | (input.rm ? mode_rm : 0) | flag_c;
  const auto full = rounded({.value = value, .bits = bits, .rm = input.rm});
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  append(cases,
         {.initial = state,
          .full = full,
          .opcode = rnd,
          .status = state.sr | flags({.raw = raw, .mode = input.mode})});
}

void round_cases(std::vector<BinaryCase> &cases) {
  for (const Word mode : {Word{0}, mode_down, mode_up}) {
    for (const bool rm : {false, true}) {
      for (std::int64_t whole = -3; whole <= 3; ++whole) {
        for (const auto offset : {-1, 0, 1}) {
          round_case(cases, {.whole = whole, .mode = mode, .offset = offset, .rm = rm});
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_arithmetic OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::multiply_cases(cases);
    dsp56300::test::negate_cases(cases);
    dsp56300::test::carry_cases(cases);
    dsp56300::test::round_cases(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent arithmetic cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
