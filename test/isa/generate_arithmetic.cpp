// SPDX-License-Identifier: GPL-3.0-only
// Independent integer expectations: this generator never executes or links a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
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
constexpr Word changed_flags = flag_n | flag_z | flag_e | flag_u | flag_v;
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
  ((input.opcode & 8U) != 0 ? item.expected.b : item.expected.a) =
      static_cast<std::uint64_t>(input.full) & accumulator_mask;
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
  bool rm = false, destination_b = false;
};

void round_case(std::vector<BinaryCase> &cases, const RoundCase input) {
  const unsigned bits =
      word_bits + (input.mode == mode_down ? 1U : 0U) - (input.mode == mode_up ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  const auto value = (input.whole * unit) + (unit / 2) + input.offset;
  auto state = initial_state();
  (input.destination_b ? state.b : state.a) = static_cast<std::uint64_t>(value) & accumulator_mask;
  state.sr = input.mode | (input.rm ? mode_rm : 0) | flag_c;
  const auto full = rounded({.value = value, .bits = bits, .rm = input.rm});
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  append(cases,
         {.initial = state,
          .full = full,
          .opcode = rnd | (input.destination_b ? 8U : 0U),
          .status = state.sr | flags({.raw = raw, .mode = input.mode})});
}

void round_cases(std::vector<BinaryCase> &cases) {
  for (const Word mode : {Word{0}, mode_down, mode_up}) {
    for (const bool rm : {false, true}) {
      for (std::int64_t whole = -3; whole <= 3; ++whole) {
        for (const auto offset : {-1, 0, 1}) {
          for (const bool destination_b : {false, true}) {
            round_case(cases,
                       {.whole = whole,
                        .mode = mode,
                        .offset = offset,
                        .rm = rm,
                        .destination_b = destination_b});
          }
        }
      }
    }
  }
}

constexpr Word x0 = 0x123456, x1 = 0xfedcba, y0 = 0x345678, y1 = 0x876543;
constexpr std::uint64_t initial_x = (std::uint64_t{x1} << word_bits) | x0;
constexpr std::uint64_t initial_y = (std::uint64_t{y1} << word_bits) | y0;

struct ProductInput {
  Registers state;
  Word left = 0, right = 0, opcode = 0;
};

void product_case(std::vector<BinaryCase> &cases, const ProductInput &input) {
  const auto before = (input.opcode & 8U) != 0 ? input.state.b : input.state.a;
  const auto initial = (input.opcode & 2U) != 0 ? signed56(before) : 0;
  const auto term = 2 * signed24(input.left) * signed24(input.right);
  auto full = initial + ((input.opcode & 4U) != 0 ? -term : term);
  bool overflowed = full < minimum56 || full > maximum56;
  if ((input.opcode & 1U) != 0) {
    const auto wrapped = static_cast<std::uint64_t>(full) & accumulator_mask;
    const auto mode = input.state.sr;
    const unsigned bits =
        word_bits + ((mode & mode_down) != 0 ? 1U : 0U) - ((mode & mode_up) != 0 ? 1U : 0U);
    full = rounded({.value = signed56(wrapped), .bits = bits, .rm = (mode & mode_rm) != 0});
    overflowed = overflowed || full < minimum56 || full > maximum56;
  }
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto status =
      (input.state.sr & ~changed_flags) |
      flags({.raw = raw, .mode = input.state.sr & (mode_down | mode_up), .overflowed = overflowed});
  append(cases, {.initial = input.state, .full = full, .opcode = input.opcode, .status = status});
}

struct ProductConfig {
  std::uint64_t accumulator = 0;
  Word mode = 0;
  bool destination_b = false, carry = false, rounded = false;
};

void product_variants(std::vector<BinaryCase> &cases, const ProductConfig input) {
  // Table 12-16, with distinct words so bank/high/low selector mistakes are visible.
  struct Pair {
    Word selector, left, right;
  };

  constexpr std::array<Pair, 8> pairs{{{.selector = 0, .left = x0, .right = x0},
                                       {.selector = 1, .left = y0, .right = y0},
                                       {.selector = 2, .left = x1, .right = x0},
                                       {.selector = 3, .left = y1, .right = y0},
                                       {.selector = 4, .left = x0, .right = y1},
                                       {.selector = 5, .left = y0, .right = x0},
                                       {.selector = 6, .left = x1, .right = y0},
                                       {.selector = 7, .left = y1, .right = x1}}};
  auto state = initial_state();
  state.a = input.accumulator;
  state.b = input.accumulator;
  state.x = initial_x;
  state.y = initial_y;
  state.sr = input.mode | flag_s | changed_flags | (input.carry ? flag_c | flag_l : 0);
  const Word base = 0x80U | (input.destination_b ? 8U : 0U) | (input.rounded ? 1U : 0U);
  for (const auto pair : pairs) {
    for (const bool negative : {false, true}) {
      for (const bool accumulate : {false, true}) {
        product_case(cases,
                     {.state = state,
                      .left = pair.left,
                      .right = pair.right,
                      .opcode = base | (pair.selector << 4) | (negative ? 4U : 0U) |
                                (accumulate ? 2U : 0U)});
      }
    }
  }
}

void product_matrix(std::vector<BinaryCase> &cases, const bool round_result) {
  for (const auto accumulator : accumulators) {
    for (const Word scaling : {Word{0}, mode_down, mode_up}) {
      for (const Word rm : {Word{0}, mode_rm}) {
        if (!round_result && rm != 0) { continue; }
        for (const bool destination_b : {false, true}) {
          for (const bool carry : {false, true}) {
            product_variants(cases,
                             {.accumulator = accumulator,
                              .mode = scaling | rm,
                              .destination_b = destination_b,
                              .carry = carry,
                              .rounded = round_result});
          }
        }
      }
    }
  }
}

void round_boundary(std::vector<BinaryCase> &cases, const ProductConfig input) {
  auto state = initial_state();
  state.a = input.accumulator;
  state.b = input.accumulator;
  state.sr = input.mode | changed_flags | (input.carry ? flag_c | flag_s | flag_l : 0);
  const unsigned bits = word_bits + ((input.mode & mode_down) != 0 ? 1U : 0U) -
                        ((input.mode & mode_up) != 0 ? 1U : 0U);
  const auto full = rounded(
      {.value = signed56(input.accumulator), .bits = bits, .rm = (input.mode & mode_rm) != 0});
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto status =
      (state.sr & ~changed_flags) |
      flags({.raw = raw, .mode = input.mode, .overflowed = full < minimum56 || full > maximum56});
  const Word destination = input.destination_b ? 8U : 0U;
  append(cases, {.initial = state, .full = full, .opcode = rnd | destination, .status = status});
  // Zero-product MACR isolates rounding from multiplication for the same values.
  product_case(cases, {.state = state, .opcode = mac | 1U | destination});
}

void round_boundary_variants(std::vector<BinaryCase> &cases, const Word mode) {
  const unsigned bits =
      word_bits + ((mode & mode_down) != 0 ? 1U : 0U) - ((mode & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::uint64_t{1} << bits;
  const auto half = unit / 2;
  const std::array<std::uint64_t, 12> values{0,
                                             1,
                                             half - 1,
                                             half,
                                             half + 1,
                                             unit + half,
                                             sign56 - 1,
                                             sign56 - half,
                                             sign56 - half - 1,
                                             sign56,
                                             sign56 + 1,
                                             accumulator_mask};
  for (const auto accumulator : values) {
    for (const bool destination_b : {false, true}) {
      for (const bool carry : {false, true}) {
        round_boundary(cases,
                       {.accumulator = accumulator,
                        .mode = mode,
                        .destination_b = destination_b,
                        .carry = carry});
      }
    }
  }
}

void round_boundary_matrix(std::vector<BinaryCase> &cases) {
  for (const Word scaling : {Word{0}, mode_down, mode_up}) {
    for (const Word rm : {Word{0}, mode_rm}) { round_boundary_variants(cases, scaling | rm); }
  }
}

void absolute_case(std::vector<BinaryCase> &cases, const ProductConfig input) {
  auto state = initial_state();
  state.a = input.accumulator;
  state.b = input.accumulator;
  state.sr = input.mode | flag_s | changed_flags | (input.carry ? flag_c | flag_l : 0);
  const auto value = signed56(input.accumulator);
  const auto full = value < 0 ? -value : value;
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto status =
      (state.sr & ~changed_flags) |
      flags({.raw = raw, .mode = input.mode, .overflowed = input.accumulator == sign56});
  constexpr Word absolute = 0x26;
  append(cases,
         {.initial = state,
          .full = full,
          .opcode = absolute | (input.destination_b ? 8U : 0U),
          .status = status});
}

void absolute_matrix(std::vector<BinaryCase> &cases) {
  for (const auto accumulator : accumulators) {
    for (const Word mode : {Word{0}, mode_down, mode_up}) {
      for (const bool destination_b : {false, true}) {
        for (const bool carry : {false, true}) {
          absolute_case(cases,
                        {.accumulator = accumulator,
                         .mode = mode,
                         .destination_b = destination_b,
                         .carry = carry});
        }
      }
    }
  }
}

void transfer_case(std::vector<BinaryCase> &cases, const ProductConfig input, const Word selector) {
  auto state = initial_state();
  state.a = accumulators[3];
  state.b = accumulators[5];
  state.x = initial_x;
  state.y = initial_y;
  state.sr = input.mode | status_byte;
  std::int64_t value = signed56(input.destination_b ? state.a : state.b);
  if (selector != 0) {
    constexpr std::array<Word, 4> inputs{x0, y0, x1, y1};
    value = signed24(inputs.at(selector - 4)) * (std::int64_t{1} << word_bits);
  }
  append(cases,
         {.initial = state,
          .full = value,
          .opcode = (selector << 4) | (input.destination_b ? 8U : 0U) | 1U,
          .status = state.sr});
}

void transfer_matrix(std::vector<BinaryCase> &cases) {
  for (const Word mode : {Word{0}, mode_down, mode_up}) {
    for (const bool destination_b : {false, true}) {
      for (const Word selector : {0U, 4U, 5U, 6U, 7U}) {
        transfer_case(cases, {.mode = mode, .destination_b = destination_b}, selector);
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
    dsp56300::test::product_matrix(cases, false);
    dsp56300::test::product_matrix(cases, true);
    dsp56300::test::round_boundary_matrix(cases);
    dsp56300::test::absolute_matrix(cases);
    dsp56300::test::transfer_matrix(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent arithmetic cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
