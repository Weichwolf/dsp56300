// SPDX-License-Identifier: GPL-3.0-only
// Independent widened products and quotient/remainder rounding; no CPU executor.
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
enum class Family : std::uint8_t { parallel, immediate, power, mac, round };
constexpr Word sm = 0x100000, power = 22;
constexpr std::int64_t coefficient = 2;
constexpr Word round_code = 0x11, x_marker = 0x654321, y_marker = 0xabcdef;
constexpr std::uint64_t wrapped = 0x7fffffff000000;
constexpr Word immediate_base = 0x0141c0, power_base = 0x0100c0, parallel_mac = 0x2000d2;
constexpr Word changed = flag_n | flag_z | flag_e | flag_u | flag_v;
constexpr std::uint64_t positive_limit = 0x007fffffffffff, negative_limit = 0xff800000000000;
constexpr std::uint64_t rounded_positive = 0x007fffff000000, other = 0x123456789abcde;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array families{
    Family::parallel, Family::immediate, Family::power, Family::mac, Family::round};
constexpr std::array seeds{sign56, sign56 + (std::uint64_t{1} << word_bits), sign56 - 1};
constexpr std::array<Word, 3> incoming{0, flag_c | flag_v, status_byte};

struct Operands {
  Word left, right;
};

constexpr std::array operands{Operands{.left = 1, .right = sign24},
                              Operands{.left = 1, .right = word_mask},
                              Operands{.left = 0, .right = sign24}};

struct Input {
  Family family;
  std::uint64_t accumulator;
  Operands source;
  Word status;
  bool destination_b, subtract;
};

struct Quantize {
  std::int64_t value;
  Word status;
};

std::int64_t quantize(const Quantize input) {
  const auto value = input.value;
  const auto status = input.status;
  const auto bits =
      word_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  auto whole = value / unit;
  auto remainder = value % unit;
  if (remainder < 0) {
    --whole;
    remainder += unit;
  }
  if (remainder > unit / 2 ||
      (remainder == unit / 2 && ((status & mode_rm) != 0 || whole % 2 != 0))) {
    ++whole;
  }
  return whole * unit;
}

struct FlagInput {
  std::uint64_t raw;
  Word status;
};

Word flags(const FlagInput input) {
  const auto raw = input.raw;
  const auto status = input.status;
  const auto point = (word_bits * 2 - 1) + ((status & mode_down) != 0 ? 1U : 0U) -
                     ((status & mode_up) != 0 ? 1U : 0U);
  const auto top = raw >> point;
  Word result = raw == 0 ? flag_z : 0;
  if ((raw & sign56) != 0) { result |= flag_n; }
  if (top != 0 && top != accumulator_mask >> point) { result |= flag_e; }
  if (((raw >> point) & 1U) == ((raw >> (point - 1)) & 1U)) { result |= flag_u; }
  return result;
}

struct Result {
  std::uint64_t value;
  Word status;
};

Result mathematical(const Input input) {
  auto full = signed56(input.accumulator);
  if (input.family != Family::round) {
    const auto right = input.family == Family::power ? coefficient : signed24(input.source.right);
    const auto term = 2 * signed24(input.source.left) * right;
    full += input.subtract ? -term : term;
  }
  if (input.family != Family::mac) { full = quantize({.value = full, .status = input.status}); }
  bool overflowed =
      full < -static_cast<std::int64_t>(sign56) || full > static_cast<std::int64_t>(sign56 - 1);
  auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  if ((input.status & sm) != 0) {
    constexpr std::array action{0U, 1U, 1U, 1U, 2U, 2U, 2U, 0U};
    constexpr unsigned extension = 48;
    constexpr unsigned main_sign = 47;
    const auto selector =
        ((raw / sign56) * 4) + (((raw >> extension) & 1U) * 2) + ((raw >> main_sign) & 1U);
    const auto limit = action.at(selector);
    if (limit != 0) {
      overflowed = true;
      if (limit == 2) {
        raw = negative_limit;
      } else if (input.family == Family::mac) {
        raw = positive_limit;
      } else {
        raw = rounded_positive;
      }
    }
  }
  return {.value = raw,
          .status = (input.status & ~changed) | flags({.raw = raw, .status = input.status}) |
                    (overflowed ? flag_v | flag_l : 0U)};
}

std::vector<Word> code(const Input input) {
  const Word destination = input.destination_b ? 8U : 0U;
  const Word sign = input.subtract ? 4U : 0U;
  switch (input.family) {
    case Family::parallel: return {parallel_mac | 1U | destination | sign};
    case Family::immediate: return {immediate_base | 3U | destination | sign, input.source.right};
    case Family::power: return {power_base | (power << 8) | (1U << 4) | 3U | destination | sign};
    case Family::mac: return {parallel_mac | destination | sign};
    case Family::round: return {parallel_none | round_code | destination};
    default: throw std::logic_error("Unknown rounded product family");
  }
}

void append(std::vector<BinaryCase> &cases, const Input input) {
  BinaryCase item;
  item.initial = item.expected = initial_state();
  item.initial.sr = input.status;
  item.initial.x = (std::uint64_t{x_marker} << word_bits) | input.source.left;
  item.initial.y = (std::uint64_t{y_marker} << word_bits) | input.source.right;
  item.initial.a = input.destination_b ? other : input.accumulator;
  item.initial.b = input.destination_b ? input.accumulator : other;
  item.expected = item.initial;
  const auto wanted = mathematical(input);
  (input.destination_b ? item.expected.b : item.expected.a) = wanted.value;
  item.expected.sr = wanted.status;
  const auto program = code(input);
  item.expected.pc += static_cast<Word>(program.size());
  item.progress.estimated_cycles = program.size();
  item.before.push_back({.words = program, .address = start});
  item.name = "Wide rounded product final flags family=" +
              std::to_string(static_cast<unsigned>(input.family)) +
              " case=" + std::to_string(cases.size());
  cases.push_back(std::move(item));
}

void group(std::vector<BinaryCase> &cases, Input input) {
  for (const auto status : incoming) {
    for (const auto mode : modes) {
      for (const Word saturation : {0U, sm}) {
        for (const bool destination_b : {false, true}) {
          input.status = status | mode | saturation;
          input.destination_b = destination_b;
          append(cases, input);
        }
      }
    }
  }
}

void literal_controls() {
  const Input input{.family = Family::parallel,
                    .accumulator = sign56,
                    .source = {.left = 1, .right = sign24},
                    .status = mode_down,
                    .destination_b = false,
                    .subtract = false};
  const auto fused = mathematical(input);
  constexpr Word fused_status = 0x438;
  constexpr Word raw_status = 0x472;
  constexpr Word rounded_status = 0x47a;
  require(fused.value == sign56 && fused.status == fused_status,
          "Literal fused tie rounds back to the signed56 minimum without new V/L");
  auto control = input;
  control.family = Family::mac;
  const auto mac = mathematical(control);
  require(mac.value == wrapped && mac.status == raw_status,
          "The same unrounded MAC genuinely overflows and sets V/L");
  control.family = Family::round;
  control.accumulator = wrapped;
  const auto rnd = mathematical(control);
  require(rnd.value == sign56 && rnd.status == rounded_status,
          "Standalone RND of the wrapped word genuinely overflows and sets V/L");
}

std::vector<BinaryCase> fixtures() {
  literal_controls();
  std::vector<BinaryCase> cases;
  for (const auto family : families) {
    for (const auto accumulator : seeds) {
      for (const auto source : operands) {
        for (const bool subtract : {false, true}) {
          if (subtract && (family == Family::mac || family == Family::round)) { continue; }
          group(cases,
                {.family = family,
                 .accumulator = accumulator,
                 .source = source,
                 .status = 0,
                 .destination_b = false,
                 .subtract = subtract});
        }
      }
    }
  }
  append(cases,
         {.family = Family::round,
          .accumulator = wrapped,
          .source = {.left = 1, .right = sign24},
          .status = mode_down,
          .destination_b = false,
          .subtract = false});
  return cases;
}
}
}

int main(const int argc, char **const argv) {
  try {
    if (argc != 2) { throw std::invalid_argument("Provide the authored fixture output path"); }
    auto cases = dsp56300::test::fixtures();
    std::cout << cases.size() << " independently widened rounded-product cases\n";
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
