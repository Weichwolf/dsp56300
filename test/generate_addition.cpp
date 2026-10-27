// SPDX-License-Identifier: GPL-3.0-only
// Independent finite-width sums, inequalities and register-bus expectations.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
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
using Cases = std::vector<BinaryCase>;
constexpr unsigned long_sign_bit = 47, word_shift = 24;
constexpr auto sign48 = std::uint64_t{1} << long_sign_bit;
constexpr Word changed = flag_c | flag_v | flag_z | flag_n | flag_e | flag_u;
constexpr Word accumulator_a = 14, accumulator_b = 15, address_r0 = 16;
constexpr std::array<std::uint64_t, 10> left_values{0,
                                                    1,
                                                    accumulator_mask,
                                                    sign56 - 1,
                                                    sign56,
                                                    sign56 + 1,
                                                    std::uint64_t{1} << 46,
                                                    std::uint64_t{1} << 47,
                                                    std::uint64_t{1} << 48,
                                                    0x0123456789abcd};
constexpr std::array<std::uint64_t, 8> accumulator_values{
    0, 1, 2, sign56 - 1, sign56, accumulator_mask, sign56 + 1, 0x0123456789abcd};
constexpr std::array<std::uint64_t, 8> long_values{
    0, 1, 2, sign48 - 1, sign48, long_mask, sign48 + 1, 0x123456fedcba};
constexpr std::array<std::uint64_t, 8> word_values{
    0, 1, 2, sign24 - 1, sign24, word_mask, sign24 + 1, 0x123456};

enum class Kind : std::uint8_t { add, subtract, compare, adc, sbc };
enum class Move : std::uint8_t { none, source, destination, bus };

struct Form {
  Kind kind = Kind::add;
  unsigned source = 1;
  bool destination_b = false;
};

struct Input {
  Form form;
  std::uint64_t left = 0, right = 0;
  Word status = 0;
  Move move = Move::none;
};

struct FlagInput {
  std::int64_t full = 0;
  Word status = 0;
};

Word arithmetic_flags(const FlagInput input) {
  const auto full = input.full;
  const auto status = input.status;
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto value = signed56(raw);
  const unsigned position =
      long_sign_bit + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
  const auto boundary = std::int64_t{1} << position;
  Word flags = value == 0 ? flag_z : 0;
  if (value < 0) { flags |= flag_n; }
  if (value < -boundary || value >= boundary) { flags |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { flags |= flag_u; }
  if (full < -static_cast<std::int64_t>(sign56) || full >= static_cast<std::int64_t>(sign56)) {
    flags |= flag_v | flag_l;
  }
  return flags;
}

std::int64_t source_operand(Registers &state, const Input input) {
  const auto group = input.form.source;
  if (group < 2) {
    (input.form.destination_b ? state.a : state.b) = input.right;
    return signed56(input.right);
  }
  if (group < 4) {
    (group == 2 ? state.x : state.y) = input.right;
    return input.right < sign48
               ? static_cast<std::int64_t>(input.right)
               : static_cast<std::int64_t>(input.right) - (std::int64_t{1} << (long_sign_bit + 1));
  }
  auto &bank = group == 4 || group == 6 ? state.x : state.y;
  if (group < 6) {
    bank = (bank & ~std::uint64_t{word_mask}) | input.right;
  } else {
    bank = (bank & word_mask) | (input.right << word_bits);
  }
  return signed24(static_cast<Word>(input.right)) * (std::int64_t{1} << word_shift);
}

Word opcode(const Form form) {
  Word kind = 0;
  switch (form.kind) {
    case Kind::add: kind = 0; break;
    case Kind::subtract: kind = 4; break;
    case Kind::compare:
    case Kind::sbc: kind = 5; break;
    case Kind::adc: kind = 1; break;
    default: throw std::logic_error("Invalid addition kind");
  }
  return (form.source << 4) | (form.destination_b ? 8U : 0U) | kind;
}

unsigned source_register(const Form form) {
  constexpr std::array<unsigned, 8> data{0, 0, 4, 6, 4, 6, 5, 7};
  if (form.source < 2) { return form.destination_b ? accumulator_a : accumulator_b; }
  return data.at(form.source);
}

Word parallel_move(Registers &state, const Input input) {
  if (input.move == Move::none) { return parallel_none; }
  const auto accumulator = input.form.destination_b ? accumulator_b : accumulator_a;
  const auto source = input.move == Move::destination ? source_register(input.form) : accumulator;
  auto target = source_register(input.form);
  if (input.move == Move::destination) { target = accumulator; }
  if (input.move == Move::bus) { target = address_r0; }
  const auto value = register_model::read(state, source);
  register_model::write(state, target, value);
  return parallel_none | (source << 13) | (target << 8);
}

void append(Cases &cases, const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(0);
  item.initial.sr = input.status | (changed & ~flag_c);
  (input.form.destination_b ? item.initial.b : item.initial.a) = input.left;
  const auto operand = source_operand(item.initial, input);
  const bool with_carry = input.form.kind == Kind::adc || input.form.kind == Kind::sbc;
  const bool subtract = input.form.kind == Kind::subtract || input.form.kind == Kind::compare ||
                        input.form.kind == Kind::sbc;
  const std::int64_t carry = with_carry && (input.status & flag_c) != 0 ? 1 : 0;
  const auto full = signed56(input.left) + (subtract ? -operand - carry : operand + carry);
  const auto unsigned_operand = static_cast<std::uint64_t>(operand) & accumulator_mask;
  const auto carry_word = static_cast<std::uint64_t>(carry);
  const auto carry_out = subtract ? input.left < unsigned_operand + carry_word
                                  : input.left + unsigned_operand + carry_word > accumulator_mask;
  item.expected = item.initial;
  const auto prefix = parallel_move(item.expected, input);
  if (input.form.kind != Kind::compare) {
    (input.form.destination_b ? item.expected.b : item.expected.a) =
        static_cast<std::uint64_t>(full) & accumulator_mask;
  }
  item.expected.sr = (item.expected.sr & ~changed) |
                     arithmetic_flags({.full = full, .status = input.status}) |
                     (carry_out ? flag_c : 0);
  item.expected.pc = start + 1;
  const auto instruction = prefix | opcode(input.form);
  item.name = "Independent sum opcode=" + std::to_string(instruction) +
              " left=" + std::to_string(input.left) + " right=" + std::to_string(input.right) +
              " status=" + std::to_string(input.status);
  item.before.push_back({.words = {instruction}, .address = start});
  cases.push_back(std::move(item));
}

std::vector<Form> forms() {
  std::vector<Form> result;
  for (const bool destination_b : {false, true}) {
    for (unsigned source = 0; source < 8; ++source) {
      if (source != 0) {
        result.push_back({.kind = Kind::add, .source = source, .destination_b = destination_b});
        result.push_back(
            {.kind = Kind::subtract, .source = source, .destination_b = destination_b});
      }
      if (source == 0 || source >= 4) {
        result.push_back({.kind = Kind::compare, .source = source, .destination_b = destination_b});
      }
      if (source == 2 || source == 3) {
        result.push_back({.kind = Kind::adc, .source = source, .destination_b = destination_b});
        result.push_back({.kind = Kind::sbc, .source = source, .destination_b = destination_b});
      }
    }
  }
  return result;
}

std::array<std::uint64_t, 8> right_values(const Form form) {
  if (form.source < 2) { return accumulator_values; }
  return form.source < 4 ? long_values : word_values;
}

void matrix(Cases &cases, const Form form) {
  for (const Word mode : {Word{0}, mode_down, mode_up}) {
    for (const Word sticky : {Word{0}, flag_c, flag_s | flag_l, flag_s | flag_l | flag_c}) {
      for (const auto left : left_values) {
        for (const auto right : right_values(form)) {
          append(cases, {.form = form, .left = left, .right = right, .status = mode | sticky});
        }
      }
    }
  }
}

void parallel_corners(Cases &cases, const Input input) {
  constexpr std::array right_indices{3U, 4U, 5U};
  for (const auto left : {std::uint64_t{0}, sign56 - 1, sign56}) {
    for (const auto index : right_indices) {
      for (const auto move : {Move::source, Move::destination, Move::bus}) {
        auto variant = input;
        variant.left = left;
        variant.right = right_values(input.form).at(index);
        variant.move = move;
        append(cases, variant);
      }
    }
  }
}

void generate(const char *const path) {
  Cases cases;
  for (const auto form : forms()) {
    matrix(cases, form);
    for (const Word mode : {Word{0}, mode_down, mode_up}) {
      for (const Word carry : {Word{0}, flag_c}) {
        parallel_corners(cases, {.form = form, .status = mode | mode_rm | carry});
      }
    }
  }
  std::cout << cases.size() << " independently modeled addition cases\n";
  write_fixture(path, std::move(cases));
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_addition OUTPUT.d56"); }
    dsp56300::test::generate(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
