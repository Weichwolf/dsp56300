// SPDX-License-Identifier: GPL-3.0-only
#include "alu_publication_support.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::alu_publication_test {
namespace {
constexpr Word operation_mask = 0xf7, sign_main = 0x800000;
constexpr Word op_not = 0x17, op_lsr = 0x23, op_lsl = 0x33, op_ror = 0x27, op_rol = 0x37;
constexpr Word op_asr = 0x22, op_asl = 0x32, op_test = 3, op_max = 0x1d, op_maxm = 0x15;
constexpr Word op_absolute = 0x26, op_negate = 0x36, op_round = 0x11, op_clear = 0x13;
constexpr Word op_tfr = 1;

void logical(Expected &output, const Registers &before, const Word code) {
  const auto kind = code & 7;
  const auto operation = code & operation_mask;
  const auto raw = output.destination_b ? before.b : before.a;
  auto main = static_cast<Word>(raw / (std::uint64_t{1} << word_bits)) & word_mask;
  const auto input =
      static_cast<Word>(static_cast<std::uint64_t>(sequence_model::source(before, code)) /
                        (std::uint64_t{1} << word_bits)) &
      word_mask;
  output.data_mask = main_mask;
  output.status_mask = logical_flags;
  bool carry = false;
  if (operation == op_not) {
    main = word_mask - main;
  } else if (operation == op_lsr || operation == op_ror) {
    carry = main % 2 != 0;
    main /= 2;
    if (operation == op_ror && (before.sr & flag_c) != 0) { main += sign_main; }
    output.status_mask |= flag_c;
  } else if (operation == op_lsl || operation == op_rol) {
    carry = main >= sign_main;
    main = (main * 2) & word_mask;
    if (operation == op_rol && (before.sr & flag_c) != 0) { ++main; }
    output.status_mask |= flag_c;
  } else if (kind == 2) {
    main |= input;
  } else if (kind == 3) {
    main ^= input;
  } else {
    main &= input;
  }
  Word flags = main == 0 ? flag_z : 0;
  if (main >= sign_main) { flags |= flag_n; }
  if (carry) { flags |= flag_c; }
  output.state.sr = (before.sr & ~output.status_mask) | flags;
  (output.destination_b ? output.state.b : output.state.a) =
      (raw & ~main_mask) | (std::uint64_t{main} << word_bits);
}

sequence_model::Arithmetic arithmetic(const Registers &before, const Word code) {
  const auto operation = code & operation_mask;
  const auto raw = (code & 8U) != 0 ? before.b : before.a;
  const auto value = signed56(raw);
  if ((code & 0x80U) != 0) { return sequence_model::multiplication(before, code); }
  if (operation == op_clear) { return {}; }
  if (operation == op_test) { return {.full = value, .compare = true}; }
  if (operation == op_asr || operation == op_asl) {
    const bool left = operation == op_asl;
    const auto result = left ? value * 2 : (value / 2) - (value % 2 < 0 ? 1 : 0);
    return {.full = result,
            .overflow = sequence_model::overflowed(result),
            .carry_changed = true,
            .carry = left ? raw >= sign56 : raw % 2 != 0};
  }
  if (operation == op_absolute || operation == op_negate || operation == op_round) {
    sequence_model::Arithmetic result{.full = value};
    if (operation == op_negate || (operation == op_absolute && value < 0)) { result.full = -value; }
    if (operation == op_round) { sequence_model::rounded(result, before.sr); }
    result.overflow = result.overflow || sequence_model::overflowed(result.full);
    return result;
  }
  return sequence_model::addition(before, code);
}

void maximum(Expected &output, const Registers &before, const Word operation) {
  auto left = signed56(before.a);
  auto right = signed56(before.b);
  if (operation == op_maxm) {
    if (left < 0) { left = -left; }
    if (right < 0) { right = -right; }
  }
  output.destination_b = true;
  output.write_data = left >= right;
  output.status_mask = flag_c;
  output.state.sr = (before.sr & ~flag_c) | (output.write_data ? 0 : flag_c);
  if (output.write_data) { output.state.b = before.a; }
}
} // namespace

Registers seed(const unsigned index) {
  auto state = register_model::seed(0);
  constexpr std::array<std::array<std::uint64_t, 4>, 6> values{
      {{{0, 0, 0, 0}},
       {{0x0000007fffffff, 0xff800000000000, 0x123456400000, 0x800000654321}},
       {{0x7fffffffffffff, 0x80000000000000, 0x800000800000, 0x7fffff7fffff}},
       {{0x00400000000000, 0x00800000000000, 0x400000400000, 0x400000400000}},
       {{0x12000010abcdef, 0x121234562468ac, 0x13579b654321, 0x2468acfedcba}},
       {{0xff123456789abc, 0xff123456789abc, 0xfffffffffffe, 0x800001000001}}}};
  const auto row = values.at(index);
  state.a = row[0];
  state.b = row[1];
  state.x = row[2];
  state.y = row[3];
  state.pc = boot;
  constexpr std::array flags{
      flag_z | flag_u, status_byte, flag_c | flag_l, flag_c | flag_s, Word{0}, flag_e | flag_n};
  state.sr = flags.at(index);
  return state;
}

Expected expected(const Registers &before, const Word code) {
  Expected output{.state = before, .destination_b = (code & 8U) != 0};
  const auto operation = code & operation_mask;
  const auto kind = code & 7U;
  const auto selector = (code >> 4) & 7U;
  const bool product = (code & 0x80U) != 0;
  const bool binary_logic = !product && selector >= 4 && (kind == 2 || kind == 3 || kind == 6);
  if (binary_logic || operation == op_not || operation == op_lsr || operation == op_lsl ||
      operation == op_ror || operation == op_rol) {
    logical(output, before, code);
    return output;
  }
  if (code == op_max || code == op_maxm) {
    output.supported = (before.sr & sm) == 0;
    maximum(output, before, code);
    return output;
  }
  if (operation == op_tfr || (!product && selector >= 4 && kind == 1)) {
    (output.destination_b ? output.state.b : output.state.a) =
        static_cast<std::uint64_t>(sequence_model::source(before, code)) & accumulator_mask;
    return output;
  }
  auto result = arithmetic(before, code);
  output.write_data = !result.compare;
  output.supported = !result.compare || (before.sr & sm) == 0;
  const auto limited = saturation_model::apply(
      static_cast<std::uint64_t>(result.full) & accumulator_mask,
      {.status = before.sr, .rounded = operation == op_round || ((code & 0x81U) == 0x81U)});
  result.full = signed56(limited.value);
  result.overflow = result.overflow || limited.limited;
  output.status_mask =
      arithmetic_flags | (result.carry_changed ? flag_c : 0) | (result.overflow ? flag_l : 0);
  if (output.write_data) {
    (output.destination_b ? output.state.b : output.state.a) = limited.value;
  }
  sequence_model::flags(output.state, result);
  return output;
}
} // namespace dsp56300::test::alu_publication_test
