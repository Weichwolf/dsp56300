// SPDX-License-Identifier: GPL-3.0-only
// Independent raw56 witnesses for compiler width certificates and table gaps.
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"
#include "value_ranges.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word clear_a = 0x200013, clear_b = 0x20001b, asr_a = 0x200022;
constexpr Word add_zero = 0x0140c0;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 10> inputs{0,
                                               1,
                                               saturation_model::positive,
                                               saturation_model::positive + 1,
                                               saturation_model::negative,
                                               sign56 - 1,
                                               sign56,
                                               accumulator_mask,
                                               0x7e123456abcdef,
                                               0x81123456789abc};
constexpr std::array<Word, 4> words{0, sign24 - 1, sign24, word_mask};
constexpr std::array<Word, 22> operations{0x13, 0x1b, 0x11, 0x19, 0x26, 0x2e, 0x36, 0x3e,
                                          0x01, 0x09, 0x41, 0x49, 0x10, 0x18, 0x14, 0x1c,
                                          0x40, 0x48, 0x21, 0x29, 0x25, 0x2d};

struct OpInput {
  Word opcode, mode;
};

detail::NativeOp operation(const OpInput input) {
  auto decoded = detail::decode(input.opcode);
  return {.instruction = decoded, .pc = start, .mode = input.mode};
}

struct Value {
  unsigned width;
  std::uint64_t raw;
};

bool contained(const Value input) {
  const auto width = input.width;
  const auto raw = input.raw;
  if (width == 0) { return raw == 0; }
  const auto bound = std::int64_t{1} << (width - 1);
  const auto value = signed56(raw);
  return value >= -bound && value < bound;
}

void witness(detail::NativeValueRanges ranges, const Registers &old, const Word code) {
  auto expected = old;
  auto unbounded = old;
  unbounded.sr &= ~saturation_model::mode;
  sequence_model::alu(unbounded, unbounded, code);
  if ((old.sr & saturation_model::mode) != 0) {
    saturation_model::alu(expected, old, code);
  } else {
    sequence_model::alu(expected, old, code);
  }
  const auto proof =
      ranges.advance(operation({.opcode = parallel_none | code, .mode = old.sr}), true, false);
  const auto raw = (code & 8U) != 0 ? expected.b : expected.a;
  require(proof.width <= accumulator_bits && contained({.width = proof.width, .raw = raw}),
          "Every width certificate contains the independent actual stored raw56 value");
  const auto form = code & sequence_model::alu_mask;
  const bool transfer =
      (code & 7U) == 1 && ((code >> 4) == 0 || (code >> 4) >= 4) && (code & 0x80U) == 0;
  if (!transfer && !proof.overflow_value()) {
    require((expected.sr & flag_v) == 0 && (expected.sr & flag_l) == (old.sr & flag_l),
            "Omitted overflow/round/limit work cannot lose V or any sticky-L event");
  }
  if (proof.saturation == detail::SaturationProof::canonical) {
    const auto source = signed56((code & 8U) != 0 ? unbounded.b : unbounded.a);
    constexpr auto lower = -(std::int64_t{1} << 47);
    constexpr auto upper = (std::int64_t{1} << 47) - 1;
    auto clamped = static_cast<std::uint64_t>(std::clamp(source, lower, upper)) & accumulator_mask;
    const bool rounded = (code & 0x81U) == 0x81U || form == sequence_model::round;
    if (rounded && source > upper) { clamped = saturation_model::rounded_positive; }
    require(raw == clamped,
            "Canonical-clamp admission equals the independent three-bit table and rounded limit");
  }
}

void probes(const detail::NativeValueRanges &ranges, Registers state, const Word mode) {
  for (const bool saturated : {false, true}) {
    state.sr = mode | status_byte | (saturated ? saturation_model::mode : 0);
    for (const auto code : operations) { witness(ranges, state, code); }
    for (Word code = 0x80; code <= status_byte; ++code) { witness(ranges, state, code); }
  }
}

struct WidthInput {
  unsigned width;
  std::uint64_t input;
  Word mode;
};

void width_shape(const WidthInput shape) {
  const auto width = shape.width;
  const auto input = shape.input;
  const auto mode = shape.mode;
  detail::NativeValueRanges ranges;
  auto state = register_model::seed(mode);
  state.a = input;
  state.sr = mode | status_byte;
  static_cast<void>(ranges.advance(operation({.opcode = clear_b, .mode = mode}), true, false));
  sequence_model::alu(state, state, clear_b);
  if (width == 0) {
    static_cast<void>(ranges.advance(operation({.opcode = clear_a, .mode = mode}), true, false));
    sequence_model::alu(state, state, clear_a);
  } else {
    for (unsigned step = width; step < accumulator_bits; ++step) {
      static_cast<void>(ranges.advance(operation({.opcode = asr_a, .mode = mode}), true, false));
      sequence_model::alu(state, state, asr_a);
    }
  }
  for (const auto word : words) {
    state.x = (std::uint64_t{word} << word_bits) | word;
    state.y = state.x ^ long_mask;
    probes(ranges, state, mode);
  }
}

void barriers() {
  detail::NativeValueRanges ranges;
  static_cast<void>(ranges.advance(operation({.opcode = clear_a, .mode = 0}), true, false));
  auto helper = operation({.opcode = 0, .mode = 0});
  static_cast<void>(ranges.advance(helper, false, false));
  const auto sm = saturation_model::mode;
  const auto after_helper =
      ranges.advance(operation({.opcode = add_zero, .mode = sm}), true, false);
  require(after_helper.width == accumulator_bits &&
              after_helper.saturation == detail::SaturationProof::general,
          "A helper erases facts before a later arithmetic operation");
  static_cast<void>(ranges.advance(operation({.opcode = clear_a, .mode = 0}), true, true));
  const auto after_observer =
      ranges.advance(operation({.opcode = add_zero, .mode = sm}), true, false);
  require(after_observer.width == accumulator_bits &&
              after_observer.saturation == detail::SaturationProof::general,
          "Observable transfers erase facts after independently bounded current arithmetic");
  detail::NativeValueRanges fresh;
  constexpr Word move_a_a = 0x21ce00;
  static_cast<void>(fresh.advance(operation({.opcode = move_a_a, .mode = sm}), true, false));
  const auto after_word = fresh.advance(operation({.opcode = add_zero, .mode = sm}), true, false);
  require(after_word.width == word_bits * 2 &&
              after_word.saturation == detail::SaturationProof::none,
          "An explicit bus-word move to an accumulator establishes a new canonical48 domain");
}

void old_parallel() {
  constexpr Word load_a_add_a_b = 0x56d818;
  constexpr std::uint64_t gap = 0x7e123456abcdef;
  detail::NativeValueRanges ranges;
  static_cast<void>(
      ranges.advance(operation({.opcode = clear_b, .mode = saturation_model::mode}), true, false));
  const auto proof = ranges.advance(
      operation({.opcode = load_a_add_a_b, .mode = saturation_model::mode}), true, false);
  auto old = register_model::seed(saturation_model::mode);
  old.a = gap;
  old.b = 0;
  auto expected = old;
  register_model::write(expected, 14, 1);
  saturation_model::alu(expected, old, load_a_add_a_b);
  require(expected.b == gap && proof.width == accumulator_bits &&
              proof.saturation == detail::SaturationProof::general,
          "Parallel ADD consumes old wide A, rather than the newly loaded word A");
  const auto next =
      ranges.advance(operation({.opcode = add_zero, .mode = saturation_model::mode}), true, false);
  require(next.width == word_bits * 2 && next.saturation == detail::SaturationProof::none,
          "The following instruction sees the post-move canonical A, independently of old B");
}

struct ImmediateInput {
  Word value, mode;
  bool destination_b = false, subtract = false, compare = false, short_form = false;
};

void immediate(const ImmediateInput input) {
  detail::NativeValueRanges ranges;
  auto old = register_model::seed(input.mode);
  old.a = old.b = 0;
  old.sr = input.mode | status_byte;
  old.x = input.value & word_mask;
  static_cast<void>(
      ranges.advance(operation({.opcode = clear_a, .mode = input.mode}), true, false));
  static_cast<void>(
      ranges.advance(operation({.opcode = clear_b, .mode = input.mode}), true, false));
  constexpr Word long_prefix = 0x0140c0;
  constexpr Word short_prefix = 0x014080;
  auto kind = input.subtract ? 4U : 0U;
  if (input.compare) { kind = 5; }
  const auto opcode = (input.short_form ? short_prefix | (input.value << 8) : long_prefix) | kind |
                      (input.destination_b ? 8U : 0U);
  auto op = operation({.opcode = opcode, .mode = input.mode});
  op.instruction.extension = input.value;
  require(op.instruction.operation == detail::Operation::immediate_alu &&
              op.instruction.words == (input.short_form ? 1U : 2U),
          "Immediate probes use independently declared short/long encodings");
  auto expected = old;
  constexpr Word x0 = 0x40;
  const auto alu = x0 | kind | (input.destination_b ? 8U : 0U);
  if ((input.mode & saturation_model::mode) != 0) {
    saturation_model::alu(expected, old, alu);
  } else {
    sequence_model::alu(expected, old, alu);
  }
  const auto proof = ranges.advance(op, true, false);
  auto raw = input.destination_b ? expected.b : expected.a;
  if (input.compare) {
    raw = static_cast<std::uint64_t>(sequence_model::addition(old, alu).full) & accumulator_mask;
  }
  require(contained({.width = proof.width, .raw = raw}),
          "Zero-minus-negative and every constant bit/sign preserve a sound immediate width");
  if (input.compare) {
    const auto following =
        ranges.advance(operation({.opcode = add_zero, .mode = input.mode}), true, false);
    require(following.width == 0,
            "Compare certificates do not overwrite the stored accumulator domain");
  }
}

void immediate_forms(const Word value, const Word mode) {
  for (const bool destination_b : {false, true}) {
    for (const bool subtract : {false, true}) {
      immediate(
          {.value = value, .mode = mode, .destination_b = destination_b, .subtract = subtract});
      if (value < 64) {
        immediate({.value = value,
                   .mode = mode,
                   .destination_b = destination_b,
                   .subtract = subtract,
                   .short_form = true});
      }
    }
    if ((mode & saturation_model::mode) == 0) {
      immediate({.value = value, .mode = mode, .destination_b = destination_b, .compare = true});
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::barriers();
    dsp56300::test::old_parallel();
    for (const auto mode : dsp56300::test::modes) {
      for (unsigned bit = 0; bit < dsp56300::test::word_bits; ++bit) {
        const auto one = dsp56300::Word{1} << bit;
        for (const auto value : {one, one ^ dsp56300::word_mask}) {
          dsp56300::test::immediate_forms(value, mode);
          dsp56300::test::immediate_forms(value, mode | dsp56300::test::saturation_model::mode);
        }
      }
      dsp56300::test::immediate_forms(dsp56300::word_mask, mode);
      dsp56300::test::immediate_forms(dsp56300::word_mask,
                                      mode | dsp56300::test::saturation_model::mode);
      dsp56300::test::immediate_forms(0, mode);
      dsp56300::test::immediate_forms(0, mode | dsp56300::test::saturation_model::mode);
    }
    for (unsigned width = 0; width <= dsp56300::test::accumulator_bits; ++width) {
      for (const auto input : dsp56300::test::inputs) {
        for (const auto mode : dsp56300::test::modes) {
          dsp56300::test::width_shape({.width = width, .input = input, .mode = mode});
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent value-width certificate checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
