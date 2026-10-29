// SPDX-License-Identifier: GPL-3.0-only
// Standalone finite-width bit/integer model; never links or executes the DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
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
constexpr unsigned main_shift = 24;
constexpr std::uint64_t main_mask = std::uint64_t{word_mask} << main_shift;
constexpr std::array<std::uint64_t, 9> values{0,
                                              1,
                                              long_mask,
                                              std::uint64_t{1} << 47,
                                              sign56 - 1,
                                              sign56,
                                              accumulator_mask,
                                              0x123456abcdef12,
                                              0xab654321123456};
constexpr std::array<unsigned, 9> arithmetic_counts{0, 1, 2, 7, 17, 24, 40, 54, 55};
constexpr std::array<unsigned, 7> logical_counts{0, 1, 2, 7, 16, 23, 24};
constexpr std::array<Word, 3> modes{0, mode_down, mode_up};
constexpr Word sticky = flag_s;
constexpr Word register_asl = 0x0c1e40, register_asr = 0x0c1e60;
constexpr Word register_lsl = 0x0c1e10, register_lsr = 0x0c1e30;
constexpr std::int64_t minimum56 = -static_cast<std::int64_t>(sign56);
constexpr std::int64_t maximum56 = static_cast<std::int64_t>(sign56 - 1);

struct StatusInput {
  std::int64_t value = 0;
  Word mode = 0;
};

Word arithmetic_status(const StatusInput input) {
  const auto value = input.value;
  const auto mode = input.mode;
  const auto bits =
      static_cast<unsigned>(47 + (mode == mode_down ? 1 : 0) - (mode == mode_up ? 1 : 0));
  const auto boundary = std::int64_t{1} << bits;
  const auto raw = static_cast<std::uint64_t>(value) & accumulator_mask;
  Word status = value == 0 ? flag_z : 0;
  if (value < 0) { status |= flag_n; }
  if (value < -boundary || value >= boundary) { status |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { status |= flag_u; }
  return status;
}

Registers seed() {
  auto state = initial_state();
  constexpr std::uint64_t initial_x = 0xace02413579b;
  constexpr std::uint64_t initial_y = 0x159abe2468cd;
  constexpr Word old_loop_address = 0x70;
  constexpr Word old_loop_count = 0x17;
  state.a = values.back();
  state.b = values[values.size() - 2];
  state.x = initial_x;
  state.y = initial_y;
  state.la = old_loop_address;
  state.lc = old_loop_count;
  for (std::size_t index = 0; index < agu_registers; ++index) {
    state.r[index] = static_cast<Word>(index) + 0x200;
    state.n[index] = static_cast<Word>(index) + 0x80;
  }
  for (std::size_t index = 0; index < stack_entries; ++index) {
    state.stack[index] = long_mask - index;
  }
  return state;
}

void append(Cases &cases, BinaryCase item, const Word opcode) {
  item.name = "independent finite-width model, opcode " + std::to_string(opcode);
  item.before.push_back({.words = {opcode}, .address = start});
  item.expected.pc = start + 1;
  cases.push_back(std::move(item));
}

struct ShiftCase {
  std::uint64_t value = 0;
  unsigned count = 1;
  Word opcode = 0, mode = 0, incoming_carry = 0;
  bool destination_b = false, source_b = false, left = false, logical = false, rotate = false;
};

void model_arithmetic(BinaryCase &item, const ShiftCase input) {
  auto raw = input.source_b ? item.initial.b : item.initial.a;
  bool overflowed = false;
  Word carry = 0;
  for (unsigned bit = 0; bit < input.count; ++bit) {
    if (input.left) {
      const auto next = (raw * 2) & accumulator_mask;
      carry = raw >= sign56 ? flag_c : 0;
      overflowed = overflowed || (raw >= sign56) != (next >= sign56);
      raw = next;
    } else {
      carry = static_cast<Word>(raw % 2);
      const auto signed_value = signed56(raw);
      const auto halved = (signed_value / 2) - (signed_value % 2 < 0 ? 1 : 0);
      raw = static_cast<std::uint64_t>(halved) & accumulator_mask;
    }
  }
  (input.destination_b ? item.expected.b : item.expected.a) = raw;
  item.expected.sr = input.mode | sticky |
                     arithmetic_status({.value = signed56(raw), .mode = input.mode}) |
                     (overflowed ? flag_v | flag_l : 0) | carry;
}

void model_logical(BinaryCase &item, const ShiftCase input) {
  auto &value = input.destination_b ? item.expected.b : item.expected.a;
  auto word = static_cast<Word>((value / (std::uint64_t{1} << main_shift)) & word_mask);
  Word carry = 0;
  for (unsigned bit = 0; bit < input.count; ++bit) {
    const auto previous = bit == 0 ? input.incoming_carry : carry;
    carry = static_cast<Word>(input.left ? word >= sign24 : word % 2 != 0);
    word = input.left ? (word * 2) & word_mask : word / 2;
    if (input.rotate) { word |= input.left ? previous : previous * sign24; }
  }
  value = (value & ~main_mask) | (std::uint64_t{word} << main_shift);
  const auto flags = (word == 0 ? flag_z : 0) | (word >= sign24 ? flag_n : 0);
  item.expected.sr = input.mode | sticky | flag_e | flag_u | flags | carry;
}

struct ControlValue {
  unsigned selector = 0;
  Word value = 0;
};

void write_control(Registers &state, const ControlValue input) {
  const auto value = input.value;
  const auto selector = input.selector;
  switch (selector) {
    case 2: state.a = (state.a & ~main_mask) | (std::uint64_t{value} << main_shift); return;
    case 3: state.b = (state.b & ~main_mask) | (std::uint64_t{value} << main_shift); return;
    case 4: state.x = (state.x & main_mask) | value; return;
    case 5: state.y = (state.y & main_mask) | value; return;
    case 6: state.x = (state.x & word_mask) | (std::uint64_t{value} << main_shift); return;
    case 7: state.y = (state.y & word_mask) | (std::uint64_t{value} << main_shift); return;
    default: throw std::logic_error("Invalid model shift control selector");
  }
}

void shift_case(Cases &cases, const ShiftCase input, const unsigned selector = 0) {
  BinaryCase item;
  item.initial = seed();
  (input.source_b ? item.initial.b : item.initial.a) = input.value;
  item.initial.sr =
      input.mode | sticky | flag_e | flag_u | flag_z | flag_n | flag_v | input.incoming_carry;
  if (selector != 0) {
    // Upper bits must be ignored by the encoded 5-/6-bit shift count.
    constexpr Word ignored_control_bits = 0xffff00;
    write_control(item.initial,
                  {.selector = selector, .value = ignored_control_bits | input.count});
  }
  item.expected = item.initial;
  if (input.logical) {
    model_logical(item, input);
  } else {
    model_arithmetic(item, input);
  }
  append(cases, std::move(item), input.opcode);
}

void arithmetic_variants(Cases &cases, const ShiftCase base) {
  constexpr Word right = 0x0c1c00;
  constexpr Word left = 0x0c1d00;
  for (const auto source : {false, true}) {
    for (const auto destination : {false, true}) {
      for (const auto prefix : {right, left}) {
        auto input = base;
        input.opcode = prefix | (source ? 128U : 0U) | (input.count * 2) | (destination ? 1U : 0U);
        input.source_b = source;
        input.destination_b = destination;
        input.left = prefix == left;
        shift_case(cases, input);
      }
    }
  }
}

void immediate_arithmetic(Cases &cases) {
  for (const auto count : arithmetic_counts) {
    for (const auto value : values) {
      for (const auto mode : modes) {
        arithmetic_variants(cases, {.value = value, .count = count, .mode = mode});
      }
    }
  }
}

void logical_shifts(Cases &cases) {
  constexpr Word immediate_right = 0x0c1ec0;
  constexpr Word immediate_left = 0x0c1e80;
  for (const auto destination : {false, true}) {
    for (const auto count : logical_counts) {
      for (const auto value : values) {
        for (const auto mode : modes) {
          for (const auto prefix : {immediate_right, immediate_left}) {
            shift_case(cases,
                       {.value = value,
                        .count = count,
                        .opcode = prefix | (count * 2) | (destination ? 1U : 0U),
                        .mode = mode,
                        .destination_b = destination,
                        .source_b = destination,
                        .left = prefix == immediate_left,
                        .logical = true});
          }
        }
      }
    }
  }
}

struct RegisterForm {
  Word prefix = 0;
  unsigned selector = 0;
};

void register_variant(Cases &cases, ShiftCase input, const RegisterForm form) {
  if (input.logical) { input.source_b = input.destination_b; }
  const auto source_bit = !input.logical && input.source_b ? 16U : 0U;
  input.opcode = form.prefix | source_bit | (form.selector * 2) | (input.destination_b ? 1U : 0U);
  shift_case(cases, input, form.selector);
}

void register_variants(Cases &cases, const RegisterForm form) {
  for (const bool source : {false, true}) {
    for (const bool destination : {false, true}) {
      for (const unsigned count : {0U, 1U, 17U, 24U}) {
        register_variant(cases,
                         {.value = values.back(),
                          .count = count,
                          .destination_b = destination,
                          .source_b = source,
                          .left = form.prefix == register_asl || form.prefix == register_lsl,
                          .logical = form.prefix < register_asl},
                         form);
      }
    }
  }
}

void register_shifts(Cases &cases) {
  constexpr std::array<Word, 4> prefixes{register_asl, register_asr, register_lsl, register_lsr};
  for (const auto prefix : prefixes) {
    for (unsigned selector = 2; selector < 8; ++selector) {
      register_variants(cases, {.prefix = prefix, .selector = selector});
    }
  }
}

void parallel_shifts(Cases &cases) {
  constexpr std::array<Word, 6> opcodes{0x22, 0x32, 0x23, 0x33, 0x27, 0x37};
  for (const auto opcode : opcodes) {
    for (const bool destination : {false, true}) {
      for (const auto value : values) {
        for (const auto carry : {Word{0}, flag_c}) {
          shift_case(cases,
                     {.value = value,
                      .opcode = parallel_none | opcode | (destination ? 8U : 0U),
                      .incoming_carry = carry,
                      .destination_b = destination,
                      .source_b = destination,
                      .left = (opcode & 16U) != 0,
                      .logical = (opcode & 1U) != 0,
                      .rotate = (opcode & 4U) != 0});
        }
      }
    }
  }
}

struct SumCase {
  std::uint64_t destination = 0, source = 0;
  Word opcode = 0, mode = 0;
  bool destination_b = false;
};

bool out_of_range(const std::int64_t value) {
  return value < minimum56 || value > maximum56;
}

void scaled_sum(Cases &cases, const SumCase input) {
  BinaryCase item;
  item.initial = seed();
  (input.destination_b ? item.initial.b : item.initial.a) = input.destination;
  (input.destination_b ? item.initial.a : item.initial.b) = input.source;
  item.initial.sr = input.mode | sticky | flag_c;
  item.expected = item.initial;
  const auto value = signed56(input.destination);
  const bool left = (input.opcode & 16U) != 0;
  const bool subtract = (input.opcode & 4U) != 0;
  const auto shifted_full = left ? value * 2 : (value / 2) - (value % 2 < 0 ? 1 : 0);
  const auto shifted = static_cast<std::uint64_t>(shifted_full) & accumulator_mask;
  const auto source = signed56(input.source);
  const auto full = signed56(shifted) + (subtract ? -source : source);
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const bool carry = subtract ? shifted < input.source : shifted + input.source > accumulator_mask;
  (input.destination_b ? item.expected.b : item.expected.a) = raw;
  item.expected.sr = input.mode | sticky |
                     arithmetic_status({.value = signed56(raw), .mode = input.mode}) |
                     (out_of_range(shifted_full) || out_of_range(full) ? flag_v | flag_l : 0) |
                     (carry ? flag_c : 0);
  append(cases, std::move(item), parallel_none | input.opcode | (input.destination_b ? 8U : 0U));
}

void scaled_sums(Cases &cases) {
  for (const auto destination : values) {
    for (const auto source : values) {
      for (const bool destination_b : {false, true}) {
        for (const Word opcode : {0x02U, 0x06U, 0x12U, 0x16U}) {
          scaled_sum(cases,
                     {.destination = destination,
                      .source = source,
                      .opcode = opcode,
                      .destination_b = destination_b});
        }
      }
    }
  }
}

void compare_magnitude(Cases &cases, const SumCase input) {
  BinaryCase item;
  item.initial = seed();
  (input.destination_b ? item.initial.b : item.initial.a) = input.destination;
  (input.destination_b ? item.initial.a : item.initial.b) = input.source;
  item.initial.sr = input.mode | sticky | flag_c;
  item.expected = item.initial;
  const auto left = signed56(input.destination);
  const auto right = signed56(input.source);
  const auto left_magnitude = static_cast<std::uint64_t>(left < 0 ? -left : left);
  const auto right_magnitude = static_cast<std::uint64_t>(right < 0 ? -right : right);
  const auto full = signed56(left_magnitude) - signed56(right_magnitude);
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  item.expected.sr =
      input.mode | sticky | arithmetic_status({.value = signed56(raw), .mode = input.mode}) |
      (out_of_range(full) ? flag_v | flag_l : 0) | (left_magnitude < right_magnitude ? flag_c : 0);
  constexpr Word compare_magnitude = 0x07;
  append(
      cases, std::move(item), parallel_none | compare_magnitude | (input.destination_b ? 8U : 0U));
}

void magnitude_comparisons(Cases &cases) {
  for (const auto destination : values) {
    for (const auto source : values) {
      for (const bool destination_b : {false, true}) {
        compare_magnitude(
            cases, {.destination = destination, .source = source, .destination_b = destination_b});
      }
    }
  }
}

struct Rejected {
  Word opcode = 0, status = 0, value = 0;
  FaultKind kind = FaultKind::invalid_instruction;
};

void rejected_case(Cases &cases, const Rejected input) {
  BinaryCase item;
  item.initial = seed();
  item.initial.sr = input.status;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {.kind = input.kind,
                .space = Space::program,
                .address = start,
                .value = input.value,
                .pc = start};
  append(cases, std::move(item), input.opcode);
}

void rejected_counts(Cases &cases) {
  constexpr Word immediate_asr = 0x0c1c00;
  constexpr Word immediate_asl = 0x0c1d00;
  constexpr Word immediate_lsl = 0x0c1e80;
  constexpr Word immediate_lsr = 0x0c1ec0;
  constexpr Word maximum_arithmetic_count = 63;
  constexpr Word maximum_logical_count = 31;
  for (const auto prefix : {immediate_asr, immediate_asl}) {
    for (Word count = accumulator_bits; count <= maximum_arithmetic_count; ++count) {
      rejected_case(cases, {.opcode = prefix | (count * 2), .value = count});
    }
  }
  for (const auto prefix : {immediate_lsr, immediate_lsl}) {
    for (Word count = word_bits + 1; count <= maximum_logical_count; ++count) {
      rejected_case(cases, {.opcode = prefix | (count * 2), .value = count});
    }
  }
  for (const auto prefix : {register_asl, register_asr, register_lsl, register_lsr}) {
    for (Word selector = 0; selector < 2; ++selector) {
      const auto opcode = prefix | (selector * 2);
      rejected_case(cases, {.opcode = opcode, .value = opcode});
    }
  }
}

void rejected_modes(Cases &cases) {
  constexpr std::array<Word, 6> unsupported_modes{Word{1} << 13,
                                                  Word{1} << 14,
                                                  Word{1} << 17,
                                                  Word{1} << 20,
                                                  mode_down | mode_up,
                                                  (Word{1} << 21) | mode_down | mode_up};
  constexpr Word immediate_asl = 0x0c1d02;
  constexpr Word x0_asl = 0x0c1e48;
  constexpr std::array<Word, 4> instructions{immediate_asl, 0x0c1e80, x0_asl, 0x0c1e38};
  for (const auto mode : unsupported_modes) {
    for (const auto opcode : instructions) {
      if (mode == saturation_model::mode && (opcode == immediate_asl || opcode == x0_asl)) {
        BinaryCase item;
        item.initial = seed();
        item.initial.sr = mode;
        item.expected = item.initial;
        const unsigned count =
            opcode == immediate_asl ? 1U : static_cast<unsigned>(item.initial.x % 64);
        shift_model::apply(
            item.expected, item.initial, {.count = count, .left = true, .saturate = false});
        append(cases, std::move(item), opcode);
        continue;
      }
      rejected_case(
          cases,
          {.opcode = opcode, .status = mode, .value = mode, .kind = FaultKind::unsupported_mode});
    }
  }
}

struct RepeatCase {
  std::uint64_t before = 0, after = 0;
  Word opcode = 0;
};

void repeat_case(Cases &cases, const RepeatCase input) {
  BinaryCase item;
  item.name = "REP retains multi-bit opcode and reevaluates the shift control";
  item.initial = seed();
  item.initial.a = input.before;
  item.initial.sr = sticky;
  item.expected = item.initial;
  item.expected.a = input.after;
  item.expected.pc = start + 2;
  item.expected.sr = sticky | arithmetic_status({.value = signed56(input.after)});
  constexpr Word repeat_three = 0x0603a0;
  item.before.push_back({.words = {repeat_three, input.opcode}, .address = start});
  item.budget = 4;
  item.dispatches = 4;
  item.progress = {.instructions = 4, .estimated_cycles = 8};
  cases.push_back(std::move(item));
}

void repeat_cases(Cases &cases) {
  constexpr Word immediate_asl_one = 0x0c1d02;
  constexpr Word register_asr_a1 = 0x0c1e64;
  constexpr std::uint64_t main_one = std::uint64_t{1} << word_bits;
  repeat_case(cases, {.before = 3, .after = std::uint64_t{3} * 8, .opcode = immediate_asl_one});
  repeat_case(cases, {.before = main_one, .after = main_one / 2, .opcode = register_asr_a1});
}

void parallel_latch(Cases &cases) {
  BinaryCase item;
  item.initial = seed();
  item.initial.sr = sticky | flag_e | flag_u;
  item.expected = item.initial;
  model_logical(item, {.left = true, .logical = true});
  const auto old_main = (item.initial.a >> word_bits) & word_mask;
  item.expected.x = (item.initial.x & main_mask) | old_main;
  constexpr Word source_a1 = 12;
  constexpr Word target_x0 = 4;
  constexpr Word logical_left = 0x33;
  append(
      cases, std::move(item), parallel_none | (source_a1 << 13) | (target_x0 << 8) | logical_left);
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_bits OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::immediate_arithmetic(cases);
    dsp56300::test::logical_shifts(cases);
    dsp56300::test::register_shifts(cases);
    dsp56300::test::parallel_shifts(cases);
    dsp56300::test::scaled_sums(cases);
    dsp56300::test::magnitude_comparisons(cases);
    dsp56300::test::rejected_counts(cases);
    dsp56300::test::rejected_modes(cases);
    dsp56300::test::repeat_cases(cases);
    dsp56300::test::parallel_latch(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent bit-operation cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
