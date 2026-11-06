// SPDX-License-Identifier: GPL-3.0-only
// Original pipelined FIR dot-product program and independent integer expectations.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
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
constexpr Word data_base = 128, coefficient_base = 768, output_address = 16;
constexpr Word dual_postincrement_x0_y0 = 0x709800, store_a_fields = 0x120000;
constexpr Word mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | (5U << 4);
constexpr Word parallel = isa::opcode<isa::Opcode::move_xy>()[0] | dual_postincrement_x0_y0;
constexpr Word preload_clear = parallel | (isa::opcode<isa::Opcode::clr_a>()[0] & status_byte);
constexpr Word pipelined_mac = parallel | (mac & status_byte);
constexpr Word rounded = isa::opcode<isa::Opcode::rnd_a>()[0];
constexpr Word loop_prefix = isa::opcode<isa::Opcode::do_immediate>()[0];
constexpr Word store_a =
    isa::opcode<isa::Opcode::move_x_absolute_short>()[0] | store_a_fields | (output_address << 8);
constexpr std::array<Word, 12> tap_counts{4, 16, 64, 512, 5, 7, 15, 63, 129, 6, 255, 256};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<Word, 2> incoming{0, flag_c | flag_s | flag_l};

enum class Signal : std::uint8_t {
  impulse,
  alternating,
  dc,
  ramp,
  positive_turn,
  negative_turn,
  negative_dc
};
enum class Coefficient : std::uint8_t { normalized, impulse, alternating, maximum, minimum };

Word signal(const Signal kind, const std::size_t index) {
  constexpr std::size_t ramp_length = 16;
  constexpr std::int64_t half_ramp = 8;
  switch (kind) {
    case Signal::impulse: return index == 0 ? sign24 - 1 : 0;
    case Signal::alternating: return index % 2 == 0 ? sign24 - 1 : sign24;
    case Signal::dc: return sign24 - 1;
    case Signal::negative_dc: return sign24;
    case Signal::positive_turn:
    case Signal::negative_turn: {
      // The 257th full-scale product overflows inside a four-lane group;
      // opposite products restore a small, unlimited output while L stays set.
      constexpr std::size_t turning_point = 257;
      const bool positive = (index < turning_point) == (kind == Signal::positive_turn);
      return positive ? sign24 - 1 : sign24;
    }
    case Signal::ramp: {
      const auto value = (static_cast<std::int64_t>(index % ramp_length) - half_ramp) *
                         (static_cast<std::int64_t>(sign24) / half_ramp);
      return static_cast<Word>(value) & word_mask;
    }
    default: throw std::logic_error("Unknown original FIR signal");
  }
}

struct Input {
  Word taps = 4, mode = 0, flags = 0;
  Signal signal = Signal::impulse;
  Coefficient coefficient = Coefficient::normalized;
};

Word coefficient(const Input input, const std::size_t index) {
  const auto unit = sign24 / input.taps;
  switch (input.coefficient) {
    case Coefficient::normalized: return unit;
    case Coefficient::impulse: return index == 0 ? sign24 - 1 : 0;
    case Coefficient::alternating:
      return index % 2 == 0 ? unit
                            : static_cast<Word>(-static_cast<std::int64_t>(unit)) & word_mask;
    case Coefficient::maximum: return sign24 - 1;
    case Coefficient::minimum: return sign24;
    default: throw std::logic_error("Unknown original FIR coefficient");
  }
}

std::vector<Word> program(const Word taps) {
  const auto count = taps - 1;
  const auto loop = loop_prefix | ((count & status_byte) << 8) | (count >> 8);
  return {preload_clear, loop, start + 3, pipelined_mac, mac, rounded, store_a};
}

void load_words(Registers &state, const Word data, const Word weight) {
  register_model::write(state, 4, data);
  register_model::write(state, 6, weight);
  ++state.r[0];
  ++state.r[4];
}

BinaryCase fixture(const Input input) {
  BinaryCase item;
  item.name = "Original FIR dot product, one output, taps=" + std::to_string(input.taps) +
              " signal=" + std::to_string(static_cast<unsigned>(input.signal)) +
              " coefficients=" + std::to_string(static_cast<unsigned>(input.coefficient)) +
              " mode=" + std::to_string(input.mode) + " flags=" + std::to_string(input.flags);
  item.memory_words = {1024, 2048, 2048};
  item.initial = register_model::seed(input.mode);
  item.initial.sr = input.mode | input.flags;
  item.initial.r[0] = data_base;
  item.initial.r[4] = coefficient_base;
  item.initial.n[0] = item.initial.n[4] = 1;
  item.initial.m[0] = item.initial.m[4] = word_mask;
  item.expected = item.initial;
  std::vector<Word> data;
  std::vector<Word> weights;
  data.reserve(input.taps);
  weights.reserve(input.taps);
  for (Word index = 0; index < input.taps; ++index) {
    data.push_back(signal(input.signal, index));
    weights.push_back(coefficient(input, index));
  }
  const auto entry = item.expected;
  load_words(item.expected, data[0], weights[0]);
  sequence_model::alu(item.expected, entry, preload_clear);
  const auto frame = item.initial.sp % stack_entries;
  item.expected.stack[(frame + 1) % stack_entries] =
      (std::uint64_t{entry.la} << word_bits) | entry.lc;
  item.expected.stack[(frame + 2) % stack_entries] =
      (std::uint64_t{start + 3} << word_bits) | item.expected.sr;
  std::int64_t mathematical_sum = 0;
  for (Word index = 0; index < input.taps; ++index) {
    const auto old = item.expected;
    mathematical_sum += 2 * signed24(data[index]) * signed24(weights[index]);
    if (index + 1 < input.taps) { load_words(item.expected, data[index + 1], weights[index + 1]); }
    sequence_model::alu(item.expected, old, mac);
  }
  require(item.expected.a == (static_cast<std::uint64_t>(mathematical_sum) & accumulator_mask),
          "Independent FIR algebraic sum agrees with the raw-56 instruction model");
  const auto sum = item.expected;
  sequence_model::alu(item.expected, sum, rounded);
  const auto output = register_model::read(item.expected, 14);
  if (input.signal == Signal::positive_turn || input.signal == Signal::negative_turn) {
    require((item.expected.sr & flag_l) != 0 && output != sign24 && output != sign24 - 1,
            "Cancelled intermediate overflow remains observable without output clipping");
  }
  const auto words = program(input.taps);
  item.expected.pc = start + static_cast<Word>(words.size());
  item.budget = input.taps + 4;
  item.retired = item.budget;
  // DO adds four base clocks; the final absolute-short store adds one.
  item.progress = {.instructions = item.budget, .estimated_cycles = item.budget + 5};
  item.before = {{.words = words, .address = start},
                 {.words = std::move(data), .address = data_base, .space = Space::x},
                 {.words = std::move(weights), .address = coefficient_base, .space = Space::y}};
  item.after = {{.words = {output}, .address = output_address, .space = Space::x}};
  return item;
}

void add_overflow_cases(std::vector<BinaryCase> &cases) {
  constexpr dsp56300::Word cancellation_taps = 514;
  for (const auto data : {Signal::positive_turn, Signal::negative_turn}) {
    for (const auto mode : modes) {
      for (const auto flags : incoming) {
        cases.push_back(fixture({.taps = cancellation_taps,
                                 .mode = mode,
                                 .flags = flags,
                                 .signal = data,
                                 .coefficient = Coefficient::maximum}));
      }
    }
  }
  for (const auto mode : modes) {
    for (const auto flags : incoming) {
      cases.push_back(fixture({.taps = 512,
                               .mode = mode,
                               .flags = flags,
                               .signal = Signal::negative_dc,
                               .coefficient = Coefficient::minimum}));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_fir OUTPUT.d56"); }
    using namespace dsp56300::test;
    std::vector<BinaryCase> cases;
    for (const auto taps : tap_counts) {
      for (const auto data : {Signal::impulse, Signal::alternating, Signal::dc, Signal::ramp}) {
        for (const auto weights : {Coefficient::normalized,
                                   Coefficient::impulse,
                                   Coefficient::alternating,
                                   Coefficient::maximum}) {
          for (const auto mode : modes) {
            for (const auto flags : incoming) {
              cases.push_back(fixture({.taps = taps,
                                       .mode = mode,
                                       .flags = flags,
                                       .signal = data,
                                       .coefficient = weights}));
            }
          }
        }
      }
    }
    add_overflow_cases(cases);
    const auto count = cases.size();
    write_fixture(argv[1], std::move(cases));
    std::cout << count << " independently modeled original FIR cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
