// SPDX-License-Identifier: GPL-3.0-only
// Original normalization fixtures; linked only to the binary fixture codec.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "saturated_normalization_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::array<std::vector<BinaryCase>, 2>;
using normalization_model::Input;
using normalization_model::Kind;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 4> values{
    saturation_model::positive, saturation_model::positive + 1, sign56, accumulator_mask};
constexpr std::array exponents{0U, 1U, sign24, word_mask};
constexpr std::array counts{-55, -48, -24, -1, 0, 1, 24, 48, 55, 56};
constexpr Word repeat_three = 0x0603a0, lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word saved_la = 0x71, saved_lc = 7, pointer_range = 64;

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status | saturation_model::mode;
  state.r.fill(word_mask);
  state.n.fill(sign24);
  state.m.fill(0);
  return state;
}

void append(Cases &cases, const Registers &initial, const Input input) {
  BinaryCase item;
  item.initial = initial;
  item.expected = initial;
  normalization_model::apply(item.expected, input);
  ++item.expected.pc;
  item.before = {{.words = {normalization_model::opcode(input)}, .address = start}};
  item.progress.estimated_cycles = normalization_model::cycles(input);
  item.name = "SM normalization: old count/CCR, raw56 shifts, limiting and preserved Carry";
  cases.at(static_cast<unsigned>(input.kind)).push_back(std::move(item));
}

void fast_counts(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
      for (const bool destination_b : {false, true}) {
        for (int count = normalization_model::minimum_count;
             count <= normalization_model::maximum_count;
             ++count) {
          for (const auto value : values) {
            auto state = seed(mode | status_byte);
            (destination_b ? state.b : state.a) = value;
            register_model::write(state,
                                  normalization_model::sources[selector],
                                  static_cast<Word>(count) & word_mask);
            append(cases, state, {.selector = selector, .destination_b = destination_b});
          }
        }
      }
    }
  }
}

void iteration_flags(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned selector = 0; selector < agu_registers; ++selector) {
      for (const bool destination_b : {false, true}) {
        for (Word flags = 0; flags <= status_byte; ++flags) {
          auto state = seed(mode | flags);
          (destination_b ? state.b : state.a) = values[flags % values.size()];
          state.r[selector] = exponents[(flags / values.size()) % exponents.size()];
          append(cases,
                 state,
                 {.kind = Kind::iteration, .selector = selector, .destination_b = destination_b});
        }
      }
    }
  }
}

void fast_flags(Cases &cases) {
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      for (const bool destination_b : {false, true}) {
        auto state = seed(mode | flags);
        (destination_b ? state.b : state.a) = values[flags % values.size()];
        const auto selector = static_cast<unsigned>(flags % normalization_model::sources.size());
        register_model::write(state,
                              normalization_model::sources[selector],
                              static_cast<Word>(counts[flags % counts.size()]) & word_mask);
        append(cases, state, {.selector = selector, .destination_b = destination_b});
      }
    }
  }
}

void extension_decisions(Cases &cases) {
  constexpr std::uint64_t low = 0x123456789abc;
  constexpr unsigned extension_shift = 48;
  constexpr unsigned msp_sign = 47;
  for (const auto mode : modes) {
    for (Word extension = 0; extension <= status_byte; ++extension) {
      for (const bool msp : {false, true}) {
        auto state = seed(mode | (extension ^ status_byte));
        state.a = state.b = (std::uint64_t{extension} << extension_shift) |
                            (msp ? std::uint64_t{1} << msp_sign : 0) | low;
        for (const int count : {-1, 0, 1}) {
          register_model::write(state, 4, static_cast<Word>(count) & word_mask);
          append(cases, state, {.selector = 2, .destination_b = msp});
        }
        append(cases, state, {.kind = Kind::iteration, .destination_b = msp});
      }
    }
  }
}

void walking_bits(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
      for (const bool inverse : {false, true}) {
        for (const bool destination_b : {false, true}) {
          auto state = seed(mode | flag_c | flag_l | flag_u);
          (destination_b ? state.b : state.a) =
              (inverse ? accumulator_mask : 0) ^ (std::uint64_t{1} << bit);
          for (const int count : {-1, 0, 1, normalization_model::maximum_count}) {
            register_model::write(state, 4, static_cast<Word>(count) & word_mask);
            append(cases, state, {.selector = 2, .destination_b = destination_b});
          }
          append(cases, state, {.kind = Kind::iteration, .destination_b = destination_b});
        }
      }
    }
  }
}

BinaryCase repeated(const Registers &initial, const Input input, const Word budget) {
  BinaryCase item;
  item.initial = initial;
  item.initial.a = item.initial.b = saturation_model::positive;
  // Self-aliases read A1/B1=1 initially and zero after the first right shift.
  register_model::write(item.initial, normalization_model::sources[input.selector], 1);
  item.expected = item.initial;
  for (Word step = 1; step < budget; ++step) { normalization_model::apply(item.expected, input); }
  const bool completed = budget == 4;
  item.expected.pc = start + (completed ? 2U : 1U);
  item.expected.lc = completed ? item.initial.lc : 4 - budget;
  item.budget = item.dispatches = budget;
  item.interruptible = completed;
  item.progress = {.instructions = budget,
                   .estimated_cycles = 5 + ((budget - 1) * normalization_model::cycles(input))};
  item.before = {{.words = {repeat_three, normalization_model::opcode(input)}, .address = start}};
  item.name = "REP SM normalization re-reads current source/flags at every prefix";
  return item;
}

void repetitions(Cases &cases) {
  for (const auto mode : modes) {
    for (const auto kind : {Kind::iteration, Kind::fast}) {
      for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
        for (const bool destination_b : {false, true}) {
          for (const Word budget : {1U, 2U, 3U, 4U}) {
            cases.at(static_cast<unsigned>(kind))
                .push_back(
                    repeated(seed(mode | flag_u | flag_c | flag_s),
                             {.kind = kind, .selector = selector, .destination_b = destination_b},
                             budget));
          }
        }
      }
    }
  }
}

void loop_endings(Cases &cases) {
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    for (const Word count : {1U, 2U}) {
      for (const Word pointer : {0U, 15U, 31U, 63U}) {
        for (const Word saved : {0U, lf, fv, lf | fv}) {
          const Input input{.kind = kind, .selector = 2};
          auto state = seed(lf | flag_e | flag_c);
          state.a = sign56;
          register_model::write(state, 4, word_mask);
          state.la = start;
          state.lc = count;
          state.sp = pointer;
          state.sc = 2;
          state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
          state.stack[(pointer + pointer_range - 1) % stack_entries] =
              (std::uint64_t{saved_la} << word_bits) | saved_lc;
          append(cases, state, input);
          auto &item = cases.at(static_cast<unsigned>(kind)).back();
          item.expected.pc = count == 1 ? start + 1 : start;
          if (count == 1) {
            item.expected.la = saved_la;
            item.expected.lc = saved_lc;
            item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
            item.expected.sc = 0;
            item.expected.sr = (item.expected.sr & ~(lf | fv)) | saved;
          } else {
            --item.expected.lc;
          }
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: generate_saturated_normalization fast|iteration OUTPUT.d56");
    }
    const std::string_view kind = argv[1];
    if (kind != "fast" && kind != "iteration") { throw std::runtime_error("Unknown fixture kind"); }
    dsp56300::test::Cases cases;
    dsp56300::test::fast_counts(cases);
    dsp56300::test::iteration_flags(cases);
    dsp56300::test::fast_flags(cases);
    dsp56300::test::extension_decisions(cases);
    dsp56300::test::walking_bits(cases);
    dsp56300::test::repetitions(cases);
    dsp56300::test::loop_endings(cases);
    const auto total = cases[0].size() + cases[1].size();
    auto &selected = cases[kind == "fast" ? 1 : 0];
    const auto count = selected.size();
    dsp56300::test::write_fixture(argv[2], std::move(selected));
    std::cout << count << " independent saturated normalization " << kind
              << " cases; complete two-archive matrix=" << total << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
