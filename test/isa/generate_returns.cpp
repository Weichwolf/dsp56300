// SPDX-License-Identifier: GPL-3.0-only
// Independent RTI frame/sequence expectations; no emulator implementation is linked.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "return_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
using return_model::foreground;
using return_model::Frame;
using return_model::fv;
using return_model::lf;
using return_model::rti;
using return_model::rts;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, round_a = 0x200011;
constexpr Word second_service = 0x140, saved_la = 0x70, pointer_range = 64;
constexpr std::uint64_t half_seed = 0x1234567ffffe;

BinaryCase single(const Frame frame) {
  BinaryCase item;
  item.initial = return_model::seed(frame);
  item.expected = item.initial;
  return_model::apply(item.expected);
  item.before = {{.words = {rti}, .address = start}};
  item.progress.estimated_cycles = 3;
  item.interruptible = false;
  item.name =
      "RTI restores raw PC/SR and decrements SP/SC; pointer=" + std::to_string(frame.pointer) +
      "; status=" + std::to_string(frame.status);
  return item;
}

void ccr_matrix(Cases &cases) {
  for (Word pointer = 1; pointer < stack_entries; ++pointer) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      const auto status = modes.at(flags % modes.size()) | flags | ((pointer % 4) << 8);
      cases.push_back(single({.status = status, .pointer = pointer}));
    }
  }
}

void occupancy(Cases &cases) {
  constexpr Word status = 0xb6;
  for (Word pointer = 1; pointer < stack_entries; ++pointer) {
    for (Word count = 1; count < stack_entries; ++count) {
      for (const auto mode : modes) {
        cases.push_back(single({.status = mode | status, .pointer = pointer, .count = count}));
      }
    }
  }
}

void basis(Cases &cases) {
  for (Word pointer = 1; pointer < stack_entries; ++pointer) {
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      for (const bool inverted : {false, true}) {
        const auto raw = (Word{1} << bit) ^ (inverted ? word_mask : 0U);
        cases.push_back(single({.status = raw, .pointer = pointer}));
        cases.push_back(single({.pc = raw, .status = mode_rm | flag_c, .pointer = pointer}));
      }
    }
    for (const Word raw : {0U, word_mask}) {
      cases.push_back(single({.status = raw, .pointer = pointer}));
      cases.push_back(single({.pc = raw, .status = mode_rm, .pointer = pointer}));
    }
  }
}

void invalid(Cases &cases) {
  constexpr Word repeat_two = 0x0602a0;
  for (Word pointer = 0; pointer < pointer_range; ++pointer) {
    for (const Word status : {0U, word_mask, mode_rm, mode_up | status_byte}) {
      auto item = single({.status = status, .pointer = pointer});
      item.initial.sc = 0;
      item.expected = item.initial;
      ++item.expected.pc;
      item.dispatches = 0;
      item.progress = {};
      item.fault = {.kind = FaultKind::stack, .address = start, .value = pointer, .pc = start};
      item.name = "RTI empty-stack failure is explicit, nonretiring and fail-stop";
      cases.push_back(std::move(item));
    }
    for (const Word operation : {rti, rts}) {
      auto item = single({.pointer = pointer});
      item.before.front().words = {repeat_two, operation};
      item.expected = item.initial;
      item.expected.pc = start + 1;
      item.expected.lc = 2;
      item.budget = 2;
      item.progress = {.instructions = 1, .estimated_cycles = 5};
      item.fault = {.kind = FaultKind::invalid_instruction,
                    .address = start + 1,
                    .value = operation,
                    .pc = start + 1};
      item.name = "REP cannot repeat either return instruction";
      cases.push_back(std::move(item));
    }
  }
}

void native_sequence(Cases &cases, const Frame frame) {
  auto item = single(frame);
  item.initial.sr = 0;
  item.initial.x = 1;
  item.initial.y = 1;
  item.initial.a = half_seed;
  item.before = {{.words = {mac_a, mac_b, rti}, .address = start},
                 {.words = {adc_x_a, round_a}, .address = foreground}};
  item.expected = item.initial;
  item.progress = {};
  item.budget = 0;
  const std::array operations{mac_a, mac_b, rti, adc_x_a, round_a};
  for (const auto operation : operations) {
    if (operation == rti) {
      return_model::apply(item.expected);
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, operation);
      ++item.expected.pc;
    }
    item.dispatches = ++item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += operation == rti ? 3U : 1U;
    item.interruptible = operation != rti;
    item.name = "Native/RTI/native: every prefix restores Carry, scaling and RM before consumers";
    cases.push_back(item);
  }
}

void native_sequences(Cases &cases) {
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      native_sequence(cases,
                      {.status = mode | flags,
                       .pointer = static_cast<Word>((flags % (stack_entries - 1)) + 1)});
    }
  }
}

struct Nested {
  Word outer = 0, inner = 0, pointer = 0;
};

void nested_trace(Cases &cases, const Nested input) {
  auto item = single({.pc = second_service,
                      .status = input.inner | status_byte,
                      .pointer = input.pointer,
                      .count = 2});
  const auto previous = (input.pointer + pointer_range - 1) % stack_entries;
  item.initial.stack[previous] = (std::uint64_t{foreground} << word_bits) | input.outer | flag_c;
  item.before = {{.words = {rti}, .address = start},
                 {.words = {rti}, .address = second_service},
                 {.words = {0, adc_x_a}, .address = foreground}};
  item.expected = item.initial;
  item.progress = {};
  item.budget = 0;
  for (unsigned phase = 0; phase < 4; ++phase) {
    if (phase < 2) {
      return_model::apply(item.expected);
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, phase == 2 ? 0U : adc_x_a);
      ++item.expected.pc;
    }
    item.dispatches = ++item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += phase < 2 ? 3U : 1U;
    item.interruptible = phase >= 2;
    item.name = "Nested RTI restores each PC/SR frame and only then admits another interrupt";
    cases.push_back(item);
  }
}

void nested(Cases &cases) {
  for (const auto outer : modes) {
    for (const auto inner : modes) {
      for (const Word pointer : {2U, 7U, 14U, 15U}) {
        nested_trace(cases, {.outer = outer, .inner = inner, .pointer = pointer});
      }
    }
  }
}

struct Loop {
  Word pointer = 0, count = 1, saved = 0, mode = 0;
  bool at_last = false;
};

void interrupted_loop(Cases &cases, const Loop input) {
  auto item = single({.pc = foreground + (input.at_last ? 1U : 0U),
                      .status = input.mode | lf | flag_c,
                      .pointer = input.pointer,
                      .count = 3});
  item.initial.sr = 0;
  item.initial.la = foreground + 1;
  item.initial.lc = input.count;
  item.initial.x = 1;
  item.initial.y = 1;
  const auto loop_frame = (input.pointer + pointer_range - 1) % stack_entries;
  const auto outer_frame = (input.pointer + pointer_range - 2) % stack_entries;
  item.initial.stack[loop_frame] = (std::uint64_t{foreground} << word_bits) | input.saved;
  item.initial.stack[outer_frame] = (std::uint64_t{saved_la} << word_bits) | 7U;
  item.before = {{.words = {rti}, .address = start},
                 {.words = {mac_a, mac_a, 0, 0, 0, 0}, .address = foreground}};
  item.expected = item.initial;
  item.progress = {};
  item.budget = 0;
  for (unsigned phase = 0; phase < 7; ++phase) {
    if (phase == 0) {
      return_model::apply(item.expected);
    } else {
      const auto pc = item.expected.pc;
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, pc <= foreground + 1 ? mac_a : 0U);
      ++item.expected.pc;
      if (pc == foreground + 1) {
        if (item.expected.lc > 1) {
          --item.expected.lc;
          item.expected.pc = foreground;
        } else {
          item.expected.sp = (input.pointer + pointer_range - 3) % pointer_range;
          item.expected.sc = 0;
          item.expected.la = saved_la;
          item.expected.lc = 7;
          item.expected.sr = (item.expected.sr & ~(lf | fv)) | input.saved;
        }
      }
    }
    item.dispatches = ++item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += phase == 0 ? 3U : 1U;
    item.interruptible = phase != 0;
    item.name = "RTI returns to interrupted DO at LA or LA-1; exact iteration/cleanup prefixes";
    cases.push_back(item);
  }
}

void loops(Cases &cases) {
  for (const auto mode : modes) {
    for (const Word pointer : {3U, 7U, 14U, 15U}) {
      for (const Word count : {1U, 2U}) {
        for (const Word saved : {0U, lf, fv, lf | fv}) {
          for (const bool at_last : {false, true}) {
            interrupted_loop(cases,
                             {.pointer = pointer,
                              .count = count,
                              .saved = saved,
                              .mode = mode,
                              .at_last = at_last});
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
      throw std::runtime_error("Usage: dsp56300_generate_returns state|sequences OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    const std::string_view group(argv[1]);
    if (group == "state") {
      dsp56300::test::ccr_matrix(cases);
      dsp56300::test::occupancy(cases);
      dsp56300::test::basis(cases);
      dsp56300::test::invalid(cases);
    } else if (group == "sequences") {
      dsp56300::test::native_sequences(cases);
      dsp56300::test::nested(cases);
      dsp56300::test::loops(cases);
    } else {
      throw std::runtime_error("Unknown return fixture group");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent RTI " << group << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
