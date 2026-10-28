// SPDX-License-Identifier: GPL-3.0-only
// Independent software-exception input/expectation authoring; no core execution/link.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "exception_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
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
using exception_model::Input;
using exception_model::Kind;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Word adc_x_a = 0x200021, round_a = 0x200011, mac_a = 0x2000d2, mac_b = 0x2000da;
constexpr unsigned form_count = 18;
constexpr Word rti = 4, repeat_base = 0x0600a0, bank_size = 512;

struct CaseSeed {
  unsigned form = 0;
  Word flags = 0;
};

Input input_for(const CaseSeed source) {
  const auto form = source.form;
  const auto flags = source.flags;
  Input input;
  if (form == 0) {
    input.kind = Kind::illegal;
  } else if (form >= 2) {
    input.kind = Kind::conditional;
    input.condition = form - 2;
  }
  input.status = flags | modes.at(flags % modes.size()) | ((flags % 4) << 8);
  input.vba = (flags % 2) * bank_size;
  input.pointer = static_cast<Word>(flags % (stack_entries - 1));
  return input;
}

BinaryCase base(const Input input) {
  BinaryCase item;
  item.initial = exception_model::seed(input);
  item.expected = item.initial;
  ++item.expected.pc;
  item.before = {{.words = {exception_model::opcode(input), 0, 0, 0, 0, 0}, .address = start},
                 {.words = {exception_model::call | exception_model::service, 0},
                  .address = exception_model::vector(input)},
                 {.words = {adc_x_a, round_a, rti}, .address = exception_model::service}};
  item.progress.estimated_cycles = exception_model::cycles(input);
  item.interruptible = !exception_model::requested(input, input.status);
  item.name =
      "Software exception request; opcode=" + std::to_string(exception_model::opcode(input)) +
      "; status=" + std::to_string(input.status);
  return item;
}

void state_matrix(Cases &cases) {
  for (unsigned form = 0; form < form_count; ++form) {
    for (const auto mode : modes) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        auto input = input_for({.form = form, .flags = flags});
        input.status = mode | flags | ((flags % 4) << 8);
        cases.push_back(base(input));
      }
    }
  }
}

void long_sequence(Cases &cases, const Input input) {
  auto item = base(input);
  item.expected = item.initial;
  item.progress = {};
  item.budget = 0;
  const bool requested = exception_model::requested(input, input.status);
  for (unsigned phase = 0; phase < 6; ++phase) {
    Word cost = 1;
    item.interruptible = true;
    if (phase == 0) {
      ++item.expected.pc;
      cost = exception_model::cycles(input);
      item.interruptible = !requested;
    } else if (!requested) {
      ++item.expected.pc;
    } else if (phase == 1) {
      exception_model::enter(item.expected, exception_model::resume(input, start));
      cost = 3;
    } else if (phase == 4) {
      exception_model::leave(item.expected);
      cost = 3;
      item.interruptible = false;
    } else {
      if (phase < 4) {
        const auto old = item.expected;
        sequence_model::alu(item.expected, old, phase == 2 ? adc_x_a : round_a);
      }
      ++item.expected.pc;
    }
    item.dispatches = ++item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += cost;
    item.name = "Software exception long service and RTI: every budget prefix and condition";
    cases.push_back(item);
  }
}

void long_sequences(Cases &cases) {
  for (unsigned form = 0; form < form_count; ++form) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      long_sequence(cases, input_for({.form = form, .flags = flags}));
    }
  }
}

void fast_sequence(Cases &cases, const Input input, const bool immediate) {
  auto item = base(input);
  item.before[1].words =
      immediate ? std::vector<Word>{exception_model::load_immediate_x0, exception_model::datum}
                : std::vector<Word>{mac_a, mac_b};
  item.expected = item.initial;
  item.progress = {};
  item.budget = 0;
  const bool requested = exception_model::requested(input, input.status);
  const unsigned length = immediate ? 3U : 4U;
  for (unsigned phase = 0; phase < length; ++phase) {
    Word cost = 1;
    item.interruptible = false;
    if (phase == 0) {
      ++item.expected.pc;
      cost = exception_model::cycles(input);
      item.interruptible = !requested;
    } else if (requested && phase == 1) {
      item.expected.pc = exception_model::resume(input, start);
      if (immediate) {
        register_model::write(item.expected, 4, exception_model::datum);
        cost = 2;
      } else {
        const auto old = item.expected;
        sequence_model::alu(item.expected, old, mac_a);
      }
    } else if (requested && !immediate && phase == 2) {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, mac_b);
    } else {
      ++item.expected.pc;
      item.interruptible = true;
    }
    item.dispatches = ++item.budget;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += cost;
    item.name =
        "Software exception fast service: two words, one/two instructions and exact budgets";
    cases.push_back(item);
  }
}

void fast_sequences(Cases &cases) {
  for (unsigned form = 0; form < form_count; ++form) {
    for (const auto mode : modes) {
      for (const Word flags : {0U, 1U, 128U, 255U}) {
        auto input = input_for({.form = form, .flags = flags});
        input.status = mode | flags | exception_model::priority_bits;
        fast_sequence(cases, input, false);
        fast_sequence(cases, input, true);
      }
    }
  }
}

void repeated_illegal(Cases &cases, const Input input, const Word encoded) {
  const Word count = encoded == 0 ? 65536U : encoded;
  for (const Word budget : {1U, 2U, count, count + 1, count + 2, count + 5}) {
    auto item = base(input);
    item.before.front().words = {
        repeat_base | (encoded << 8), 5, exception_model::invalid_canary, 0};
    item.expected = item.initial;
    const Word iterations = std::min(budget - 1, count);
    const bool completed = iterations == count;
    item.expected.pc = start + (completed ? 2U : 1U);
    item.expected.lc = completed ? item.initial.lc : count - iterations;
    if (budget == 1) { item.expected.lc = encoded; }
    item.progress = {.instructions = budget,
                     .estimated_cycles = 5 + (std::uint64_t{iterations} * 5)};
    item.interruptible = false;
    for (Word extra = count + 1; extra < budget; ++extra) {
      const Word phase = extra - count;
      if (phase == 1) {
        exception_model::enter(item.expected, start + 3);
        item.progress.estimated_cycles += 3;
        item.interruptible = true;
      } else if (phase == 4) {
        exception_model::leave(item.expected);
        item.progress.estimated_cycles += 3;
        item.interruptible = false;
      } else {
        const auto old = item.expected;
        sequence_model::alu(item.expected, old, phase == 2 ? adc_x_a : round_a);
        ++item.expected.pc;
        ++item.progress.estimated_cycles;
      }
    }
    item.budget = item.dispatches = budget;
    item.name = "REP ILLEGAL coalesces one exception, defers vector entry and returns past the "
                "skipped word";
    cases.push_back(std::move(item));
  }
}

void repeats(Cases &cases) {
  for (const auto mode : modes) {
    auto input = input_for({.flags = flag_c | flag_s});
    input.status |= mode;
    for (const Word encoded : {0U, 4U}) { repeated_illegal(cases, input, encoded); }
  }
  for (unsigned form = 1; form < form_count; ++form) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      const auto input = input_for({.form = form, .flags = flags});
      auto item = base(input);
      const auto opcode = exception_model::opcode(input);
      item.before.front().words = {repeat_base | (4U << 8), opcode};
      item.expected = item.initial;
      item.expected.pc = start + 1;
      item.expected.lc = 4;
      item.budget = 2;
      item.progress = {.instructions = 1, .estimated_cycles = 5};
      item.interruptible = false;
      item.fault = {.kind = FaultKind::invalid_instruction,
                    .address = start + 1,
                    .value = opcode,
                    .pc = start + 1};
      item.name = "REP prohibits TRAP and TRAPcc even when the condition is false";
      cases.push_back(std::move(item));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_exceptions state|sequences OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    const std::string_view group(argv[1]);
    if (group == "state") {
      dsp56300::test::state_matrix(cases);
      dsp56300::test::repeats(cases);
    } else if (group == "sequences") {
      dsp56300::test::long_sequences(cases);
      dsp56300::test::fast_sequences(cases);
    } else {
      throw std::runtime_error("Unknown software exception fixture group");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent software exception " << group << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
