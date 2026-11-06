// SPDX-License-Identifier: GPL-3.0-only
// Independent DEBUG/DEBUGcc fixtures; no emulator decoder/executor is linked.
#include "binary_fixture.hpp"
#include "debug_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "exception_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
using debug_model::Input;
constexpr Word round_a = 0x200011, adc_a = 0x200021;
constexpr Word repeat = 0x0600a0, rti = 4, invalid = 7;

BinaryCase base(const Input input) {
  BinaryCase item;
  item.initial = item.expected = debug_model::seed(input);
  ++item.expected.pc;
  item.debug_interrupt_mode = input.interrupt;
  const bool requested = debug_model::taken(input, input.status);
  item.debug = debug_model::status(input, requested);
  item.state = requested && !input.interrupt ? ExecutionState::debugging : ExecutionState::running;
  item.interrupt_ready = !requested;
  item.progress.estimated_cycles = debug_model::cycles(input);
  item.before = {{.words = {debug_model::opcode(input), invalid, 0}, .address = start},
                 {.words = {exception_model::call | exception_model::service, 0},
                  .address = debug_model::vector(input)},
                 {.words = {adc_a, round_a, rti}, .address = exception_model::service}};
  item.name = "DEBUG/DEBUGcc: exact CCR, mode, IME/SWO, pending request or halt";
  return item;
}

void state_matrix(Cases &cases) {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    {
      for (const auto mode : debug_model::modes) {
        for (Word flags = 0; flags <= status_byte; ++flags) {
          auto input = debug_model::input({.form = form, .flags = flags, .interrupt = false});
          input.status = mode | flags | ((flags % 4) * 256);
          auto item = base(input);
          if (item.state == ExecutionState::debugging) { item.budget = 4; }
          cases.push_back(std::move(item));
        }
      }
    }
  }
}

void raw_status(Cases &cases) {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    {
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        const Word basis = Word{1} << bit;
        for (const Word status : {basis, word_mask ^ basis}) {
          auto input = debug_model::input({.form = form, .interrupt = false});
          input.status = status;
          cases.push_back(base(input));
        }
      }
    }
  }
}

void zero_budgets(Cases &cases) {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    {
      for (const auto mode : debug_model::modes) {
        for (const Word flags : {0U, status_byte}) {
          auto input = debug_model::input({.form = form, .flags = flags, .interrupt = false});
          input.status = mode | flags;
          auto item = base(input);
          item.budget = item.retired = 0;
          item.expected = item.initial;
          item.progress = {};
          item.debug = {.interrupt_mode = false};
          item.state = ExecutionState::running;
          item.interrupt_ready = true;
          item.name = "Zero budget cannot evaluate DEBUGcc, set SWO or queue a debug vector";
          cases.push_back(std::move(item));
        }
      }
    }
  }
}

struct RepeatInput {
  Word count = 4, budget = 0;
};

BinaryCase repeat_case(const Input input, const RepeatInput repeated) {
  auto item = base(input);
  const Word count = repeated.count == 0 ? 65536U : repeated.count;
  const Word budget = repeated.budget;
  const bool requested = debug_model::taken(input, input.status);
  item.before.front().words = {repeat | (repeated.count * 256), debug_model::opcode(input), 0, 0};
  item.expected = item.initial;
  item.budget = item.retired = budget;
  item.progress = {};
  item.debug = {.interrupt_mode = input.interrupt};
  item.state = ExecutionState::running;
  item.interrupt_ready = budget == 0;
  item.name = "REP DEBUG: halt after one iteration or coalesce/defer the IME request";
  if (budget == 0) { return item; }
  Word iterations = std::min(budget - 1, count);
  if (requested && !input.interrupt) { iterations = std::min(iterations, 1U); }
  item.expected.pc = start + 1;
  item.expected.lc = iterations == 0 ? repeated.count : count - iterations;
  item.retired = 1 + iterations;
  item.progress = {.instructions = item.retired,
                   .estimated_cycles =
                       5 + (std::uint64_t{iterations} * debug_model::cycles(input))};
  if (iterations != 0 && requested && !input.interrupt) {
    item.debug.software_occurrence = true;
    item.state = ExecutionState::debugging;
  }
  if (iterations == count) {
    item.expected.pc = start + 2;
    item.expected.lc = item.initial.lc;
    item.interrupt_ready = !requested;
    if (budget > count + 1) {
      if (requested) {
        debug_model::enter(item.expected);
        item.progress.estimated_cycles += 3;
      } else {
        ++item.expected.pc;
        ++item.progress.estimated_cycles;
      }
      ++item.retired;
      ++item.progress.instructions;
      item.interrupt_ready = true;
    }
  }
  return item;
}

void repeats(Cases &cases) {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    {
      for (const Word flags : {0U, 1U, 16U, 128U, status_byte, status_byte - 4}) {
        const auto input = debug_model::input({.form = form, .flags = flags, .interrupt = false});
        for (const Word budget : {0U, 1U, 2U, 4U, 5U, 6U}) {
          cases.push_back(repeat_case(input, {.budget = budget}));
        }
      }
    }
  }
  {
    for (const Word flags : {0U, status_byte}) {
      for (const Word budget : {0U, 1U, 2U, 65536U, 65537U, 65538U}) {
        cases.push_back(repeat_case(debug_model::input({.flags = flags, .interrupt = false}),
                                    {.count = 0, .budget = budget}));
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_debug state|sequences OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    const std::string_view group(argv[1]);
    if (group == "state") {
      dsp56300::test::state_matrix(cases);
      dsp56300::test::raw_status(cases);
      dsp56300::test::zero_budgets(cases);
    } else if (group == "sequences") {
      dsp56300::test::repeats(cases);
    } else {
      throw std::runtime_error("Unknown debug fixture group");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(
        argv[2], std::move(cases), dsp56300::test::FixtureVersion::debugging);
    std::cout << count << " independent debug " << group << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
