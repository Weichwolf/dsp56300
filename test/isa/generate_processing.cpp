// SPDX-License-Identifier: GPL-3.0-only
// Independent processing/RESET fixtures; no emulator execution or linkage.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "processing_model.hpp"
#include "support.hpp"

#include <algorithm>
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
using Cases = std::vector<BinaryCase>;
using processing_model::Input;
using processing_model::Kind;
using processing_model::Reply;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_a = 0x200021, round_a = 0x200011;
constexpr Word repeat = 0x0600a0;
constexpr std::array kinds{Kind::reset, Kind::wait, Kind::stop};
constexpr std::array replies{Reply::absent, Reply::accept, Reply::reject};

BinaryCase base(const Input input) {
  BinaryCase item;
  item.initial = item.expected = processing_model::seed(input.status);
  item.expected.pc = start + 1;
  item.state = processing_model::state(input.kind);
  item.interrupt_ready = input.kind != Kind::stop;
  item.before = {{.words = {processing_model::opcode(input.kind), processing_model::invalid},
                  .address = start}};
  item.progress.estimated_cycles = processing_model::cycles(input.kind);
  if (input.kind == Kind::reset) {
    if (input.reply != Reply::absent) {
      item.reset_actions = {{.pc = start, .accepted = input.reply == Reply::accept}};
    }
    if (input.reply != Reply::accept) {
      item.retired = 0;
      item.progress = {};
      item.interrupt_ready = false;
      item.fault = {.kind = FaultKind::host_event,
                    .address = start,
                    .value = processing_model::reset,
                    .pc = start};
    }
  }
  item.name = "Processing instruction: full state, CCR/IPL and explicit RESET acknowledgement";
  return item;
}

void state_matrix(Cases &cases) {
  for (const auto kind : kinds) {
    for (const auto reply : replies) {
      if (kind != Kind::reset && reply != Reply::accept) { continue; }
      for (const auto mode : processing_model::modes) {
        for (Word mask = 0; mask < 4; ++mask) {
          for (Word flags = 0; flags <= status_byte; ++flags) {
            auto item = base({.kind = kind, .reply = reply, .status = mode | (mask * 256) | flags});
            // Sleeping must stop even with excess budget before the invalid successor.
            if (kind != Kind::reset) { item.budget = 4; }
            cases.push_back(std::move(item));
          }
        }
      }
    }
  }
}

void zero_budgets(Cases &cases, const Input input) {
  for (const auto mode : processing_model::modes) {
    for (Word mask = 0; mask < 4; ++mask) {
      for (const Word flags : {0U, status_byte}) {
        auto item =
            base({.kind = input.kind, .reply = input.reply, .status = mode | (mask * 256) | flags});
        item.budget = item.retired = 0;
        item.expected = item.initial;
        item.progress = {};
        item.fault = {};
        item.state = ExecutionState::running;
        item.interrupt_ready = true;
        item.reset_actions.clear();
        item.name = "Zero budget does not issue RESET or enter WAIT/STOP";
        cases.push_back(std::move(item));
      }
    }
  }
}

void raw_status_and_zero(Cases &cases) {
  for (const auto kind : kinds) {
    for (const auto reply : replies) {
      if (kind != Kind::reset && reply != Reply::accept) { continue; }
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        const Word basis = Word{1} << bit;
        for (const Word status : {basis, word_mask ^ basis}) {
          cases.push_back(base({.kind = kind, .reply = reply, .status = status}));
        }
      }
      zero_budgets(cases, {.kind = kind, .reply = reply});
    }
  }
}

BinaryCase native_prefix(const Input input, const Word budget) {
  const std::array prefix{mac_a, mac_b, processing_model::reset, adc_a, round_a};
  auto item = base(input);
  item.before.front().words.assign(prefix.begin(), prefix.end());
  item.before.front().words.push_back(processing_model::opcode(input.kind));
  item.before.front().words.push_back(processing_model::invalid);
  item.expected = item.initial;
  item.budget = budget;
  item.retired = std::min(budget, input.reply == Reply::accept ? 6U : 2U);
  item.progress = {.instructions = item.retired};
  item.state = ExecutionState::running;
  item.interrupt_ready = true;
  if (budget > 2) {
    item.reset_actions = {{.pc = start + 2, .accepted = input.reply == Reply::accept}};
    if (input.reply == Reply::reject) {
      item.fault = {.kind = FaultKind::host_event,
                    .address = start + 2,
                    .value = processing_model::reset,
                    .pc = start + 2};
      item.interrupt_ready = false;
    }
  }
  for (Word index = 0; index < item.retired; ++index) {
    const auto old = item.expected;
    if (index < prefix.size() && index != 2) {
      sequence_model::alu(item.expected, old, prefix[index]);
    }
    ++item.expected.pc;
    item.progress.estimated_cycles += index == 2 ? 7U : 1U;
    if (index == 5) {
      item.state = processing_model::state(input.kind);
      item.interrupt_ready = input.kind == Kind::wait;
      item.progress.estimated_cycles += 9;
    }
  }
  item.name = "Native/general RESET and sleep: every exact budget prefix or rejection";
  return item;
}

void native_prefixes(Cases &cases) {
  for (const auto kind : {Kind::wait, Kind::stop}) {
    for (const auto mode : processing_model::modes) {
      for (const Word flags : {0U, 1U, 127U, 128U, status_byte}) {
        for (const auto reply : {Reply::accept, Reply::reject}) {
          for (Word budget = 0; budget < 10; ++budget) {
            cases.push_back(
                native_prefix({.kind = kind, .reply = reply, .status = mode | flags}, budget));
          }
        }
      }
    }
  }
}

struct RepeatInput {
  Word encoded = 0, budget = 0;
};

BinaryCase repeated_reset(const Input input, const RepeatInput repeat_input) {
  const auto encoded = repeat_input.encoded;
  const auto budget = repeat_input.budget;
  const Word count = encoded == 0 ? 65536U : encoded;
  auto item = base(input);
  item.before.front().words = {repeat | (encoded * 256), processing_model::reset, 0, 0};
  item.expected = item.initial;
  item.reset_actions.clear();
  item.budget = item.retired = budget;
  item.progress = {};
  item.fault = {};
  item.interrupt_ready = budget == 0;
  item.name = "REP RESET: exact host requests, partial budgets, LC restore and encoded zero";
  if (budget == 0) { return item; }
  const bool accepted = input.reply == Reply::accept;
  const Word iterations = accepted ? std::min(budget - 1, count) : 0;
  item.expected.pc = start + 1;
  item.expected.lc = iterations == 0 ? encoded : count - iterations;
  item.progress = {.instructions = budget, .estimated_cycles = 5 + (std::uint64_t{iterations} * 7)};
  if (input.reply != Reply::absent && budget > 1) {
    const Word calls = accepted ? iterations : 1;
    item.reset_actions.assign(calls, {.pc = start + 1, .accepted = accepted});
  }
  if (!accepted && budget > 1) {
    item.retired = 1;
    item.progress = {.instructions = 1, .estimated_cycles = 5};
    item.fault = {.kind = FaultKind::host_event,
                  .address = start + 1,
                  .value = processing_model::reset,
                  .pc = start + 1};
  }
  if (iterations == count) {
    item.expected.pc = start + 2;
    item.expected.lc = item.initial.lc;
    item.interrupt_ready = true;
    if (budget > count + 1) {
      ++item.expected.pc;
      ++item.progress.estimated_cycles;
    }
  }
  return item;
}

void repeat_reset(Cases &cases, const Input input, const Word encoded) {
  const Word count = encoded == 0 ? 65536U : encoded;
  for (const Word budget : {0U, 1U, 2U, count, count + 1, count + 2}) {
    cases.push_back(repeated_reset(input, {.encoded = encoded, .budget = budget}));
  }
}

void repeats(Cases &cases) {
  for (const auto mode : processing_model::modes) {
    for (const Word flags : {0U, status_byte}) {
      for (const auto reply : replies) {
        const Input input{.reply = reply, .status = mode | flags};
        for (const Word count : {0U, 4U}) { repeat_reset(cases, input, count); }
      }
    }
    for (const auto kind : {Kind::wait, Kind::stop}) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        auto item = base({.kind = kind, .status = mode | flags});
        const auto opcode = processing_model::opcode(kind);
        item.before.front().words = {repeat | (4U * 256), opcode};
        item.budget = 2;
        item.retired = 1;
        item.progress = {.instructions = 1, .estimated_cycles = 5};
        item.fault = {.kind = FaultKind::invalid_instruction,
                      .address = start + 1,
                      .value = opcode,
                      .pc = start + 1};
        item.state = ExecutionState::running;
        item.interrupt_ready = false;
        item.name = "REP rejects WAIT/STOP without sleeping or retiring the body";
        cases.push_back(std::move(item));
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_processing state|sequences OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    const std::string_view group(argv[1]);
    if (group == "state") {
      dsp56300::test::state_matrix(cases);
      dsp56300::test::raw_status_and_zero(cases);
    } else if (group == "sequences") {
      dsp56300::test::native_prefixes(cases);
      dsp56300::test::repeats(cases);
    } else {
      throw std::runtime_error("Unknown processing fixture group");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(
        argv[2], std::move(cases), dsp56300::test::FixtureVersion::processing);
    std::cout << count << " independent processing " << group << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
