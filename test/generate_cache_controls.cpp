// SPDX-License-Identifier: GPL-3.0-only
// Independent cache-control fixture authoring; linked only to fixture-format.
#include "binary_fixture.hpp"
#include "cache_isa_model.hpp"
#include "dsp56300/core.hpp"
#include "exception_model.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <algorithm>
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
using cache_isa_model::Form;
using cache_isa_model::Kind;
constexpr Word mac_a = 0x2000d2, round_a = 0x200011, adc_a = 0x200021;
constexpr Word rti = 4, repeat = 0x0600a0;

std::vector<Form> forms() {
  std::vector<Form> result;
  for (const auto kind : {Kind::flush,
                          Kind::flush_unlocked,
                          Kind::free,
                          Kind::lock_relative,
                          Kind::unlock_relative}) {
    result.push_back({.kind = kind, .extra = word_mask - status_byte});
  }
  for (const auto kind : {Kind::lock, Kind::unlock}) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (unsigned index = 0; index < 8; ++index) {
        if (mode != 6 || index == 0) {
          result.push_back({.kind = kind, .mode = mode, .index = index, .extra = word_mask});
        }
      }
    }
  }
  return result;
}

BinaryCase base(const Form form, const Registers &state) {
  BinaryCase item;
  item.initial = item.expected = state;
  cache_isa_model::apply(item.expected, form);
  item.interruptible = (state.sr & cache_isa_model::enable) != 0;
  item.progress.estimated_cycles = cache_isa_model::cycles(form);
  item.before = {{.words = {cache_isa_model::opcode(form)}, .address = start}};
  if (cache_isa_model::words(form) == 2) { item.before.front().words.push_back(form.extra); }
  item.name = "Cache control: exact flags, PC, addressing and CE-disabled illegal request";
  return item;
}

void state_matrix(Cases &cases) {
  for (const auto form : forms()) {
    for (const bool enabled : {false, true}) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        cases.push_back(base(form, cache_isa_model::seed(flags, enabled)));
      }
    }
  }
}

Word sequence_opcode(const bool enabled, const Word phase) {
  if (enabled) {
    if (phase > 2) { return 0; }
    return phase == 1 ? mac_a : round_a;
  }
  if (phase < 4) { return phase == 2 ? adc_a : round_a; }
  return phase == 5 ? mac_a : round_a;
}

void long_sequence(Cases &cases, const Form form, const Registers &input) {
  auto item = base(form, input);
  const bool enabled = (input.sr & cache_isa_model::enable) != 0;
  const Word resume = start + 2;
  const Word continuation = enabled ? start + cache_isa_model::words(form) : resume;
  item.before.push_back({.words = {mac_a, round_a, 0, 0, 0, 0}, .address = continuation});
  item.before.push_back(
      {.words = {exception_model::call | exception_model::service, 0}, .address = input.vba + 4});
  item.before.push_back({.words = {adc_a, round_a, rti}, .address = exception_model::service});
  item.expected = input;
  item.progress = {};
  for (Word phase = 0; phase < 7; ++phase) {
    Word cost = 1;
    item.interruptible = true;
    if (phase == 0) {
      cache_isa_model::apply(item.expected, form);
      cost = cache_isa_model::cycles(form);
      item.interruptible = enabled;
    } else if (!enabled && phase == 1) {
      exception_model::enter(item.expected, resume);
      cost = 3;
    } else if (!enabled && phase == 4) {
      exception_model::leave(item.expected);
      cost = 3;
      item.interruptible = false;
    } else {
      const auto opcode = sequence_opcode(enabled, phase);
      const auto old = item.expected;
      if (opcode != 0) { sequence_model::alu(item.expected, old, opcode); }
      ++item.expected.pc;
    }
    item.budget = item.dispatches = phase + 1;
    item.progress.instructions = item.budget;
    item.progress.estimated_cycles += cost;
    item.name = "Cache control: every long illegal-service/RTI or enabled foreground prefix";
    cases.push_back(item);
  }
}

void sequences(Cases &cases) {
  for (const auto form : forms()) {
    for (const bool enabled : {false, true}) {
      for (const Word flags : {0U, 1U, 16U, 128U, status_byte - 4, status_byte}) {
        long_sequence(cases, form, cache_isa_model::seed(flags, enabled));
      }
    }
  }
}

void repeated(Cases &cases, const Form form, const Registers &input, const Word encoded) {
  const Word count = encoded == 0 ? 65536U : encoded;
  const bool enabled = (input.sr & cache_isa_model::enable) != 0;
  for (const Word budget : {1U, 2U, count, count + 1, count + 2}) {
    auto item = base(form, input);
    const auto opcode = cache_isa_model::opcode(form);
    item.before.front().words = {repeat | (encoded * 256), opcode, 0, 0, 0};
    item.before.push_back(
        {.words = {exception_model::call | exception_model::service, 0}, .address = input.vba + 4});
    item.expected = input;
    item.expected.pc = start + 1;
    item.expected.lc = encoded;
    item.interruptible = false;
    item.progress = {.instructions = budget, .estimated_cycles = 5};
    const Word iterations = std::min(budget - 1, count);
    for (Word iteration = 0; iteration < iterations; ++iteration) {
      cache_isa_model::apply(item.expected, form);
      item.expected.pc = start + 1;
    }
    item.expected.lc = iterations == 0 ? encoded : count - iterations;
    if (iterations == count) {
      item.expected.pc = start + 2;
      item.expected.lc = input.lc;
      item.interruptible = enabled;
    }
    item.progress.estimated_cycles += std::uint64_t{iterations} * cache_isa_model::cycles(form);
    if (budget > count + 1) {
      if (enabled) {
        ++item.expected.pc;
        ++item.progress.estimated_cycles;
      } else {
        exception_model::enter(item.expected, start + 3);
        item.progress.estimated_cycles += 3;
      }
      item.interruptible = true;
    }
    item.budget = item.dispatches = budget;
    item.name = "REP cache control: repeated EA, LC restoration, deferred/coalesced illegal";
    cases.push_back(std::move(item));
  }
}

void repeats(Cases &cases) {
  for (const auto form : forms()) {
    for (const bool enabled : {false, true}) {
      const auto input = cache_isa_model::seed(flag_c | flag_s, enabled);
      if (cache_isa_model::words(form) == 1) {
        repeated(cases, form, input, 4);
        if (!cache_isa_model::effective(form) || (form.mode == 3 && form.index == 0)) {
          repeated(cases, form, input, 0);
        }
      } else {
        auto item = base(form, input);
        item.before.front().words.insert(item.before.front().words.begin(), repeat | (4U * 256));
        item.budget = 2;
        item.dispatches = 1;
        item.interruptible = false;
        item.progress = {.instructions = 1, .estimated_cycles = 5};
        item.fault = {.kind = FaultKind::invalid_instruction,
                      .address = start + 1,
                      .value = cache_isa_model::opcode(form),
                      .pc = start + 1};
        item.name = "REP prohibits all two-word cache-control bodies";
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
      throw std::runtime_error(
          "Usage: dsp56300_generate_cache_controls state|sequences OUTPUT.d56");
    }
    dsp56300::test::Cases cases;
    const std::string_view group(argv[1]);
    if (group == "state") {
      dsp56300::test::state_matrix(cases);
    } else if (group == "sequences") {
      dsp56300::test::sequences(cases);
      dsp56300::test::repeats(cases);
    } else {
      throw std::runtime_error("Unknown cache-control fixture group");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent cache-control " << group << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
