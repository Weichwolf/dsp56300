// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsp56300::test::checkpoint_test {
namespace {
constexpr Word service_mask = 0x300, long_return_cycles = 17;

struct Route {
  Backend origin;
  Backend destination;
};

struct Expected {
  Registers state;
  Progress progress;
};

void check(Core &core, const Expected &expected) {
  require(core.registers() == expected.state && core.progress() == expected.progress,
          "Full registers and retirement/base-cost counters have independent expectations");
}

std::vector<Expected> vector_trace(const bool long_entry) {
  auto state = seed();
  state.pc = start + 1;
  std::vector<Expected> trace{
      {.state = state, .progress = {.instructions = 1, .estimated_cycles = 9}}};
  add(state);
  trace.push_back({.state = state, .progress = {.instructions = 2, .estimated_cycles = 10}});
  if (long_entry) {
    state.sp = state.sc = 1;
    state.stack[1] = (std::uint64_t{start + 1} << word_bits) | state.sr;
    state.sr |= service_mask;
    state.pc = service;
    trace.push_back({.state = state, .progress = {.instructions = 3, .estimated_cycles = 13}});
    ++state.pc;
    trace.push_back({.state = state, .progress = {.instructions = 4, .estimated_cycles = 14}});
    state.pc = start + 1;
    state.sr = flag_u;
    state.sp = state.sc = 0;
    trace.push_back(
        {.state = state, .progress = {.instructions = 5, .estimated_cycles = long_return_cycles}});
  } else {
    add(state);
    trace.push_back({.state = state, .progress = {.instructions = 3, .estimated_cycles = 11}});
  }
  ++state.pc;
  const auto before = trace.back().progress;
  trace.push_back({.state = state,
                   .progress = {.instructions = before.instructions + 1,
                                .estimated_cycles = before.estimated_cycles + 1}});
  return trace;
}

void repeat_case(const Route route, const unsigned completed) {
  Machine source(route.origin);
  source.core.load(Space::program, start, std::array{repeat_four, add_x0, Word{0}});
  source.core.set_registers(seed());
  step(source.core);
  auto expected = seed();
  expected.pc = start + 1;
  expected.lc = 4;
  for (unsigned index = 0; index < completed; ++index) {
    step(source.core);
    add(expected);
    expected.lc = index == 3 ? 7U : 3 - index;
    if (index == 3) { ++expected.pc; }
  }
  check(source.core,
        {.state = expected,
         .progress = {.instructions = 1 + completed, .estimated_cycles = 5 + completed}});
  // A captured REP opcode must survive a later backing-program mutation.
  if (completed != 0 && completed != 4) { source.core.write(Space::program, start + 1, 0); }
  const auto saved = source.core.checkpoint();
  Machine target(route.destination);
  target.memory = source.memory;
  const auto memory = target.memory;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    target.memory = memory;
    target.core.restore(saved);
    auto result = expected;
    for (unsigned index = completed; index < 4; ++index) {
      require(!target.core.interruptible(), "Checkpoint retains live REP exclusion");
      step(target.core);
      add(result);
      result.lc = index == 3 ? 7U : 3 - index;
      if (index == 3) { ++result.pc; }
      check(target.core,
            {.state = result,
             .progress = {.instructions = 2 + index, .estimated_cycles = 6 + index}});
    }
    step(target.core);
    ++result.pc;
    check(target.core, {.state = result, .progress = {.instructions = 6, .estimated_cycles = 10}});
  }
}

void vector_case(const Route route, const bool long_entry, const std::size_t point) {
  Machine source(route.origin);
  source.core.set_registers(seed());
  source.core.load(Space::program, start, std::array{trap, Word{0}, Word{0}});
  source.core.load(
      Space::program, 8, std::array{add_x0, long_entry ? short_jsr | service : add_x0});
  source.core.load(Space::program, service, std::array{Word{0}, rti});
  const auto trace = vector_trace(long_entry);
  for (std::size_t index = 0; index <= point; ++index) {
    step(source.core);
    check(source.core, trace[index]);
  }
  const auto saved = source.core.checkpoint();
  Machine target(route.destination);
  target.memory = source.memory;
  source.core.reset();
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    target.core.restore(saved);
    check(target.core, trace[point]);
    for (std::size_t index = point + 1; index < trace.size(); ++index) {
      step(target.core);
      check(target.core, trace[index]);
    }
    require(target.core.interruptible(), "One actual foreground NOP consumes service exclusion");
  }
}
}

void repetition() {
  for (const auto origin : backends) {
    for (const auto destination : backends) {
      for (unsigned completed = 0; completed <= 4; ++completed) {
        repeat_case({.origin = origin, .destination = destination}, completed);
      }
    }
  }
}

void vectors() {
  for (const auto origin : backends) {
    for (const auto destination : backends) {
      for (const bool long_entry : {false, true}) {
        const auto count = vector_trace(long_entry).size();
        for (std::size_t point = 0; point < count; ++point) {
          vector_case({.origin = origin, .destination = destination}, long_entry, point);
        }
      }
    }
  }
}
} // namespace dsp56300::test::checkpoint_test
