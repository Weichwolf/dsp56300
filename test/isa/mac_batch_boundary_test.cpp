// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "memory_arena.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word paired_mac = 0xf098d2, loop = Word{1} << 15;
constexpr Word port = 0xffff80, datum = 0xfedcba, count = 129;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array options{TestOptions{.backend = Backend::jit}};

constexpr std::array<Word, 7> xdata{1, sign24, sign24 - 1, 0, 3, word_mask, 7};
constexpr std::array<Word, 8> ydata{word_mask, 1, sign24, 9, sign24 - 1, 0, 11, datum};

Registers initial(const Word mode) {
  auto state = register_model::seed(mode | loop | flag_c | flag_s);
  state.la = start;
  state.lc = count;
  state.r[0] = 0;
  state.r[4] = 0;
  state.m[0] = state.m[4] = word_mask;
  state.a = sign56 - 1;
  state.stack[2] = std::uint64_t{start} << word_bits;
  return state;
}

struct Pair {
  Word x, y;
};

void step(Registers &state, const Pair data) {
  const auto old = state;
  register_model::write(state, 4, data.x);
  register_model::write(state, 6, data.y);
  ++state.r[0];
  ++state.r[4];
  sequence_model::alu(state, old, paired_mac);
  state.lc = state.lc == 0 ? word_mask : state.lc - 1;
}

Registers prefix(Registers state, const std::span<const Word> x, const std::span<const Word> y) {
  for (std::size_t index = 0; index < x.size(); ++index) {
    step(state, {.x = x[index], .y = y[index]});
  }
  return state;
}

class Observer final : public Io {
public:
  TestCore *core = nullptr;
  Registers expected;
  bool accept = true, invalidate = false, matched = true;
  unsigned reads = 0;
  Word completed = 7;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++reads;
    matched =
        matched && core != nullptr && space == Space::x && address == port && pc == start &&
        core->registers() == expected &&
        core->progress() == Progress{.instructions = completed, .estimated_cycles = completed};
    if (matched && accept && invalidate) { core->write(Space::program, start, 0); }
    value = datum;
    return matched && accept;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

struct Input {
  TestOptions options;
  Word mode = 0;
  unsigned action = 0;
  Word completed = 7;
};

void io_boundary(const Input input) {
  std::vector<Word> x(input.completed);
  std::vector<Word> y(input.completed + 1);
  for (std::size_t index = 0; index < x.size(); ++index) { x[index] = xdata[index % xdata.size()]; }
  for (std::size_t index = 0; index < y.size(); ++index) { y[index] = ydata[index % ydata.size()]; }
  const MemoryArena arena;
  Observer observer;
  observer.accept = input.action != 2;
  observer.invalidate = input.action == 1;
  TestCore core(arena.buffers(), &observer, input.options);
  observer.core = &core;
  observer.completed = input.completed;
  core.write(Space::program, start, paired_mac);
  core.load(Space::x, port - input.completed, x);
  core.load(Space::y, 0, y);
  auto state = initial(input.mode);
  state.r[0] = port - input.completed;
  if (input.completed > xdata.size()) { state.a = 0; }
  auto expected = prefix(state, x, y);
  observer.expected = expected;
  ++observer.expected.r[0];
  ++observer.expected.r[4];
  ++observer.expected.pc;
  core.set_registers(state);
  const auto result = core.run(input.completed + 1);
  require(
      observer.matched && observer.reads == 1,
      "The first I/O beyond a batch sees exact canonical flags, counters, addresses and registers");
  if (!observer.accept) {
    require(
        result.retired == input.completed &&
            result.progress ==
                Progress{.instructions = input.completed, .estimated_cycles = input.completed} &&
            result.fault.kind == FaultKind::io_read && result.fault.address == port &&
            result.fault.pc == start && core.registers().a == expected.a &&
            core.registers().lc == expected.lc,
        "A rejected first I/O retains the completed batch/scalar prefix without retiring its ALU");
  } else {
    step(expected, {.x = datum, .y = y.back()});
    require(result.retired == input.completed + 1 && result.fault.kind == FaultKind::none &&
                core.registers() == expected,
            "A successful first I/O resumes its original prefetched MAC and old operands");
    if (observer.invalidate) {
      const auto next = core.run(1);
      --expected.lc;
      require(next.retired == 1 && next.fault.kind == FaultKind::none &&
                  core.registers() == expected,
              "I/O P mutation takes effect at the next instruction without replaying a batch");
    }
  }
}

void fault_boundary(const Input input) {
  std::array<Word, memory_words> p{};
  std::vector<Word> x(input.completed);
  for (std::size_t index = 0; index < x.size(); ++index) { x[index] = xdata[index % xdata.size()]; }
  auto y = x;
  TestCore core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  auto state = initial(input.mode);
  if (input.completed > xdata.size()) { state.a = 0; }
  auto expected = state;
  for (std::size_t index = 0; index < x.size(); ++index) {
    step(expected, {.x = x[index], .y = y[index]});
  }
  core.set_registers(state);
  const auto result = core.run(input.completed + 3);
  require(
      result.retired == input.completed &&
          result.progress ==
              Progress{.instructions = input.completed, .estimated_cycles = input.completed} &&
          result.fault.kind == FaultKind::memory_read && result.fault.space == Space::x &&
          result.fault.address == input.completed && result.fault.pc == start,
      "A span ending after a compiled prefix and three scalar steps faults at the exact next word");
  require(core.registers().a == expected.a && core.registers().b == expected.b &&
              core.registers().lc == expected.lc && core.registers().stack == expected.stack,
          "A later RAM fault preserves completed arithmetic and does not decrement or pop its "
          "failing ending");
}

void changed_frame(const Input input) {
  std::array<Word, memory_words> p{};
  std::array<Word, 64> x{};
  std::array<Word, 64> y{};
  TestCore core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  auto state = initial(input.mode);
  core.set_registers(state);
  require(core.run(16).retired == 16, "Warm a real batched loop before changing its frame");
  state.stack[2] = std::uint64_t{start + 1} << word_bits;
  auto expected = state;
  step(expected, {.x = 0, .y = 0});
  expected.pc = start + 8;
  core.set_registers(state);

  const auto result = core.run(8);
  require(result.retired == 8 && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "A changed saved start rejects cached batching and follows the new frame after one MAC");
}

void zero_counter(const Input input) {
  std::array<Word, memory_words> p{};
  auto x = ydata;
  auto y = ydata;
  TestCore core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  auto state = initial(input.mode);
  state.lc = 0;
  auto expected = state;
  for (std::size_t index = 0; index < x.size(); ++index) {
    step(expected, {.x = x[index], .y = y[index]});
  }
  core.set_registers(state);
  const auto result = core.run(static_cast<Word>(x.size()));
  require(result.retired == x.size() && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "A zero LC uses authoritative scalar wrap before an eligible batch resumes");
}

void program_alias(const Input input) {
  std::array<Word, memory_words> p{};
  auto y = ydata;
  TestCore core({.program = p, .x = p, .y = y}, nullptr, input.options);
  constexpr Word base = start - 2;
  core.load(Space::x, base, ydata);
  core.write(Space::program, start, paired_mac);
  const auto original = p;
  auto state = initial(input.mode);
  state.r[0] = base;
  auto expected = state;
  for (std::size_t index = 0; index < y.size(); ++index) {
    step(expected, {.x = p[base + index], .y = y[index]});
  }
  core.set_registers(state);
  const auto result = core.run(static_cast<Word>(y.size()));
  require(result.retired == y.size() && result.fault.kind == FaultKind::none &&
              core.registers() == expected && p == original,
          "Read-only X/P aliases retain program words and exact pipelined MAC input order");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto options : dsp56300::test::options) {
      for (const auto mode : dsp56300::test::modes) {
        constexpr dsp56300::Word medium_tail = 19;
        constexpr dsp56300::Word wide_tail = 67;
        for (const dsp56300::Word completed : {7U, medium_tail, wide_tail}) {
          for (unsigned action = 0; action < 3; ++action) {
            dsp56300::test::io_boundary(
                {.options = options, .mode = mode, .action = action, .completed = completed});
          }
          dsp56300::test::fault_boundary(
              {.options = options, .mode = mode, .completed = completed});
        }
        dsp56300::test::changed_frame({.options = options, .mode = mode});
        dsp56300::test::zero_counter({.options = options, .mode = mode});
        dsp56300::test::program_alias({.options = options, .mode = mode});
      }
    }
    std::cout << dsp56300::test::checks << " MAC batch boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
