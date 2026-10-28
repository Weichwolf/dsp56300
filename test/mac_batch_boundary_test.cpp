// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word paired_mac = 0xf098d2, loop = Word{1} << 15;
constexpr Word port = 0xffff80, datum = 0xfedcba, count = 33;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array options{
    Options{.backend = Backend::reference},
    Options{.backend = Backend::cached},
    Options{.backend = Backend::jit, .batch_backend = BatchBackend::disabled},
    Options{.backend = Backend::jit, .batch_backend = BatchBackend::scalar},
    Options{.backend = Backend::jit, .batch_backend = BatchBackend::avx2},
    Options{.backend = Backend::jit, .batch_backend = BatchBackend::automatic},
    Options{.backend = Backend::jit, .optimize = false, .batch_backend = BatchBackend::avx2}};
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

Registers prefix(Registers state, const Word instructions) {
  for (Word index = 0; index < instructions; ++index) {
    step(state, {.x = xdata[index], .y = ydata[index]});
  }
  return state;
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  bool accept = true, invalidate = false, matched = true;
  unsigned reads = 0;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++reads;
    matched = matched && core != nullptr && space == Space::x && address == port && pc == start &&
              core->registers() == expected &&
              core->progress() == Progress{.instructions = 7, .estimated_cycles = 7};
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
  Options options;
  Word mode = 0;
  unsigned action = 0;
};

void io_boundary(const Input input) {
  const MemoryArena arena;
  Observer observer;
  observer.accept = input.action != 2;
  observer.invalidate = input.action == 1;
  Core core(arena.buffers(), &observer, input.options);
  observer.core = &core;
  core.write(Space::program, start, paired_mac);
  core.load(Space::x, port - static_cast<Word>(xdata.size()), xdata);
  core.load(Space::y, 0, ydata);
  auto state = initial(input.mode);
  state.r[0] = port - static_cast<Word>(xdata.size());
  auto expected = prefix(state, 7);
  observer.expected = expected;
  ++observer.expected.r[0];
  ++observer.expected.r[4];
  ++observer.expected.pc;
  core.set_registers(state);
  const auto result = core.run(8);
  require(
      observer.matched && observer.reads == 1,
      "The first I/O beyond a batch sees exact canonical flags, counters, addresses and registers");
  if (!observer.accept) {
    require(
        result.dispatches == 7 &&
            result.progress == Progress{.instructions = 7, .estimated_cycles = 7} &&
            result.fault.kind == FaultKind::io_read && result.fault.address == port &&
            result.fault.pc == start && core.registers().a == expected.a &&
            core.registers().lc == expected.lc,
        "A rejected first I/O retains the completed batch/scalar prefix without retiring its ALU");
  } else {
    step(expected, {.x = datum, .y = ydata.back()});
    require(result.dispatches == 8 && result.fault.kind == FaultKind::none &&
                core.registers() == expected,
            "A successful first I/O resumes its original prefetched MAC and old operands");
    if (observer.invalidate) {
      const auto next = core.run(1);
      --expected.lc;
      require(next.dispatches == 1 && next.fault.kind == FaultKind::none &&
                  core.registers() == expected,
              "I/O P mutation takes effect at the next instruction without replaying a batch");
    }
  }
  const bool batches = input.options.backend == Backend::jit && input.options.optimize &&
                       input.options.batch_backend != BatchBackend::disabled;
  require(core.jit_statistics().batch_instructions == (batches ? 4U : 0U),
          "The I/O witness includes exactly one real four-instruction compiled batch");
}

void fault_boundary(const Input input) {
  std::array<Word, memory_words> p{};
  auto x = xdata;
  auto y = xdata;
  Core core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  const auto state = initial(input.mode);
  auto expected = state;
  for (std::size_t index = 0; index < x.size(); ++index) {
    step(expected, {.x = x[index], .y = y[index]});
  }
  core.set_registers(state);
  const auto result = core.run(10);
  require(
      result.dispatches == 7 &&
          result.progress == Progress{.instructions = 7, .estimated_cycles = 7} &&
          result.fault.kind == FaultKind::memory_read && result.fault.space == Space::x &&
          result.fault.address == 7 && result.fault.pc == start,
      "A span ending after a four-step batch and three scalar steps faults at the exact next word");
  require(core.registers().a == expected.a && core.registers().b == expected.b &&
              core.registers().lc == expected.lc && core.registers().stack == expected.stack,
          "A later RAM fault preserves completed arithmetic and does not decrement or pop its "
          "failing ending");
}

void changed_frame(const Input input) {
  std::array<Word, memory_words> p{};
  std::array<Word, 64> x{};
  std::array<Word, 64> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  auto state = initial(input.mode);
  core.set_registers(state);
  require(core.run(16).dispatches == 16, "Warm a real batched loop before changing its frame");
  state.stack[2] = std::uint64_t{start + 1} << word_bits;
  auto expected = state;
  step(expected, {.x = 0, .y = 0});
  expected.pc = start + 8;
  core.set_registers(state);
  const auto before = core.jit_statistics().batch_instructions;
  const auto result = core.run(8);
  require(result.dispatches == 8 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && core.jit_statistics().batch_instructions == before,
          "A changed saved start rejects cached batching and follows the new frame after one MAC");
}

void zero_counter(const Input input) {
  std::array<Word, memory_words> p{};
  auto x = ydata;
  auto y = ydata;
  Core core({.program = p, .x = x, .y = y}, nullptr, input.options);
  core.write(Space::program, start, paired_mac);
  auto state = initial(input.mode);
  state.lc = 0;
  auto expected = state;
  for (std::size_t index = 0; index < x.size(); ++index) {
    step(expected, {.x = x[index], .y = y[index]});
  }
  core.set_registers(state);
  const auto result = core.run(static_cast<Word>(x.size()));
  require(result.dispatches == x.size() && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "A zero LC uses authoritative scalar wrap before an eligible batch resumes");
}

void program_alias(const Input input) {
  std::array<Word, memory_words> p{};
  auto y = ydata;
  Core core({.program = p, .x = p, .y = y}, nullptr, input.options);
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
  require(result.dispatches == y.size() && result.fault.kind == FaultKind::none &&
              core.registers() == expected && p == original &&
              core.jit_statistics().invalidations == 0,
          "Read-only X/P aliases retain program words and exact pipelined MAC input order");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto options : dsp56300::test::options) {
      for (const auto mode : dsp56300::test::modes) {
        for (unsigned action = 0; action < 3; ++action) {
          dsp56300::test::io_boundary({.options = options, .mode = mode, .action = action});
        }
        dsp56300::test::fault_boundary({.options = options, .mode = mode});
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
