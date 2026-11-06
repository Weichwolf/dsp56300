// SPDX-License-Identifier: GPL-3.0-only
#include "cache_isa_model.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word ce = Word{1} << 19, be = Word{1} << 10, external = 1024;
constexpr Word words = 4096, immediate_x0 = 0x44f400, movem_read_x0 = 0x07f084;
constexpr Word movem_write_x0 = 0x077084, movec_sr = 0x05f439, mac_a = 0x2000d2;
constexpr Word lock_absolute = 0x0bf081, unlock_absolute = 0x0af081;
constexpr Word first = 0x123456, second = 0x654321, third = 0xabcdef;
constexpr std::array backends{Backend::jit};

struct Step {
  Word pc = start, length = 1, cost = 1, alu = 0;
  std::optional<Word> x0 = std::nullopt;
};

struct Fixture {
  Fixture(const Backend backend, const unsigned width, const bool burst = false)
      : core({.program = program, .x = data, .y = program},
             nullptr,
             {.backend = backend,
              .cacheable_begin = external,
              .cacheable_end = words,
              .cache_sector_words = width}) {
    auto state = initial_state();
    state.sr = ce | status_byte;
    state.omr = burst ? be : 0;
    core.set_registers(state);
  }

  std::array<Word, words> program{};
  std::array<Word, 16> data{};
  TestCore core;

  void input_x0(const Word value) {
    auto state = core.registers();
    state.x = (state.x / (std::uint64_t{1} << word_bits)) * (std::uint64_t{1} << word_bits) + value;
    core.set_registers(state);
  }

  void run(const Step step) {
    auto input = core.registers();
    input.pc = step.pc;
    core.set_registers(input);
    auto expected = input;
    if (step.alu != 0) { sequence_model::alu(expected, input, step.alu); }
    expected.pc += step.length;
    if (step.x0) {
      expected.x =
          (expected.x / (std::uint64_t{1} << word_bits)) * (std::uint64_t{1} << word_bits) +
          *step.x0;
    }
    const auto before = core.progress();
    const auto result = core.run(1);
    require(result.fault.kind == FaultKind::none && result.retired == 1,
            "Cache-aware instruction retires exactly once without fault");
    require(core.registers() == expected, "Cache-aware instruction has exact architectural state");
    require(result.progress == Progress{.instructions = before.instructions + 1,
                                        .estimated_cycles = before.estimated_cycles + step.cost},
            "Cache-aware instruction adds its exact nominal base cost");
  }

  void control(const Word opcode, const std::optional<Word> extra = std::nullopt) {
    core.write(Space::program, start, opcode);
    if (extra) { core.write(Space::program, start + 1, *extra); }
    Word cost = 1;
    if (extra) { cost = opcode < 16 ? 4U : 3U; }
    run({.length = extra ? 2U : 1U, .cost = cost});
  }

  void prime() {
    const std::array code{immediate_x0, first, Word{0}, Word{0}};
    core.load(Space::program, external, code);
    run({.pc = external, .length = 2, .cost = 2, .x0 = first});
  }
};

void flush_and_lock(const Backend backend, const unsigned width) {
  Fixture fixture(backend, width);
  fixture.prime();
  fixture.core.write(Space::program, external + 1, second);
  require(fixture.core.read(Space::program, external + 1) == second,
          "Host DMA-style read sees external RAM, not cached instruction data");
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = first});
  fixture.core.clear_code_cache();
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = first});
  fixture.control(1);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = second});
  fixture.control(15, external - start);
  fixture.core.write(Space::program, external + 1, third);
  fixture.control(1);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = second});
  fixture.control(2);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = second});
  fixture.control(1);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = third});
  fixture.control(lock_absolute, external);
  fixture.core.write(Space::program, external + 1, first);
  fixture.control(3);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = first});
  fixture.control(15, external - start);
  fixture.core.write(Space::program, external + 1, second);
  fixture.control(14, external - start);
  fixture.control(1);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = second});
}

void program_transfers(const Backend backend, const unsigned width) {
  Fixture fixture(backend, width);
  fixture.prime();
  fixture.core.write(Space::program, external + 1, second);
  const std::array read{movem_read_x0, external + 1};
  fixture.core.load(Space::program, start, read);
  fixture.run({.pc = start, .length = 2, .cost = 7, .x0 = first});
  fixture.input_x0(third);
  const std::array write{movem_write_x0, external + 1};
  fixture.core.load(Space::program, start, write);
  fixture.run({.pc = start, .length = 2, .cost = 7});
  require(fixture.core.read(Space::program, external + 1) == third,
          "Guest PMOVE writes through to external RAM");
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = third});
  fixture.core.write(Space::y, external + 1, first);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = third});
  fixture.control(3);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = first});
  fixture.control(unlock_absolute, external);
}

void burst_fetch(const Backend backend, const unsigned width, const bool burst) {
  Fixture fixture(backend, width, burst);
  fixture.prime();
  const std::array replacement{immediate_x0, second};
  fixture.core.load(Space::program, external + 2, replacement);
  fixture.run({.pc = external + 2,
               .length = burst ? 1U : 2U,
               .cost = burst ? 1U : 2U,
               .x0 = burst ? std::nullopt : std::optional<Word>{second}});
  require(fixture.core.run(0).retired == 0, "Zero budgets never fetch or flush cache words");
}

void mode_and_reset(const Backend backend, const unsigned width) {
  Fixture fixture(backend, width);
  fixture.prime();
  fixture.core.write(Space::program, external + 1, second);
  const std::array disable{movec_sr, Word{0}};
  fixture.core.load(Space::program, start, disable);
  auto input = fixture.core.registers();
  input.pc = start;
  fixture.core.set_registers(input);
  auto expected = input;
  expected.pc = start + 2;
  expected.sr = 0;
  const auto result = fixture.core.run(1);
  require(result.fault.kind == FaultKind::none && fixture.core.registers() == expected,
          "Guest CE clear preserves unrelated state and disables cache");
  expected.sr = ce | flag_c;
  fixture.core.set_registers(expected);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = second});
  fixture.core.write(Space::program, external + 1, third);
  fixture.core.reset();
  expected = initial_state();
  expected.sr = ce;
  fixture.core.set_registers(expected);
  fixture.run({.pc = external, .length = 2, .cost = 2, .x0 = third});
}

void native_mode_boundary(const unsigned width) {
  Fixture fixture(Backend::jit, width);
  fixture.core.reset();
  const std::array code{mac_a, movec_sr, ce, mac_a, mac_a, movec_sr, Word{0}, mac_a};
  fixture.core.load(Space::program, start, code);
  auto input = initial_state();
  input.x = input.y = 1;
  fixture.core.set_registers(input);
  auto expected = input;
  sequence_model::alu(expected, input, mac_a);
  expected.sr = ce;
  for (unsigned count = 0; count < 2; ++count) {
    const auto old = expected;
    sequence_model::alu(expected, old, mac_a);
  }
  expected.sr = 0;
  const auto old = expected;
  sequence_model::alu(expected, old, mac_a);
  expected.pc = start + code.size();
  const auto result = fixture.core.run(6);
  require(result.retired == 6 && result.fault.kind == FaultKind::none &&
              fixture.core.registers() == expected &&
              result.progress == Progress{.instructions = 6, .estimated_cycles = 8},
          "Native/general/native CE transitions preserve complete state and budgets");
}

void effective_sector(const Backend backend,
                      const unsigned width,
                      const cache_isa_model::Form form) {
  Fixture fixture(backend, width);
  const Word cursor = external + (2 * width);
  const Word target = form.mode == 7 ? cursor - 1 : cursor + (form.mode == 5 ? width : 0U);
  const Word address = form.mode == 6 ? external : target;
  fixture.run({.pc = address});
  fixture.core.write(Space::program, address, mac_a);
  auto input = fixture.core.registers();
  input.pc = start;
  input.r[form.index] = cursor;
  input.n[form.index] = width;
  input.x = input.y = 1;
  fixture.core.set_registers(input);
  auto expected = input;
  cache_isa_model::apply(expected, form);
  const std::array code{cache_isa_model::opcode(form), external};
  fixture.core.load(Space::program, start, code);
  const auto result = fixture.core.run(1);
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              fixture.core.registers() == expected,
          "Every effective cache lock selects its independently modeled EA and update");
  fixture.control(1);
  fixture.run({.pc = address});
  auto unlock = form;
  unlock.kind = cache_isa_model::Kind::unlock;
  input = fixture.core.registers();
  input.pc = start;
  input.r[form.index] = cursor;
  fixture.core.set_registers(input);
  const std::array unlock_code{cache_isa_model::opcode(unlock), external};
  fixture.core.load(Space::program, start, unlock_code);
  expected = input;
  cache_isa_model::apply(expected, unlock);
  const auto unlocked = fixture.core.run(1);
  require(unlocked.retired == 1 && unlocked.fault.kind == FaultKind::none &&
              fixture.core.registers() == expected,
          "Every effective unlock selects the same sector and exact AGU result");
  fixture.control(1);
  fixture.run({.pc = address, .alu = mac_a});
}

void effective_sectors(const Backend backend, const unsigned width) {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < 8; ++index) {
      if (mode != 6 || index == 0) {
        effective_sector(
            backend, width, {.kind = cache_isa_model::Kind::lock, .mode = mode, .index = index});
      }
    }
  }
}

void invalid_effective(const Backend backend, const unsigned width) {
  for (const Word prefix : {lock_absolute, unlock_absolute}) {
    for (Word index = 1; index < 8; ++index) {
      Fixture fixture(backend, width);
      const Word opcode = prefix + (index * 256);
      const std::array code{opcode, external};
      fixture.core.load(Space::program, start, code);
      const auto input = fixture.core.registers();
      auto expected = input;
      expected.pc += 2;
      const auto result = fixture.core.run(1);
      require(result.retired == 0 &&
                  result.fault == Fault{.kind = FaultKind::invalid_instruction,
                                        .address = start,
                                        .value = opcode,
                                        .pc = start} &&
                  fixture.core.registers() == expected && fixture.core.progress() == Progress{},
              "Invalid cache EA selector cannot update state, retire or access a sector");
      require(fixture.core.run(1).retired == 0, "Failed cache control remains stopped");
    }
  }
}

void invalid_configuration() {
  std::array<Word, 16> memory{};
  for (const TestOptions options : {TestOptions{.cacheable_begin = 1, .cacheable_end = 0},
                                    TestOptions{.cacheable_end = word_mask + 2},
                                    TestOptions{.cache_sector_words = 64}}) {
    bool rejected = false;
    try {
      const TestCore core({.program = memory, .x = memory, .y = memory}, nullptr, options);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Invalid cache geometry/address intervals are explicitly rejected");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::invalid_configuration();
    for (const unsigned width : {128U, 256U}) {
      for (const auto backend : dsp56300::test::backends) {
        dsp56300::test::effective_sectors(backend, width);
        dsp56300::test::invalid_effective(backend, width);
        dsp56300::test::flush_and_lock(backend, width);
        dsp56300::test::program_transfers(backend, width);
        dsp56300::test::burst_fetch(backend, width, false);
        dsp56300::test::burst_fetch(backend, width, true);
        dsp56300::test::mode_and_reset(backend, width);
      }
      dsp56300::test::native_mode_boundary(width);
    }
    std::cout << dsp56300::test::checks << " architectural cache integration checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
