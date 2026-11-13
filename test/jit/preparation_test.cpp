// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word entry = test::start, setup = entry - 2;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
constexpr Word loop_flag = 1U << 15, forever_flag = 1U << 16;
constexpr Word pointer_mask = 63, previous_la = 0x123, previous_lc = 0x456;
constexpr Word inherited_flags = 0x7f, all_flags = 0xff;
constexpr Word x_host_bits = 0xab000000, y_host_bits = 0xcd000000;
constexpr Word long_body_pairs = 130, io_begin = 0xffff80;
constexpr unsigned x_register_shift = 18;
constexpr std::size_t extent = 1024;
using Data = std::array<Word, extent>;
using Program = std::array<Word, test::memory_words>;

struct Routing {
  bool reversed_load = false, b_to_x = true, b_to_y = false;
};

struct LoopInput {
  Word count = 0, flags = 0, sp = 2;
};

struct Fixture {
  explicit Fixture(const Routing route, const bool aliases, const Word pairs = 1)
      : aliased(aliases), routing(route), body_words(pairs * 2) {
    constexpr Word fields = (3U << x_register_shift) | (3U << 16);
    for (Word index = 0; index < pairs; ++index) {
      program[entry + (index * 2)] = (load & ~fields) |
                                     ((route.reversed_load ? 3U : 2U) << x_register_shift) |
                                     ((route.reversed_load ? 2U : 3U) << 16);
      program[entry + (index * 2) + 1] = (store & ~fields) |
                                         ((route.b_to_x ? 3U : 2U) << x_register_shift) |
                                         ((route.b_to_y ? 3U : 2U) << 16);
    }
    program[setup] = static_cast<Word>(isa::Opcode::do_immediate) | (3U << 8);
    program[setup + 1] = entry + body_words - 1;
    program[entry + body_words] = isa::rts.front();
  }

  [[nodiscard]] Registers state(const LoopInput input) const {
    const auto [count, flags, sp] = input;
    auto result = test::register_model::seed(0);
    result.sr = loop_flag | flags;
    result.la = entry + body_words - 1;
    result.lc = count;
    result.sp = sp;
    result.sc = 2;
    result.r[0] = 4;
    result.r[4] = aliased ? 5U : 4U;
    result.stack[sp % stack_entries] = std::uint64_t{entry} << test::word_bits;
    result.stack[((sp + pointer_mask) & pointer_mask) % stack_entries] =
        (std::uint64_t{previous_la} << test::word_bits) | previous_lc;
    return result;
  }

  void reset_data() {
    constexpr std::array<Word, 8> values{
        0, 1, test::sign24, word_mask, test::sign24 - 1, 0x600000, 0x200000, 0xabcdef};
    for (std::size_t index = 0; index < extent; ++index) {
      x[index] = values[index % values.size()] | x_host_bits;
      y[index] = values[(index + 3) % values.size()] | y_host_bits;
    }
  }

  void step(Registers &current, Data &expected_x, Data &expected_y) const {
    auto &right = aliased ? expected_x : expected_y;
    if ((current.pc - entry) % 2 == 0) {
      const auto from_x = expected_x[current.r[0]] & word_mask;
      const auto from_y = right[current.r[4]] & word_mask;
      test::register_model::write(current, routing.reversed_load ? 15U : 14U, from_x);
      test::register_model::write(current, routing.reversed_load ? 14U : 15U, from_y);
    } else {
      const auto to_x =
          test::register_model::bus_word(current, routing.b_to_x ? current.b : current.a);
      const auto to_y =
          test::register_model::bus_word(current, routing.b_to_y ? current.b : current.a);
      expected_x[current.r[0]++] = to_x;
      right[current.r[4]++] = to_y;
    }
    ++current.pc;
    if (current.pc - 1 != current.la) { return; }
    if ((current.sr & forever_flag) != 0 || current.lc != 1) {
      current.lc = (current.lc - 1) & word_mask;
      current.pc = entry;
    } else {
      test::loop_frames::complete(current);
    }
  }

  Program program{};
  Data x{}, y{};
  bool aliased;
  Routing routing;
  Word body_words;
};

void prefixes(const Routing route, const bool aliased, const Word pairs = 1) {
  Fixture fixture(route, aliased, pairs);
  Core core(Memory::separate({.program = fixture.program,
                              .x = fixture.x,
                              .y = aliased ? std::span(fixture.x) : std::span(fixture.y)}));
  const auto source = fixture.program;
  constexpr std::array<Word, 1> entries{setup};
  const auto before = core.state();
  const auto report = core.prepare(entries);
  const auto repeated = core.prepare(entries);
  const auto after = core.state();
  require(report.prepared_entries == 1 && report.native_bytes > 0 && report.failure == Fault{} &&
              repeated.prepared_entries == 1 && repeated.native_bytes == 0 &&
              before.registers == after.registers && before.progress == after.progress &&
              before.execution == after.execution && before.failure == after.failure,
          "Preparation discovers a DO body, installs native code once, and never executes ISA");
  for (const Word flags : {0U, inherited_flags, all_flags, forever_flag}) {
    for (const Word count : {0U, 1U, 3U, 5U}) {
      const auto total = (count == 0 || flags == forever_flag ? 5U : count) * fixture.body_words;
      for (Word prefix = 0; prefix <= total; ++prefix) {
        fixture.reset_data();
        auto expected_x = fixture.x;
        auto expected_y = fixture.y;
        auto expected = fixture.state({.count = count, .flags = flags, .sp = prefix % 64});
        core.set_registers(expected);
        for (Word index = 0; index < prefix; ++index) {
          fixture.step(expected, expected_x, expected_y);
        }
        const auto progress = core.progress();
        const auto result =
            prefix % 2 == 0 ? core.run(prefix) : core.run_until(progress.estimated_cycles + prefix);
        require(result.retired == prefix && result.fault == Fault{} && !result.yielded &&
                    result.progress ==
                        Progress{.instructions = progress.instructions + prefix,
                                 .estimated_cycles = progress.estimated_cycles + prefix} &&
                    core.registers() == expected && fixture.x == expected_x &&
                    fixture.y == expected_y && fixture.program == source,
                "Installed loops preserve every prefix, full state, flags, frames and RAM aliases");
      }
    }
  }
}

struct External final : Io {
  External()
      : windows{MemoryWindow{.words = fixture.x, .known = {}, .writable = true},
                MemoryWindow{.words = fixture.y, .known = {}, .writable = true}} {}

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space,
                                                             Word /*mode*/) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(windows).subspan(space == Space::x ? 0U : 1U, 1);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (change_revision) {
      ++revision;
      change_revision = false;
    }
    if (reentrant_core != nullptr) {
      constexpr std::array<Word, 1> entries{setup};
      reentrant_failure = reentrant_core->prepare(entries).failure;
    }
    return address < fixture.program.size() ? std::optional(fixture.program[address])
                                            : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    ++accesses;
    if (space != Space::program || address >= fixture.program.size()) { return false; }
    value = fixture.program[address];
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    ++accesses;
    if (space != Space::program || address >= fixture.program.size()) { return false; }
    fixture.program[address] = value;
    ++program_writes;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++accesses;
    return false;
  }

  Fixture fixture{{}, false};
  std::array<MemoryWindow, 2> windows;
  mutable std::uint64_t revision = 0;
  mutable bool change_revision = false;
  Core *reentrant_core = nullptr;
  mutable Fault reentrant_failure;
  unsigned accesses = 0;
  unsigned program_writes = 0;
};

void external_bindings() {
  External memory;
  Core core(Memory::external(), &memory);
  memory.reentrant_core = &core;
  constexpr std::array<Word, 1> entries{setup};
  require(core.prepare(entries).prepared_entries == 1 && memory.accesses == 0 &&
              memory.reentrant_failure.kind == FaultKind::host_event &&
              core.state().failure == Fault{},
          "External initialized grants prepare without guest I/O callbacks");
  memory.reentrant_core = nullptr;
  auto state = memory.fixture.state({.count = 3});
  memory.fixture.reset_data();
  auto expected_x = memory.fixture.x;
  auto expected_y = memory.fixture.y;
  core.set_registers(state);
  for (unsigned index = 0; index < 6; ++index) {
    memory.fixture.step(state, expected_x, expected_y);
  }
  require(core.run(6).retired == 6 && core.registers() == state && memory.fixture.x == expected_x &&
              memory.fixture.y == expected_y,
          "External prepared entries execute with the actual captured RAM grants");
  ++memory.revision;
  memory.windows[0].writable = false;
  const auto before = core.state();
  const auto refused = core.prepare(entries);
  require(refused.prepared_entries == 0 && refused.unresolved_entries > 0 &&
              core.state().registers == before.registers && core.progress() == before.progress &&
              core.state().failure == before.failure,
          "A changed read-only grant cannot reuse stale writable optimized code");
  ++memory.revision;
  memory.windows[0].address = io_begin;
  require(core.prepare(entries).failure.kind == FaultKind::host_event &&
              core.state().failure == Fault{},
          "Invalid external descriptors return a non-latching preparation failure");
}

void changing_revision() {
  External memory;
  Core core(Memory::external(), &memory);
  constexpr std::array<Word, 1> entries{setup};
  memory.change_revision = true;
  const auto before = core.state();
  const auto refused = core.prepare(entries);
  require(refused.prepared_entries == 0 && refused.failure.kind == FaultKind::host_event &&
              core.registers() == before.registers && core.progress() == before.progress &&
              core.state().failure == before.failure,
          "A revision changed during capture cannot install code or latch an ISA fault");
  require(core.prepare(entries).prepared_entries == 1,
          "The next stable binding can prepare after a non-latching revision refusal");
}

void independent_entries() {
  Fixture fixture({}, false);
  constexpr Word invalid = 0x0d3000; // Unaudited encoding, deliberately not an ISA operation.
  fixture.program[0] = invalid;
  Core core(Memory::separate({.program = fixture.program, .x = fixture.x, .y = fixture.y}));
  constexpr std::array<Word, 2> entries{0, setup};
  const auto before = core.state();
  const auto report = core.prepare(entries);
  require(report.prepared_entries == 1 && report.native_bytes > 0 &&
              report.failure.kind == FaultKind::unsupported_instruction && report.failure.pc == 0 &&
              report.failure.value == invalid && core.registers() == before.registers &&
              core.progress() == before.progress && core.state().failure == Fault{},
          "An invalid seed retains its diagnostic without losing an independent valid loop");
}

void native_setup_join() {
  Fixture fixture({}, false);
  fixture.reset_data();
  Core core(Memory::separate({.program = fixture.program, .x = fixture.x, .y = fixture.y}));
  constexpr std::array<Word, 1> entries{setup};
  require(core.prepare(entries).prepared_entries == 1, "Prepare a body before its DO executes");
  auto initial = fixture.state({.count = previous_lc});
  initial.pc = setup;
  initial.la = previous_la;
  initial.sr = inherited_flags;
  initial.sp = initial.sc = 0;
  core.set_registers(initial);
  auto expected = initial;
  expected.pc = entry;
  expected.la = entry + 1;
  expected.lc = 3;
  expected.sr |= loop_flag;
  expected.sp = expected.sc = 2;
  expected.stack[1] = (std::uint64_t{previous_la} << test::word_bits) | previous_lc;
  expected.stack[2] = (std::uint64_t{entry} << test::word_bits) | initial.sr;
  auto x = fixture.x;
  auto y = fixture.y;
  for (unsigned index = 0; index < 6; ++index) { fixture.step(expected, x, y); }
  const auto result = core.run(7);
  constexpr std::uint64_t setup_cycles = 5;
  constexpr std::uint64_t body_cycles = 6;
  require(result.retired == 7 && result.fault == Fault{} && core.registers() == expected &&
              result.progress ==
                  Progress{.instructions = 7, .estimated_cycles = setup_cycles + body_cycles} &&
              fixture.x == x && fixture.y == y,
          "Actual native DO setup joins the prepared body and restores its real outer frame");
}

void program_write_reinstallation() {
  External memory;
  constexpr Word writer = entry + 8;
  constexpr Word absolute = 0x30;
  constexpr Word x0 = 4;
  constexpr Word register_change = (1U << 18) | (1U << 16);
  const auto changed_load = memory.fixture.program[entry] ^ register_change;
  memory.fixture.program[writer] =
      static_cast<Word>(isa::Opcode::movem_effective) | (absolute << 8) | x0;
  memory.fixture.program[writer + 1] = entry;
  memory.fixture.program[writer + 2] = isa::rts.front();
  Core core(Memory::external(), &memory);
  auto writing = test::register_model::seed(0);
  writing.pc = writer;
  writing.x = changed_load;
  core.set_registers(writing);
  constexpr std::array<Word, 1> entries{setup};
  require(core.prepare(entries).prepared_entries == 1,
          "Prepare the original RAM loop before a genuine P write");
  const auto store_result = core.run(1);
  require(
      store_result.retired == 1 && store_result.fault == Fault{} && memory.program_writes == 1 &&
          memory.fixture.program[entry] == changed_load && memory.revision == 0,
      "Native MOVEM changes observed program bytes without a host revision or preparation call");
  memory.fixture.routing.reversed_load = true;
  const auto verify = [&](const Word grant, const bool cycles) {
    memory.fixture.reset_data();
    auto expected_x = memory.fixture.x;
    auto expected_y = memory.fixture.y;
    auto expected = memory.fixture.state({.count = 5});
    core.set_registers(expected);
    for (Word index = 0; index < grant; ++index) {
      memory.fixture.step(expected, expected_x, expected_y);
    }
    const auto before = core.progress();
    const auto result = cycles ? core.run_until(before.estimated_cycles + grant) : core.run(grant);
    require(result.retired == grant && result.fault == Fault{} && core.registers() == expected &&
                memory.fixture.x == expected_x && memory.fixture.y == expected_y &&
                core.progress() == Progress{.instructions = before.instructions + grant,
                                            .estimated_cycles = before.estimated_cycles + grant},
            "Post-write execution follows new ISA bytes with exact prefixes, full state and memory "
            "effects");
  };
  verify(2, false);
  const auto before_preparation = core.state();
  const auto restored = core.prepare(entries);
  require(restored.prepared_entries == 1 && restored.native_bytes > 0 &&
              core.registers() == before_preparation.registers &&
              core.progress() == before_preparation.progress,
          "After a real P write, fast native execution leaves optimizing materialization to "
          "explicit preparation");
  const auto repeated = core.prepare(entries);
  require(repeated.prepared_entries == 1 && repeated.native_bytes == 0,
          "The explicitly reinstalled image is retained after the genuine code change");
  for (Word prefix = 0; prefix <= 10; ++prefix) {
    verify(prefix, false);
    verify(prefix, true);
  }
}
}

int main() try {
  for (const bool reversed : {false, true}) {
    for (const bool b_to_x : {false, true}) {
      for (const bool b_to_y : {false, true}) {
        for (const bool alias : {false, true}) {
          prefixes({.reversed_load = reversed, .b_to_x = b_to_x, .b_to_y = b_to_y}, alias);
        }
      }
    }
  }
  // Natural regions exceed the removed 64/256-instruction translation limits.
  prefixes({}, false, long_body_pairs);
  external_bindings();
  independent_entries();
  changing_revision();
  native_setup_join();
  program_write_reinstallation();
  std::cout << test::checks << " independent preparation checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
