// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using dsp56300::Word;
constexpr std::size_t memory_words = 1024;
constexpr Word start = 0x100;
constexpr std::uint32_t long_grant = 100;
constexpr unsigned word_bits = 24;
constexpr Word parallel_none = static_cast<Word>(dsp56300::isa::Opcode::move_none);
constexpr Word mac_operation =
    (static_cast<Word>(dsp56300::isa::Opcode::mac_register_a) & 0xff) | (5U << 4);
constexpr Word mac_x0_y0_a = parallel_none | mac_operation;
constexpr Word test_word = 0x123456;
constexpr std::uint64_t accumulator_initial = 0x00100000000000;
constexpr std::uint64_t accumulator_mac = 0x001296cd9619c8;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

constexpr Word move_a_x1 = parallel_none | (14U << 13) | (5U << 8);
constexpr Word repeat_four = static_cast<Word>(dsp56300::isa::Opcode::rep_immediate) | (4U << 8);
constexpr Word repeat_two = static_cast<Word>(dsp56300::isa::Opcode::rep_immediate) | (2U << 8);
constexpr Word jump = static_cast<Word>(dsp56300::isa::Opcode::jmp_absolute_short) | start;
constexpr std::array<std::array<Word, 5>, 7> programs{{{mac_x0_y0_a},
                                                       {move_a_x1 | mac_operation},
                                                       {repeat_four, mac_x0_y0_a, 0},
                                                       {repeat_four, mac_x0_y0_a},
                                                       {repeat_four, 0, 0, 0},
                                                       {repeat_four, 0, repeat_two, 0, 0},
                                                       {repeat_two, jump}}};
constexpr std::array<std::size_t, programs.size()> lengths{1, 1, 3, 2, 4, 5, 2};

dsp56300::NativeImage prepare() {
  std::vector<dsp56300::CodeSpan> spans;
  spans.reserve(programs.size());
  for (std::size_t index = 0; index < programs.size(); ++index) {
    spans.push_back({.address = start, .words = std::span(programs[index]).first(lengths[index])});
  }
  constexpr std::array entries{start, start, start, start, start + 2, start + 2, start};
  constexpr std::array transitions{
      dsp56300::CodeTransition{.from = 3, .to = 4, .entry = start + 2}};
  return dsp56300::test::compile_programs(spans, entries, transitions);
}

struct Fixture {
  Fixture(const dsp56300::NativeImage &image, const std::size_t variant, dsp56300::Io *io = nullptr)
      : core(dsp56300::Memory::separate({.program = p, .x = x, .y = y}), io) {
    core.load(
        dsp56300::Space::program, start, std::span(programs[variant]).first(lengths[variant]));
    core.install(image, variant);
    auto state = core.registers();
    state.pc = start;
    core.set_registers(state);
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  dsp56300::Core core;
};

void check_mac(const dsp56300::NativeImage &image) {
  Fixture fixture(image, 0);
  auto state = fixture.core.registers();
  state.x = test_word;
  state.y = test_word;
  state.a = accumulator_initial;
  fixture.core.set_registers(state);
  const auto result = fixture.core.run(1);
  require(result.fault.kind == dsp56300::FaultKind::none, "MAC executes");
  require(fixture.core.registers().a == accumulator_mac, "Manual fractional MAC example");
}

void check_parallel_latch(const dsp56300::NativeImage &image) {
  Fixture fixture(image, 1);
  // Appendix A: register move eeeee=A=14, ddddd=X1=5; MAC low byte 0xd2.
  auto state = fixture.core.registers();
  state.x = test_word;
  state.y = test_word;
  state.a = accumulator_initial;
  fixture.core.set_registers(state);
  const auto result = fixture.core.run(1);
  require(result.fault.kind == dsp56300::FaultKind::none, "Parallel MAC executes");
  const auto actual = fixture.core.registers();
  require(actual.a == accumulator_mac && actual.x >> word_bits == 0x100000,
          "Parallel move reads old A and ALU reads old X/Y");
}

void check_loop(const dsp56300::NativeImage &image) {
  Fixture fixture(image, 2);
  auto state = fixture.core.registers();
  state.x = 1;
  state.y = 1;
  state.lc = 7;
  fixture.core.set_registers(state);
  require(fixture.core.run(5).fault.kind == dsp56300::FaultKind::none, "REP executes");
  const auto actual = fixture.core.registers();
  require(actual.a == 8 && actual.lc == 7 && actual.pc == start + 2, "REP count and LC restore");
  require(fixture.core.progress().estimated_cycles == 9,
          "REP base cost five plus four single-cycle MAC instructions");
}

void check_unsupported_source() {
  constexpr std::array<Word, 1> reserved{0x0d3000};
  const std::array spans{dsp56300::CodeSpan{.address = start, .words = reserved}};
  constexpr std::array entries{start};
  const std::array variants{
      dsp56300::CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  bool refused = false;
  try {
    static_cast<void>(dsp56300::Core::lower(
        {.variants = variants},
        {.memory = {}, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}}));
  } catch (const std::invalid_argument &error) {
    const std::string_view reason(error.what());
    require(reason.contains("P:$000100") && reason.contains("unsupported instruction encoding"),
            "Unsupported source refusal identifies the actual address and encoding failure");
    refused = true;
  }
  require(refused, "An unsupported source cannot become a partially prepared executable image");
}

void check_repeat_latch(const dsp56300::NativeImage &image, const std::uint32_t allowance) {
  Fixture fixture(image, 3);
  auto state = fixture.core.registers();
  state.x = 1;
  state.y = 1;
  fixture.core.set_registers(state);
  require(fixture.core.run(2).retired == 2, "REP and first iteration retire");
  require(!fixture.core.interrupt_ready(), "Interrupts wait for REP completion");
  fixture.core.load(dsp56300::Space::program, start + 1, std::array<Word, 1>{0});
  const auto captured = fixture.core.state();
  bool refused = false;
  try {
    fixture.core.install(image, 4);
  } catch (const std::invalid_argument &error) {
    refused = std::string_view(error.what()).contains("latched REP instruction");
  }
  require(refused && fixture.core.registers() == captured.registers &&
              fixture.core.progress() == captured.progress &&
              fixture.core.state().failure == captured.failure,
          "An early resident-image replacement cannot replace a different captured REP body");
  const auto continued = fixture.core.run(allowance);
  const auto expected = std::min(allowance, std::uint32_t{3});
  require(continued.retired == expected && continued.fault == dsp56300::Fault{} &&
              fixture.core.registers().a == std::uint64_t{2} * (1 + expected) &&
              fixture.p[start + 1] == 0 &&
              fixture.core.progress() == dsp56300::Progress{.instructions = 2 + expected,
                                                            .estimated_cycles = 6 + expected},
          "REP executes its latched opcode after a host program-memory change: retired=" +
              std::to_string(continued.retired) +
              " fault=" + std::to_string(static_cast<unsigned>(continued.fault.kind)) +
              " pc=" + std::to_string(continued.fault.pc));
  if (expected < 3) {
    require(!continued.yielded && !fixture.core.interrupt_ready(),
            "A partial retained repeat keeps its actual interrupt exclusion");
    const auto finished = fixture.core.run(long_grant);
    require(finished.retired == 3 - expected && finished.fault == dsp56300::Fault{} &&
                finished.yielded,
            "A large grant ends exactly at the prepared code-change handoff: retired=" +
                std::to_string(finished.retired) +
                " fault=" + std::to_string(static_cast<unsigned>(finished.fault.kind)));
  } else {
    require(continued.yielded, "Completion reaches the declared code-version handoff");
  }
  require(fixture.core.registers().a == 8 && fixture.core.registers().lc == state.lc &&
              fixture.core.progress() ==
                  dsp56300::Progress{.instructions = 5, .estimated_cycles = 9},
          "Retained MACs and the original saved LC/cycles finish without consuming fresh code");
  fixture.core.install(image, 4);
  require(fixture.core.interrupt_ready(), "Interrupts are admitted before a non-REP successor");
  fixture.core.load(dsp56300::Space::program, start + 2, std::array{repeat_two});
  fixture.core.install(image, 5);
  require(!fixture.core.interrupt_ready(),
          "Adjacent REP instructions form one interrupt exclusion");
  require(fixture.core.run(3).retired == 3 && fixture.core.interrupt_ready(),
          "Interrupt exclusion ends after the final adjacent REP body");
}

struct RevisionBus final : dsp56300::Io {
  std::uint64_t revision = 0;

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(dsp56300::Space /*space*/,
            Word /*address*/,
            Word /*pc*/,
            Word & /*value*/) noexcept override {
    return false;
  }

  bool write(dsp56300::Space /*space*/,
             Word /*address*/,
             Word /*pc*/,
             Word /*value*/) noexcept override {
    return false;
  }

  bool bit(dsp56300::Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           dsp56300::BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

void check_repeat_rebinding(const dsp56300::NativeImage &image, const bool changed_revision) {
  RevisionBus io;
  Fixture fixture(image, 2, &io);
  auto state = fixture.core.registers();
  state.x = state.y = 1;
  fixture.core.set_registers(state);
  require(fixture.core.run(2).retired == 2, "The repeat captures its original MAC first");
  fixture.core.load(dsp56300::Space::program, start + 1, std::array<Word, 1>{0});
  if (changed_revision) { ++io.revision; }
  const auto result = fixture.core.run(long_grant);
  const auto expected_count = changed_revision ? 0U : 3U;
  const auto expected_pc = start + (changed_revision ? 1U : 2U);
  require(result.retired == expected_count &&
              result.fault == dsp56300::Fault{.kind = dsp56300::FaultKind::unprepared_execution,
                                              .space = dsp56300::Space::program,
                                              .address = expected_pc,
                                              .pc = expected_pc} &&
              fixture.core.registers().a == (changed_revision ? 2U : 8U) &&
              fixture.core.registers().pc == expected_pc && fixture.p[start + 1] == 0,
          "A changed owner revision blocks the latch; a missing handoff blocks only fresh code");
  const auto progress = fixture.core.progress();
  require(fixture.core.run(long_grant).retired == 0 && fixture.core.progress() == progress,
          "Binding failures remain fail-stop without replaying the retained arithmetic");
}

void check_uncaptured_body(const dsp56300::NativeImage &image) {
  Fixture fixture(image, 3);
  auto state = fixture.core.registers();
  state.x = state.y = 1;
  fixture.core.set_registers(state);
  require(fixture.core.run(1).retired == 1, "Only the REP header has executed");
  fixture.core.load(dsp56300::Space::program, start + 1, std::array<Word, 1>{0});
  const auto failed = fixture.core.run(long_grant);
  require(failed.retired == 0 &&
              failed.fault == dsp56300::Fault{.kind = dsp56300::FaultKind::unprepared_execution,
                                              .space = dsp56300::Space::program,
                                              .address = start + 1,
                                              .pc = start + 1} &&
              fixture.core.registers().a == 0 &&
              fixture.core.progress() ==
                  dsp56300::Progress{.instructions = 1, .estimated_cycles = 5},
          "An uncaptured changed body requires its new prepared source before its first fetch");
}

void check_repeat_restriction(const dsp56300::NativeImage &image) {
  Fixture fixture(image, 6);
  const auto result = fixture.core.run(2);
  require(result.retired == 1 && result.fault.kind == dsp56300::FaultKind::invalid_instruction &&
              result.fault.pc == start + 1,
          "REP rejects a control-flow instruction instead of repeating it");
}
}

int main() {
  try {
    const auto image = prepare();
    check_mac(image);
    check_parallel_latch(image);
    check_loop(image);
    check_unsupported_source();
    for (const auto allowance : {1U, 2U, 3U, long_grant}) { check_repeat_latch(image, allowance); }
    check_repeat_rebinding(image, false);
    check_repeat_rebinding(image, true);
    check_uncaptured_body(image);
    check_repeat_restriction(image);
    std::cout << "Independent DSP instruction checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
