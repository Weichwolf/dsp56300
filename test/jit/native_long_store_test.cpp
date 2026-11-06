// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_long_store_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::native_long {
namespace {
constexpr std::array backends{
    Backend::reference, Backend::cached, Backend::jit, Backend::automatic};

void check_store(const Input input) {
  for (const auto backend : backends) {
    std::array<Word, 2> program{opcode(input), input.mode == 6 ? address : 0U};
    std::array<Word, words> x{};
    std::array<Word, words> y{};
    x.fill(sentinel);
    y.fill(sentinel);
    auto expected_x = x;
    auto expected_y = y;
    const auto prediction = expected(input);
    expected_x.at(effective_address(input)) = prediction.data[0];
    expected_y.at(effective_address(input)) = prediction.data[1];
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    cpu.set_registers(seed(input));
    const auto run = cpu.run(1);
    const auto cycles =
        !input.short_address && (input.mode == 5 || input.mode == 6 || input.mode == 7) ? 2U : 1U;
    require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                cpu.registers() == prediction.state && x == expected_x && y == expected_y &&
                cpu.progress() == Progress{.instructions = 1, .estimated_cycles = cycles},
            "Independent long source, 48/24-bit limiting, sticky flags, single AGU and full RAM");
    if (backend == Backend::jit || backend == Backend::automatic) {
      const auto path = cpu.jit_statistics();
      require(path.native_move_instructions == 1 && path.native_alu_instructions == 1 &&
                  path.helper_instructions == 0,
              "Eligible long stores execute emitted code without a general transfer helper");
    }
  }
}

void ordinary_cases() {
  constexpr Word masked_status = 0x300;
  constexpr std::array<std::uint64_t, 9> values{0,
                                                0x007fffffffffff,
                                                0xff800000000000,
                                                0x00800000000000,
                                                0xff7fffffffffff,
                                                0x7fffffffffffff,
                                                0x80000000000000,
                                                0x00123456765432,
                                                0xfffffffffffffd};
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (const unsigned index : {0U, 7U}) {
        if (mode == 6 && index != 0) { continue; }
        check_store({.selector = selector, .mode = mode, .index = index});
      }
    }
    check_store({.selector = selector, .short_address = true});
    for (const auto value : values) {
      for (const Word mode : {0U, mode_down, mode_up}) {
        for (const Word flags : {0U, status_byte}) {
          check_store(
              {.selector = selector, .status = masked_status | mode | flags, .value = value});
        }
      }
    }
  }
}

void repeated_store(const unsigned selector) {
  constexpr Word repeat_four = 0x0604a0;
  constexpr Word saved_lc = 0x357;
  const Input input{.selector = selector, .mode = 3};
  std::array<Word, 2> program{repeat_four, opcode(input)};
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  auto expected_x = x;
  auto expected_y = y;
  const auto prediction = expected(input);
  for (unsigned index = 0; index < 4; ++index) {
    expected_x.at(address + index) = prediction.data[0];
    expected_y.at(address + index) = prediction.data[1];
  }
  auto before = seed(input);
  before.lc = saved_lc;
  Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  cpu.set_registers(before);
  require(cpu.run(1).dispatches == 1 && cpu.progress().estimated_cycles == 5,
          "Long-store REP retains genuine setup and retirement");
  const auto counters = cpu.jit_statistics();
  auto final = prediction.state;
  final.pc = 2;
  final.lc = saved_lc;
  final.r[0] = address + 4;
  const auto run = cpu.run(4);
  require(run.dispatches == 4 && run.fault.kind == FaultKind::none && cpu.registers() == final &&
              x == expected_x && y == expected_y &&
              cpu.progress() == Progress{.instructions = 5, .estimated_cycles = 9},
          "Repeated long stores use one AGU update per shared X/Y destination");
  const auto current = cpu.jit_statistics();
  require(current.native_move_instructions == counters.native_move_instructions + 4 &&
              current.helper_instructions == counters.helper_instructions,
          "Repeated long-store bodies execute emitted code");
}

void dirty_source() {
  constexpr Word add = 0x200040;
  constexpr Word mac = 0x2000d2;
  constexpr Word mac_alu = 0xd2;
  const Input input;
  std::array<Word, 2> program{add, opcode(input) | mac_alu};
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  auto state = seed(input);
  const auto before = state;
  sequence_model::alu(state, before, add);
  const auto prediction = expected({.status = state.sr, .value = state.a});
  state.sr = prediction.state.sr;
  const auto old = state;
  sequence_model::alu(state, old, mac);
  state.pc = 2;
  auto expected_x = x;
  auto expected_y = y;
  expected_x.at(address) = prediction.data[0];
  expected_y.at(address) = prediction.data[1];
  Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  cpu.set_registers(before);
  const auto run = cpu.run(2);
  require(run.dispatches == 2 && run.fault.kind == FaultKind::none && cpu.registers() == state &&
              x == expected_x && y == expected_y,
          "Parallel full-long capture uses dirty cached old A before MAC publication");
  require(cpu.jit_statistics().native_move_instructions == 1 &&
              cpu.jit_statistics().helper_instructions == 0,
          "Parallel long transfers retain native execution");
}
}
} // namespace dsp56300::test::native_long

int main() {
  try {
    using namespace dsp56300::test::native_long;
    ordinary_cases();
    for (unsigned selector = 0; selector < 8; ++selector) { repeated_store(selector); }
    dirty_source();
    boundary_cases();
    for (const auto domain : domains) {
      for (const bool use_long : {false, true}) {
        const auto result = workload(domain, use_long, 1);
        dsp56300::test::require(result.native == result.instructions &&
                                    result.moves == result.instructions / 2 && result.helpers == 0,
                                "Independent three-domain shared fixture executes natively");
      }
    }
    std::cout << "PASS native long stores: " << dsp56300::test::checks << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
