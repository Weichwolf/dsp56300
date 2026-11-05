// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_long_load_fixture.hpp"
#include "native_sequence_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::long_load {
namespace {
void check_load(const Input input) {
  constexpr Word sentinel_x = 0x123456;
  constexpr Word sentinel_y = 0x654321;
  for (const auto backend :
       {Backend::reference, Backend::cached, Backend::jit, Backend::automatic}) {
    std::array<Word, 2> program{opcode(input), input.mode == 6 ? address : 0U};
    std::array<Word, words> x{};
    std::array<Word, words> y{};
    x.fill(sentinel_x);
    y.fill(sentinel_y);
    x.at(effective_address(input)) = static_cast<Word>(input.value >> word_bits);
    y.at(effective_address(input)) = static_cast<Word>(input.value & word_mask);
    const auto before_x = x;
    const auto before_y = y;
    auto expected = seed(input);
    expected_load(expected, input);
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
    cpu.set_registers(seed(input));
    const auto result = cpu.run(1);
    const auto cycles =
        !input.short_address && (input.mode == 5 || input.mode == 6 || input.mode == 7) ? 2U : 1U;
    require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                cpu.registers() == expected && x == before_x && y == before_y &&
                cpu.progress() == Progress{.instructions = 1, .estimated_cycles = cycles},
            "Independent long destinations, sign/extension, unchanged flags/RAM and sole AGU");
    if (backend == Backend::jit || backend == Backend::automatic) {
      const auto path = cpu.jit_statistics();
      require(path.native_move_instructions == 1 && path.native_alu_instructions == 1 &&
                  path.helper_instructions == 0,
              "Eligible long loads execute emitted transfers without a general helper");
    }
  }
}

void ordinary_cases() {
  constexpr std::array<std::uint64_t, 7> values{
      0, 1, 0x7fffffffffff, 0x800000000000, 0xffffffffffff, 0x923456abcdef, 0x123456fedcba};
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (const unsigned index : {0U, 7U}) {
        if (mode == 6 && index != 0) { continue; }
        check_load({.selector = selector, .mode = mode, .index = index});
      }
    }
    check_load({.selector = selector, .short_address = true});
    for (const auto value : values) {
      for (const Word mode : {0U, mode_down, mode_up}) {
        for (const Word flags : {0U, status_byte}) {
          check_load(
              {.selector = selector, .status = default_status | mode | flags, .value = value});
        }
      }
    }
  }
}

void parallel_inputs() {
  constexpr Word add = 0x200040;
  constexpr std::array<Word, 3> operations{0xd2, 0x40, 0x43};
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const auto alu : operations) {
      const Input input{.selector = selector};
      std::array program{add, opcode(input) | alu, add};
      std::array<Word, words> x{};
      std::array<Word, words> y{};
      x.at(address) = static_cast<Word>(input.value >> word_bits);
      y.at(address) = static_cast<Word>(input.value & word_mask);
      auto expected = seed(input);
      auto old = expected;
      sequence_model::alu(expected, old, add);
      ++expected.pc;
      old = expected;
      expected_load(expected, input);
      expected_alu(expected, old, parallel_none | alu);
      old = expected;
      sequence_model::alu(expected, old, add);
      ++expected.pc;
      Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
      cpu.set_registers(seed(input));
      const auto result = cpu.run(3);
      if (cpu.registers() != expected) {
        const auto actual = cpu.registers();
        std::cerr << "selector=" << selector << " alu=" << std::hex << alu
                  << " expected a=" << expected.a << " b=" << expected.b << " x=" << expected.x
                  << " y=" << expected.y << " sr=" << expected.sr << " pc=" << expected.pc
                  << " actual a=" << actual.a << " b=" << actual.b << " x=" << actual.x
                  << " y=" << actual.y << " sr=" << actual.sr << " pc=" << actual.pc << '\n';
      }
      require(result.dispatches == 3 && result.fault.kind == FaultKind::none &&
                  cpu.registers() == expected,
              "Long loads preserve dirty old ALU inputs and publish real destinations afterward");
      require(cpu.jit_statistics().native_move_instructions == 1 &&
                  cpu.jit_statistics().native_alu_instructions == 3 &&
                  cpu.jit_statistics().helper_instructions == 0,
              "Parallel long load and surrounding ALU remain native");
    }
  }
}

void partial_range_invalidation() {
  constexpr std::uint64_t negative_long = 0x800000000000;
  constexpr Word negative_word = 0x800000;
  constexpr std::array<Word, 2> clears{0x200013, 0x20001b};
  constexpr std::array<Word, 2> additions{0x200040, 0x200048};
  for (unsigned selector = 0; selector < 2; ++selector) {
    const Input input{.selector = selector,
                      .status = default_status | saturation_model::mode,
                      .value = negative_long};
    const Word clear = clears.at(selector);
    const Word add_zero = additions.at(selector);
    std::array program{clear, opcode(input), add_zero};
    std::array<Word, words> x{};
    std::array<Word, words> y{};
    x.at(address) = negative_word;
    auto before = seed(input);
    before.x = 0;
    auto expected = before;
    auto old = expected;
    saturation_model::alu(expected, old, clear);
    ++expected.pc;
    expected_load(expected, input);
    old = expected;
    saturation_model::alu(expected, old, add_zero);
    ++expected.pc;
    Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
    cpu.set_registers(before);
    const auto actual = cpu.run(3);
    require(actual.dispatches == 3 && actual.fault.kind == FaultKind::none &&
                cpu.registers() == expected && cpu.jit_statistics().native_move_instructions == 1 &&
                cpu.jit_statistics().native_alu_instructions == 3 &&
                cpu.jit_statistics().helper_instructions == 0,
            "A10/B10 load invalidates narrow proofs before actual subsequent SM saturation");
  }
}
}
} // namespace dsp56300::test::long_load

int main() {
  try {
    using namespace dsp56300::test::long_load;
    ordinary_cases();
    parallel_inputs();
    partial_range_invalidation();
    repeated_cases();
    boundary_cases();
    std::cout << "PASS native long loads: " << dsp56300::test::checks << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
