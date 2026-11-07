// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

namespace dsp56300::test {
namespace {
std::size_t checks = 0;

void require(const bool condition, const std::string_view message) {
  ++checks;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

static_assert(!std::is_constructible_v<Core, Buffers>);
static_assert(!std::is_constructible_v<Core, Io &>);

void execution() {
  std::array<Word, 32> ram{};
  ram[0] = isa::nop.front();
  ram[1] = isa::stop.front();
  Core core(Memory::unified(ram));
  const auto initial = core.state();
  require(initial.execution == ExecutionState::running && initial.failure.kind == FaultKind::none &&
              initial.progress == Progress{} && core.progress() == Progress{} &&
              initial.registers.pc == 0,
          "Read-only state begins at the reset boundary");
  for (const auto modifier : initial.registers.m) {
    require(modifier == word_mask, "Reset initializes every linear AGU modifier");
  }
  const auto first = core.step();
  require(first.retired == 1 && first.fault.kind == FaultKind::none &&
              first.state == ExecutionState::running && core.state().registers.pc == 1 &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Step executes the actual NOP and publishes its boundary");
  const auto last = core.run();
  require(last.retired == 1 && last.state == ExecutionState::stopped &&
              last.fault.kind == FaultKind::none && core.state().registers.pc == 2,
          "Run reaches the actual STOP without a consumer quantum");
  require(core.run().retired == 0 && core.step().retired == 0, "Stopped execution retires nothing");
  require(initial.registers.pc == 0 && initial.progress == Progress{},
          "A state snapshot remains an independent read-only observation");
  require(core.state().registers == core.registers() &&
              core.state().progress == Progress{.instructions = 2, .estimated_cycles = 11} &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 11},
          "State exposes canonical DSP coordinates and actual instruction progress");
  core.reset();
  require(core.state().registers == initial.registers && core.state().progress == Progress{} &&
              core.progress() == Progress{} && core.state().failure.kind == FaultKind::none,
          "Reset restores the architectural entry state");
  auto seed = initial.registers;
  constexpr std::uint64_t positive = 0x007fffffffffff;
  constexpr std::uint64_t negative = 0xff800000000000;
  seed.a = positive;
  seed.b = negative;
  core.set_registers(seed);
  require(core.state().registers == seed,
          "Inspection exposes unpadded 56-bit accumulators in canonical DSP coordinates");
  require(core.step().retired == 1 && core.state().registers.a == positive &&
              core.state().registers.b == negative,
          "Native execution preserves initialized canonical accumulator values");
}

void invalid_bindings() {
  bool rejected = false;
  try {
    const Core core;
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "External memory needs its actual I/O owner");
  std::array<Word, 32> ram{};
  rejected = false;
  try {
    const Core core(
        Memory::separate({.program = ram, .x = ram, .y = ram, .cache = {.sector_words = 64}}));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Derivative instruction-cache settings retain validation");
}
}
}

int main() {
  try {
    dsp56300::test::execution();
    dsp56300::test::invalid_bindings();
    std::cout << dsp56300::test::checks << " public Core/Memory checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
