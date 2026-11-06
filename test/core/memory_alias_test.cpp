// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word store_x0_negate_a = 0x446036, clear_a = 0x200013, negate_a = 0x200036;
constexpr Word move_immediate_a = 0x56f400;
constexpr Word displacement = 16;

enum class Arrangement : std::uint8_t { same, program_after_data, data_after_program };

void guest_write(const Backend backend, const bool y, const Arrangement arrangement) {
  std::array<Word, memory_words + displacement> shared{};
  std::array<Word, memory_words> other{};
  const Word program_offset = arrangement == Arrangement::program_after_data ? displacement : 0;
  const Word data_offset = arrangement == Arrangement::data_after_program ? displacement : 0;
  const std::span program{shared.data() + program_offset, memory_words};
  const std::span data{shared.data() + data_offset, memory_words};
  TestCore core(
      {.program = program, .x = y ? std::span{other} : data, .y = y ? data : std::span{other}},
      nullptr,
      {.backend = backend});
  constexpr Word y_space = 0x080000;
  const std::array words{store_x0_negate_a | (y ? y_space : 0), clear_a};
  core.load(Space::program, start, words);
  auto state = initial_state();
  state.r[0] = start + 1 + program_offset - data_offset;
  state.a = 3;
  core.set_registers(state);
  const auto result = core.run(2);
  require(result.retired == 2 && result.fault.kind == FaultKind::none,
          "Guest data write through a program alias completes");
  require(core.registers().a == accumulator_mask - 2 && core.registers().pc == start + 2 &&
              program[start + 1] == 0,
          "A data-space program alias invalidates the planned successor before execution");
}

void host_write(const Backend backend, const bool y, const bool extension) {
  std::array<Word, memory_words + displacement> shared{};
  std::array<Word, memory_words> other{};
  const std::span program{shared.data() + displacement, memory_words};
  const std::span data{shared.data(), memory_words};
  TestCore core(
      {.program = program, .x = y ? std::span{other} : data, .y = y ? data : std::span{other}},
      nullptr,
      {.backend = backend});
  const std::array words{extension ? move_immediate_a : clear_a, Word{1}};
  core.load(Space::program, start, words);
  auto state = initial_state();
  state.a = 3;
  core.set_registers(state);
  require(core.run(1).retired == 1, "Program alias translation is populated");
  const auto space = y ? Space::y : Space::x;
  core.write(space, start + displacement + (extension ? 1U : 0U), extension ? 2U : negate_a);
  core.set_registers(state);
  require(core.run(1).retired == 1 &&
              core.registers().a ==
                  (extension ? (std::uint64_t{2} << word_bits) : accumulator_mask - 2),
          "Host data-space writes invalidate opcode and extension aliases");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend : {dsp56300::Backend::jit}) {
      for (const bool y : {false, true}) {
        for (const auto arrangement : {dsp56300::test::Arrangement::same,
                                       dsp56300::test::Arrangement::program_after_data,
                                       dsp56300::test::Arrangement::data_after_program}) {
          dsp56300::test::guest_write(backend, y, arrangement);
        }
        for (const bool extension : {false, true}) {
          dsp56300::test::host_write(backend, y, extension);
        }
      }
    }
    std::cout << dsp56300::test::checks << " memory-alias checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
