// SPDX-License-Identifier: GPL-3.0-only
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using byte_model::Field;
constexpr std::array fields{Field::mr, Field::ccr, Field::com, Field::eom};
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, round_a = 0x200011, adc_x_a = 0x200021;
constexpr std::uint64_t half_tie = 0x123456800000;
constexpr Word clear_scaling = 243, exhaustive_operations = 524288;

void exhaustive(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core(
      {.program = p, .x = x, .y = y}, nullptr, {.backend = backend, .instructions_per_block = 1});
  for (const auto field : fields) {
    for (const bool inclusive_or : {false, true}) {
      std::array<Word, status_byte + 1> program{};
      for (Word immediate = 0; immediate <= status_byte; ++immediate) {
        program[immediate] = byte_model::opcode(
            {.field = field, .immediate = immediate, .inclusive_or = inclusive_or});
      }
      core.load(Space::program, start, program);
      for (Word old = 0; old <= status_byte; ++old) {
        auto initial = byte_model::seed(field, old);
        for (Word immediate = 0; immediate <= status_byte; ++immediate) {
          initial.pc = start + immediate;
          core.set_registers(initial);
          auto expected = initial;
          byte_model::apply(expected,
                            {.field = field, .immediate = immediate, .inclusive_or = inclusive_or});
          ++expected.pc;
          const auto before = core.progress();
          const auto result = core.run(1);
          require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                      result.progress ==
                          Progress{.instructions = before.instructions + 1,
                                   .estimated_cycles = before.estimated_cycles + 3} &&
                      core.registers() == expected && core.interruptible(),
                  "Every old byte and immediate combination preserves all untargeted bits/state");
        }
      }
    }
  }
  if (backend == Backend::jit) {
    require(core.jit_statistics().helper_instructions == exhaustive_operations,
            "The exhaustive byte matrix executes actual translated one-operation helper blocks");
  }
}

void warm_carry(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  for (const bool inclusive_or : {false, true}) {
    for (const Word immediate : {0U, 1U, 254U, 255U}) {
      const byte_model::Input input{
          .field = Field::ccr, .immediate = immediate, .inclusive_or = inclusive_or};
      const std::array program{mac_a, mac_b, byte_model::opcode(input), adc_x_a};
      core.load(Space::program, start, program);
      auto initial = register_model::seed(0);
      initial.sr = inclusive_or ? 0U : status_byte;
      core.set_registers(initial);
      auto expected = initial;
      for (const auto operation : {mac_a, mac_b}) {
        const auto old = expected;
        sequence_model::alu(expected, old, operation);
      }
      byte_model::apply(expected, input);
      const auto old = expected;
      sequence_model::alu(expected, old, adc_x_a);
      expected.pc = start + static_cast<Word>(program.size());
      const auto native = core.jit_statistics().native_alu_instructions;
      require(core.run(4).dispatches == 4 && core.registers() == expected,
              "CCR changes materialize dirty native A/B and feed the next native Carry consumer");
      if (backend == Backend::jit) {
        require(
            core.jit_statistics().native_alu_instructions == native + 3,
            "The CCR boundary sequence includes actual native arithmetic and Carry consumption");
      }
    }
  }
}

void mode_changes(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  const std::array program{
      byte_model::opcode({.field = Field::mr, .immediate = clear_scaling}),
      byte_model::opcode({.field = Field::mr, .immediate = 8, .inclusive_or = true}),
      round_a};
  core.load(Space::program, start, program);
  for (const Word mode :
       {0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down}) {
    auto initial = register_model::seed(mode);
    initial.a = half_tie;
    core.set_registers(initial);
    auto expected = initial;
    byte_model::apply(expected, {.field = Field::mr, .immediate = clear_scaling});
    byte_model::apply(expected, {.field = Field::mr, .immediate = 8, .inclusive_or = true});
    const auto old = expected;
    sequence_model::alu(expected, old, round_a);
    expected.pc = start + static_cast<Word>(program.size());
    require(core.run(3).dispatches == 3 && core.registers() == expected,
            "ANDI/ORI MR writes leave stale numeric specialization before native rounding");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::exhaustive(backend);
      dsp56300::test::warm_carry(backend);
      dsp56300::test::mode_changes(backend);
    }
    std::cout << dsp56300::test::checks
              << " control byte exhaustive and native-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
