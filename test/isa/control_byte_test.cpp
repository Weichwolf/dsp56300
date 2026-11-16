// SPDX-License-Identifier: GPL-3.0-only
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
std::uint64_t cases = 0;
using byte_model::Field;
constexpr std::array fields{Field::mr, Field::ccr, Field::com, Field::eom};
constexpr Word mac_a = static_cast<Word>(isa::Opcode::mac_register_a) | (5U << 4);
constexpr Word mac_b = static_cast<Word>(isa::Opcode::mac_register_b) | (5U << 4);
constexpr Word round_a = static_cast<Word>(isa::Opcode::rnd_a);
constexpr Word adc_x_a = static_cast<Word>(isa::Opcode::adc_x_a);
constexpr std::uint64_t half_tie = 0x123456800000;
constexpr Word clear_scaling = 243;

NativeImage prepare() {
  std::vector<std::vector<Word>> words;
  std::vector<CodeSpan> programs;
  constexpr unsigned carry_programs = 8;
  const auto count = (fields.size() * 2 * (status_byte + 1)) + carry_programs + 1;
  words.reserve(count);
  programs.reserve(count);
  const auto append = [&](const Word entry, std::vector<Word> code) {
    words.push_back(std::move(code));
    programs.push_back({.address = entry, .words = words.back()});
  };
  for (const auto field : fields) {
    for (const bool inclusive_or : {false, true}) {
      for (Word immediate = 0; immediate <= status_byte; ++immediate) {
        append(start + immediate,
               {byte_model::opcode(
                   {.field = field, .immediate = immediate, .inclusive_or = inclusive_or})});
      }
    }
  }
  for (const bool inclusive_or : {false, true}) {
    for (const Word immediate : {0U, 1U, 254U, 255U}) {
      append(start,
             {mac_a,
              mac_b,
              byte_model::opcode(
                  {.field = Field::ccr, .immediate = immediate, .inclusive_or = inclusive_or}),
              adc_x_a});
    }
  }
  append(start,
         {byte_model::opcode({.field = Field::mr, .immediate = clear_scaling}),
          byte_model::opcode({.field = Field::mr, .immediate = 8, .inclusive_or = true}),
          round_a});
  return compile_programs(programs);
}

void exhaustive(const NativeImage &image) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  const Buffers memory{.program = p, .x = x, .y = y};
  Core core(Memory::separate(memory));
  std::uint64_t variant = 0;
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
          core.install(image, variant + immediate);
          auto expected = initial;
          byte_model::apply(expected,
                            {.field = field, .immediate = immediate, .inclusive_or = inclusive_or});
          ++expected.pc;
          const auto before = core.progress();
          const auto result = core.run(1);
          ++cases;
          require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                      result.progress ==
                          Progress{.instructions = before.instructions + 1,
                                   .estimated_cycles = before.estimated_cycles + 3} &&
                      core.registers() == expected && core.interrupt_ready(),
                  "Every old byte and immediate combination preserves all untargeted bits/state");
        }
      }
      variant += status_byte + 1;
    }
  }
}

void warm_carry(const NativeImage &image) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  const Buffers memory{.program = p, .x = x, .y = y};
  Core core(Memory::separate(memory));
  std::uint64_t variant = fields.size() * 2 * (status_byte + 1);
  for (const bool inclusive_or : {false, true}) {
    for (const Word immediate : {0U, 1U, 254U, 255U}) {
      const byte_model::Input input{
          .field = Field::ccr, .immediate = immediate, .inclusive_or = inclusive_or};
      const std::array program{mac_a, mac_b, byte_model::opcode(input), adc_x_a};
      core.load(Space::program, start, program);
      core.install(image, variant++);
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

      ++cases;
      require(core.run(4).retired == 4 && core.registers() == expected,
              "CCR changes materialize dirty native A/B and feed the next native Carry consumer");
    }
  }
}

void mode_changes(const NativeImage &image) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  const Buffers memory{.program = p, .x = x, .y = y};
  Core core(Memory::separate(memory));
  const std::array program{
      byte_model::opcode({.field = Field::mr, .immediate = clear_scaling}),
      byte_model::opcode({.field = Field::mr, .immediate = 8, .inclusive_or = true}),
      round_a};
  core.load(Space::program, start, program);
  constexpr std::uint64_t variant = (fields.size() * 2 * (status_byte + 1)) + 8;
  core.install(image, variant);
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
    ++cases;
    require(core.run(3).retired == 3 && core.registers() == expected,
            "ANDI/ORI MR writes leave stale numeric specialization before native rounding");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const auto before = std::chrono::steady_clock::now();
    const auto image = dsp56300::test::prepare();
    const auto ready = std::chrono::steady_clock::now();
    std::cout << "Control-byte preparation="
              << std::chrono::duration<double>(ready - before).count() << "s\n"
              << std::flush;
    dsp56300::test::exhaustive(image);
    dsp56300::test::warm_carry(image);
    dsp56300::test::mode_changes(image);
    std::cout << dsp56300::test::checks << " control byte assertions across "
              << dsp56300::test::cases << " cases passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
