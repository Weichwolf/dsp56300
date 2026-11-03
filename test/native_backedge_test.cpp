// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word loop_flag = Word{1} << 15;
constexpr Word mac_a = 0x2000d3, mac_b = 0x2000fb, add_immediate = 0x0140c0;
constexpr Word immediate = 0xff1234;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array counts{0U, 1U, 2U, 3U, 7U, 255U};

struct Step {
  Word opcode = 0;
  bool immediate = false;
};

struct Input {
  Word count = 0, pointer = 0, target = start, budget = 0;
};

Registers initial(const std::span<const Step> body, const Word mode, const Input input) {
  auto result = register_model::seed(mode | loop_flag | flag_s | flag_l);
  result.la = start - 1;
  for (const auto step : body) { result.la += step.immediate ? 2U : 1U; }
  result.lc = input.count;
  result.sp = input.pointer;
  result.sc = 0; // Continuing execution reads a frame without popping it.
  result.stack[input.pointer % stack_entries] = (std::uint64_t{input.target} << word_bits) | 3;
  return result;
}

struct Expected {
  Registers state;
  Word retired = 0;
  std::uint64_t cycles = 0;
};

Expected model(const std::span<const Step> body, const Registers &state, const Word budget) {
  Expected result{.state = state};
  // A short warm entry now uses its independently expected native prefix.
  // Complete entries retain the guarded whole-body backedge contract.
  const auto steps = std::min(static_cast<Word>(body.size()), budget);
  while (steps != 0 && budget - result.retired >= steps) {
    for (std::size_t index = 0; index < steps; ++index) {
      const auto step = body[index];
      const bool ending = index == body.size() - 1;
      if (ending && result.state.lc <= 1) { return result; }
      auto old = result.state;
      if (step.immediate) { old.x = immediate; }
      sequence_model::alu(result.state, old, step.immediate ? 0x40 : step.opcode);
      result.state.pc += step.immediate ? 2U : 1U;
      if (ending) {
        --result.state.lc;
        const auto frame = result.state.stack[result.state.sp % stack_entries];
        result.state.pc = static_cast<Word>(frame >> word_bits);
      }
      ++result.retired;
      result.cycles += step.immediate ? 2U : 1U;
    }
    if (result.state.pc != state.pc) { return result; }
  }
  return result;
}

void check_case(detail::Jit &jit,
                detail::Cpu &cpu,
                const std::span<const Step> body,
                const Registers &state,
                const Word budget) {
  cpu = {};
  cpu.reg = state;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = 8;
  cpu.cycles = 16;
  const auto expected = model(body, state, budget);
  const auto retired = jit.run(budget);
  auto actual = cpu.reg;
  actual.a >>= detail::padding_bits;
  actual.b >>= detail::padding_bits;
  require(retired == expected.retired && actual == expected.state &&
              cpu.instructions == 8 + expected.retired && cpu.cycles == 16 + expected.cycles,
          "One warm native entry batches only guarded iterations within its exact budget");
  if (state.lc > 3 && state.stack[state.sp % stack_entries] >> word_bits == start &&
      budget >= body.size() * 3) {
    require(retired > body.size(), "A single generated entry really traverses native backedges");
  }
}

void arithmetic(const std::vector<Step> &body, const Word mode) {
  std::array<Word, memory_words> program{};
  auto cursor = start;
  for (const auto step : body) {
    program.at(cursor++) = step.opcode;
    if (step.immediate) { program.at(cursor++) = immediate; }
  }
  detail::Cpu cpu;
  detail::Jit jit(cpu, program, {.backend = Backend::jit}, {});
  const auto size = static_cast<Word>(body.size());
  // Prime a complete body, then reuse it across LC, SP, frame and budget changes.
  cpu.reg = initial(body, mode, {.count = 3});
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  require(jit.run(size) == size, "Warm a complete native body with a one-iteration budget");
  for (const auto count : counts) {
    for (const Word pointer : {0U, 15U, 31U, 63U}) {
      for (const Word target : {start, start + 1, Word{0}, word_mask}) {
        for (const Word budget : {0U,
                                  1U,
                                  size,
                                  size + 1,
                                  size * 2,
                                  size * 3,
                                  size * 255,
                                  std::numeric_limits<Word>::max()}) {
          const auto state =
              initial(body, mode, {.count = count, .pointer = pointer, .target = target});
          check_case(jit, cpu, body, state, budget);
        }
      }
    }
  }
  check_case(jit, cpu, body, initial(body, mode, {.count = word_mask}), size * 3);
  const auto compiled = size == 1 ? 1U : 2U;
  require(
      jit.statistics().compiled_blocks == compiled,
      "Complete body and one short prefix compile once across LC, frame, SP and budget changes");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const std::array bodies{
        std::vector<dsp56300::test::Step>{{.opcode = dsp56300::test::mac_a}},
        std::vector<dsp56300::test::Step>{{.opcode = dsp56300::test::mac_a},
                                          {.opcode = dsp56300::test::mac_b}},
        std::vector<dsp56300::test::Step>{{.opcode = dsp56300::test::mac_a}, {.opcode = 0}},
        std::vector<dsp56300::test::Step>{
            {.opcode = dsp56300::test::mac_b},
            {.opcode = dsp56300::test::add_immediate, .immediate = true}}};
    for (const auto &body : bodies) {
      for (const auto mode : dsp56300::test::modes) { dsp56300::test::arithmetic(body, mode); }
    }
    std::cout << dsp56300::test::checks << " native backedge checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
