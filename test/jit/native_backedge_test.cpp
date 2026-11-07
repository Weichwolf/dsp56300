// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#if defined(__aarch64__)
#include "a64_loop_state.hpp"
#include "loop_control_model.hpp"
#endif

#include <array>
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
  Fault fault{};
};

#if defined(__aarch64__)
Expected model(const std::span<const Step> body, const Registers &state, const Word budget) {
  loop_model::State modeled{};
  modeled.reg = state;
  Expected result;
  while (result.retired < budget) {
    const auto index = result.retired % body.size();
    const auto step = body[index];
    const auto site = modeled.reg.pc;
    auto old = modeled.reg;
    if (step.immediate) { old.x = immediate; }
    sequence_model::alu(modeled.reg, old, step.immediate ? 0x40 : step.opcode);
    modeled.reg.pc += step.immediate ? 2U : 1U;
    loop_model::finish(modeled, {.pc = site, .words = step.immediate ? 2U : 1U});
    if (modeled.fault.kind != FaultKind::none) { break; }
    ++result.retired;
    result.cycles += step.immediate ? 2U : 1U;
    if (index + 1 == body.size() && modeled.reg.pc != state.pc) { break; }
  }
  result.state = modeled.reg;
  result.fault = modeled.fault;
  return result;
}
#else
Expected model(const std::span<const Step> body, const Registers &state, const Word budget) {
  Expected result{.state = state};
  // A retained successor also handles a partial final iteration. The loop-end
  // guard still rejects final/empty frames before executing that instruction.
  while (result.retired < budget) {
    const auto index = result.retired % body.size();
    const auto step = body[index];
    const bool ending = index + 1 == body.size();
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
    if (ending && result.state.pc != state.pc) { return result; }
  }
  return result;
}
#endif

void check_case(detail::Jit &jit,
                detail::Cpu &cpu,
                Fault &fault,
                const std::span<const Step> body,
                const Registers &state,
                const Word budget) {
  cpu = {};
  fault = {};
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
  require(retired == expected.retired && actual == expected.state && fault == expected.fault &&
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
  Fault fault;
#if defined(__aarch64__)
  const detail::NativeBus bus{.failure = &fault};
#else
  const detail::NativeBus bus{};
#endif
  detail::Jit jit(cpu, program, {}, bus);
  const auto size = static_cast<Word>(body.size());
  const auto published = codegen::publications();
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
          check_case(jit, cpu, fault, body, state, budget);
        }
      }
    }
  }
  check_case(jit, cpu, fault, body, initial(body, mode, {.count = word_mask}), size * 3);
  require(
      codegen::publications().count == published.count + 1,
      "Full and checked paths share one immutable image across LC, frame, SP and grant changes");
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
