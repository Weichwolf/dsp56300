// SPDX-License-Identifier: GPL-3.0-only
// Counted arithmetic recurrence and explicit loop frames; no emulator execution.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word loop_flag = Word{1} << 15;
constexpr Word mac_a = 0x2000d3, mac_b = 0x2000fb, add_immediate = 0x0140c0;
constexpr Word immediate = 0xff1234, add_x0_a = 0x40, do_base = 0x060080;
constexpr std::array counts{1U, 2U, 3U, 4U, 7U, 16U, 64U, 255U};
constexpr std::array pointers{0U, 15U, 31U, 63U};

struct Step {
  Word opcode = 0;
  bool immediate = false;
};

std::vector<Step> body(const unsigned kind) {
  switch (kind) {
    case 0: return {{.opcode = mac_a}};
    case 1: return {{.opcode = mac_a}, {.opcode = mac_b}};
    case 2: return {{.opcode = mac_a}, {.opcode = 0}};
    case 3: return {{.opcode = mac_b}, {.opcode = add_immediate, .immediate = true}};
    default: throw std::invalid_argument("Unknown loop body kind");
  }
}

struct Input {
  Word count = 1, mode = 0, sp = 0, budget_kind = 0;
  unsigned kind = 0;
};

BinaryCase generate(const Input input) {
  const auto steps = body(input.kind);
  std::vector<Word> program{do_base | (input.count << 8), 0};
  for (const auto step : steps) {
    program.push_back(step.opcode);
    if (step.immediate) { program.push_back(immediate); }
  }
  const auto last = start + static_cast<Word>(program.size()) - 1;
  program[1] = last;
  BinaryCase item;
  const auto total = 1 + (input.count * static_cast<Word>(steps.size()));
  const std::array budgets{Word{1}, Word{2}, total - 1, total};
  item.budget = budgets.at(input.budget_kind);
  item.retired = item.budget;
  item.name = "Counted loop count=" + std::to_string(input.count) +
              " body=" + std::to_string(input.kind) + " budget=" + std::to_string(item.budget);
  item.initial = register_model::seed(input.mode | flag_l | flag_s);
  item.initial.sp = input.sp;
  item.expected = item.initial;
  auto &state = item.expected;
  const auto first_slot = (input.sp + 1) % stack_entries;
  const auto second_slot = (input.sp + 2) % stack_entries;
  state.stack[first_slot] = (std::uint64_t{state.la} << word_bits) | state.lc;
  state.stack[second_slot] = (std::uint64_t{start + 2} << word_bits) | state.sr;
  state.sp = (input.sp + 2) % 64;
  state.sc += 2;
  state.lc = input.count;
  state.la = last;
  state.sr |= loop_flag;
  state.pc = start + 2;
  item.progress = {.instructions = item.budget, .estimated_cycles = 5};
  for (Word executed = 0; executed < item.budget - 1; ++executed) {
    const auto position = executed % steps.size();
    const auto step = steps.at(position);
    auto old = state;
    if (step.immediate) { old.x = immediate; }
    sequence_model::alu(state, old, step.immediate ? add_x0_a : step.opcode);
    state.pc += step.immediate ? 2U : 1U;
    item.progress.estimated_cycles += step.immediate ? 2U : 1U;
    if (position != steps.size() - 1) { continue; }
    const auto iteration = (executed / steps.size()) + 1;
    if (iteration < input.count) {
      state.lc = input.count - static_cast<Word>(iteration);
      state.pc = start + 2;
    } else {
      state.sp = item.initial.sp;
      state.sc = item.initial.sc;
      state.la = item.initial.la;
      state.lc = item.initial.lc;
      state.sr &= ~loop_flag;
    }
  }
  item.before.push_back({.words = std::move(program), .address = start});
  return item;
}

void variants(std::vector<BinaryCase> &cases, Input input) {
  for (const auto sp : pointers) {
    input.sp = sp;
    for (Word budget = 0; budget < 4; ++budget) {
      input.budget_kind = budget;
      cases.push_back(generate(input));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_loops OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    for (const auto count : dsp56300::test::counts) {
      for (const auto mode :
           {dsp56300::Word{0}, dsp56300::test::mode_down, dsp56300::test::mode_up}) {
        for (const auto rm : {dsp56300::Word{0}, dsp56300::test::mode_rm}) {
          for (unsigned kind = 0; kind < 4; ++kind) {
            dsp56300::test::variants(cases, {.count = count, .mode = mode | rm, .kind = kind});
          }
        }
      }
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent loop cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
