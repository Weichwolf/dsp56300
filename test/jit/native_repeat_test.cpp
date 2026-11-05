// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "native_repeat_fixture.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::native_repeat {
namespace {
constexpr Word post_load_xor = 0x44d843, post_store = 0x445800, datum = 0x9654a1;
constexpr Word four = 4;

struct BudgetCase {
  Word mode = 0, count = 0;
  RunBudget budget;
  bool bounded = false;
};

void budget_case(const external_run::Workload input, const BudgetCase request) {
  const auto count = request.count;
  const auto mode = request.mode;
  const auto instructions = request.budget.instructions;
  const auto cycles = static_cast<Word>(request.budget.cycles);
  const auto bounded = request.bounded;
  Bus bus;
  bus.program[start] = repeat(count);
  bus.program[body] = input.opcode;
  Core core(bus);
  core.set_registers(seed(input, mode));
  require(core.run(1).dispatches == 1 && core.progress().estimated_cycles == 5 &&
              !core.interruptible(),
          "Actual REP setup retires five cycles and excludes IRQs");
  const auto counters = core.jit_statistics();
  const auto total = count == 0 ? zero_count : count;
  const auto wanted = bounded ? std::min(instructions, cycles) : instructions;
  auto state = expected(input, {.mode = mode, .count = count, .retired = std::min(total, wanted)});
  state.pc += wanted - std::min(total, wanted);
  const auto result =
      bounded ? core.run(RunBudget{.instructions = instructions, .cycles = cycles}).execution
              : core.run(instructions);
  require(result.dispatches == wanted && result.fault.kind == FaultKind::none &&
              core.registers() == state &&
              core.progress() == Progress{.instructions = 1 + std::uint64_t{wanted},
                                          .estimated_cycles = 5 + std::uint64_t{wanted}},
          "Native REP retains the independent full state, exact budgets and restored LC");
  require(core.jit_statistics().native_alu_instructions - counters.native_alu_instructions ==
                  wanted &&
              core.jit_statistics().helper_instructions == counters.helper_instructions &&
              bus.program_reads == 0 && core.interruptible() == (wanted >= total),
          "Eligible repeated bodies execute genuinely natively without a fetch or helper");
}

void budgets() {
  for (const auto input : external_run::workloads) {
    for (const auto mode : modes) {
      for (const Word count : {0U, 1U, 2U, four}) {
        for (Word instructions = 0; instructions <= four; ++instructions) {
          budget_case(input,
                      {.mode = mode, .count = count, .budget = {.instructions = instructions}});
          for (Word cycles = 0; cycles <= four; ++cycles) {
            budget_case(input,
                        {.mode = mode,
                         .count = count,
                         .budget = {.instructions = instructions, .cycles = cycles},
                         .bounded = true});
          }
        }
      }
    }
    budget_case(input,
                {.budget = {.instructions = zero_count, .cycles = zero_count}, .bounded = true});
  }
}

void mutation_and_restore() {
  for (const auto backend : external_run::backends) {
    Bus bus;
    const auto input = external_run::workloads.front();
    bus.program[start] = repeat(four);
    bus.program[body] = input.opcode;
    Core core(bus);
    core.set_registers(seed(input));
    require(core.run(2).dispatches == 2, "REP setup and first body retire before mutation");
    bus.program[body] = external_run::workloads.back().opcode;
    ++bus.revision;
    const auto saved = core.checkpoint();
    Core restored(bus, {.backend = backend});
    restored.restore(saved);
    bus.visible = false;
    require(
        restored.run(3).dispatches == 3 &&
            restored.registers() == expected(input, {.count = four, .retired = four}),
        "Fresh cross-backend restore uses the genuine retained opcode despite unknown backing P");
    if (backend == Backend::jit) {
      require(restored.jit_statistics().native_alu_instructions == 3 &&
                  restored.jit_statistics().helper_instructions == 0,
              "A restored latched REP body resumes natively without backing-code inspection");
    }
    bus.visible = true;
    core.restore(saved);
    require(core.run(3).dispatches == 3 &&
                core.registers() == expected(input, {.count = four, .retired = four}),
            "Code invalidation cannot replace a running REP latch");
    core.set_registers(seed(external_run::workloads.back()));
    require(core.run(5).dispatches == 5 &&
                core.registers() ==
                    expected(external_run::workloads.back(), {.count = four, .retired = four}),
            "A new REP at the same PC uses its new actual body rather than an old translation");
  }
}

Registers memory_seed() {
  auto state = seed(external_run::workloads.back());
  state.r[0] = data_address;
  return state;
}

void load_step(Registers &state, const Word value) {
  const auto old = state;
  ++state.r[0];
  register_model::write(state, 4, value);
  bfu_model::apply(state, old, {.function = bfu_model::Function::bit_xor});
}

void memory_prefix(const bool bounded) {
  Bus bus;
  bus.program[start] = repeat(four);
  bus.program[body] = post_load_xor;
  bus.data[data_address] = datum;
  bus.data[data_address + 1] = datum ^ word_mask;
  bus.known[data_address] = bus.known[data_address + 1] = 1;
  Core core(bus);
  core.set_registers(memory_seed());
  require(core.run(1).dispatches == 1, "Real REP setup precedes direct RAM");
  auto wanted = memory_seed();
  load_step(wanted, datum);
  load_step(wanted, datum ^ word_mask);
  wanted.pc = body;
  wanted.lc = 2;
  const auto result = core.run(RunBudget{.instructions = four, .cycles = four});
  require(result.execution.dispatches == 2 && result.cycles == 2 &&
              result.stop == RunStop::external_effect && core.registers() == wanted &&
              core.jit_statistics().native_move_instructions == 2 && bus.reads == 0,
          "A later unknown REP operand preserves the exact native prefix and pre-effect boundary");
  const auto checkpoint = core.checkpoint();
  const auto again = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(again.execution.dispatches == 0 && core.registers() == wanted && bus.reads == 0,
          "A repeated failed guard neither consumes a count nor replays the accepted prefix");
  core.restore(checkpoint);
  bus.data[data_address + 2] = 7;
  bus.data[data_address + 3] = 11;
  bus.known[data_address + 2] = bus.known[data_address + 3] = 1;
  load_step(wanted, 7);
  load_step(wanted, 11);
  wanted.pc = body + 1;
  wanted.lc = saved_lc;
  const auto done =
      bounded ? core.run(RunBudget{.instructions = 2, .cycles = 2}).execution : core.run(2);
  require(
      done.dispatches == 2 && core.registers() == wanted && bus.reads == 0 &&
          core.jit_statistics().native_move_instructions == four,
      "A real owner write repairs knowledge and the retained native body resumes after restore");
}

void refusal_and_yield(const bool yield) {
  Bus bus;
  bus.program[start] = repeat(four);
  bus.program[body] = post_load_xor;
  bus.grant = false;
  bus.yield = yield;
  bus.data[data_address] = datum;
  bus.known[data_address] = yield ? 1 : 0;
  Core core(bus);
  bus.owner = &core;
  core.set_registers(memory_seed());
  require(core.run(1).dispatches == 1, "Owner fallback starts after genuine REP setup");
  const auto before = core.registers();
  const auto stopped = core.run(RunBudget{.instructions = four, .cycles = four});
  require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
              core.registers() == before && bus.reads == 0,
          "An ungranted repeated operand has no speculative CPU or bus effect");
  const auto actual = core.run(four);
  if (yield) {
    auto wanted = memory_seed();
    load_step(wanted, datum);
    wanted.pc = body;
    wanted.lc = 3;
    require(actual.dispatches == 1 && actual.yielded && core.registers() == wanted &&
                bus.reads == 1,
            "The real callback yields after exactly one repeated owner instruction");
  } else {
    require(actual.dispatches == 0 && actual.fault.kind == FaultKind::memory_read &&
                actual.fault.space == Space::x && actual.fault.address == data_address &&
                actual.fault.pc == body && bus.reads == 1,
            "Unsupported REP data still produces the real precise owner refusal");
  }
}

void stores() {
  Bus bus;
  bus.program[start] = repeat(four);
  bus.program[body] = post_store;
  Core core(bus);
  auto wanted = memory_seed();
  core.set_registers(wanted);
  require(core.run(1).dispatches == 1 && core.run(4).dispatches == four,
          "Native REP stores execute the actual repeated opcode");
  wanted.r[0] += four;
  wanted.pc = body + 1;
  require(core.registers() == wanted && bus.writes == 0 &&
              core.jit_statistics().native_move_instructions == four,
          "Repeated native stores retain independent address and LC state without callbacks");
  for (Word index = 0; index < four; ++index) {
    require(bus.data[data_address + index] == (wanted.x & word_mask) &&
                bus.known[data_address + index] == 1,
            "Each repeated native store initializes only its actual word and known byte");
  }
}

void general_fallbacks() {
  for (const auto input : external_run::workloads) {
    Bus bus;
    bus.program[start] = repeat(four);
    bus.program[body] = input.opcode;
    Core core(bus, {.optimize = false});
    core.set_registers(seed(input));
    require(core.run(1).dispatches == 1, "Disabled lowering retains the real REP setup");
    const auto counters = core.jit_statistics();
    require(core.run(four).dispatches == four &&
                core.registers() == expected(input, {.count = four, .retired = four}) &&
                core.jit_statistics().native_alu_instructions == 0 &&
                core.jit_statistics().helper_instructions - counters.helper_instructions == four,
            "Explicitly disabled lowering preserves full repeated state in the general executor");
  }
  Bus bus;
  constexpr Word test_a = 0x200003;
  bus.program[start] = repeat(four);
  Core core(bus);
  for (const bool supported : {false, true, false}) {
    const auto input = external_run::workloads.front();
    bus.program[body] = supported ? input.opcode : test_a;
    ++bus.revision;
    core.set_registers(seed(input));
    require(core.run(1).dispatches == 1, "Each changed repeated body has a genuine setup");
    auto wanted = expected(input, {.count = four, .retired = four});
    if (!supported) {
      wanted.a = 0;
      wanted.sr = ce | flag_z | flag_u; // TST of zero: Z/U set, E/N/V clear.
    }
    const auto before = core.jit_statistics();
    require(core.run(four).dispatches == four && core.registers() == wanted &&
                core.jit_statistics().native_alu_instructions - before.native_alu_instructions ==
                    (supported ? four : 0U) &&
                core.jit_statistics().helper_instructions - before.helper_instructions ==
                    (supported ? 0U : four),
            "An unsupported REP body retains real execution and cannot poison a later native body");
  }
}

void loop_ending() {
  Bus bus;
  constexpr Word do_two = 0x060280;
  constexpr Word repeat_address = start + 2;
  constexpr Word loop_body = start + 3;
  bus.program[start] = do_two;
  bus.program[start + 1] = loop_body;
  bus.program[repeat_address] = repeat(four);
  bus.program[loop_body] = external_run::workloads.front().opcode;
  Core core(bus);
  auto initial = seed(external_run::workloads.front());
  core.set_registers(initial);
  require(core.run(11).dispatches == 11, "Two real DO iterations each execute a four-body REP");
  auto wanted = initial;
  for (unsigned index = 0; index < 8; ++index) { arithmetic(wanted, bus.program[loop_body]); }
  wanted.pc = loop_body + 1;
  wanted.stack[1] = saved_lc;
  wanted.stack[2] = (std::uint64_t{repeat_address} << word_bits) | initial.sr;
  require(core.registers() == wanted && core.interruptible(),
          "REP completion alone performs each actual DO ending and restores the genuine frame");
}
}
}

int main() {
  try {
    using namespace dsp56300::test::native_repeat;
    budgets();
    mutation_and_restore();
    memory_prefix(false);
    memory_prefix(true);
    refusal_and_yield(false);
    refusal_and_yield(true);
    stores();
    general_fallbacks();
    loop_ending();
    std::cout << dsp56300::test::checks << " native REP checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "native REP: " << error.what() << '\n';
    return 1;
  }
}
