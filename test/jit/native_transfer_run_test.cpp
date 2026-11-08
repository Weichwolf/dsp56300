// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"
#include "transfer_run_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
namespace fixture = transfer_run;

struct Point {
  Word mode = 0, count = 1, prefix = 1, chunk = 0, hole = fixture::ram_words;
  bool shared = false, deadline = false, yield = false;
};

void check(fixture::Machine &machine, const Point point) {
  const auto x = machine.x;
  const auto y = machine.y;
  machine.x_known.fill(1);
  machine.y_known.fill(1);
  if (point.hole < fixture::ram_words) {
    machine.x_known[point.hole] = machine.y_known[point.hole] = 0;
  }
  machine.accesses = 0;
  machine.observed = true;
  machine.yield_on_access = point.yield;
  auto expected = fixture::initial(
      {.iterations = point.count, .mode = point.mode, .modifier = fixture::ram_words - 1});
  auto expected_x = x;
  auto expected_y = y;
  machine.core.set_registers(expected);
  const auto before = machine.core.state().progress;
  machine.base = before.instructions;
  Word retired = 0;
  while (retired < point.prefix) {
    auto budget = point.chunk == 0 ? point.prefix : std::min(point.chunk, point.prefix - retired);
    // A callback can request a yield, but the current paired operation completes.
    if (point.yield) { budget = std::min(budget, (point.hole * fixture::length) + 1 - retired); }
    for (Word index = 0; index < budget; ++index) {
      fixture::step(
          expected, {.x = expected_x, .y = point.shared ? expected_x : expected_y}, machine.kernel);
    }
    const auto run = point.deadline
                         ? machine.core.run_until(before.estimated_cycles + retired + budget)
                         : machine.core.run(budget);
    retired += budget;
    require(run.retired == budget && run.state == ExecutionState::running &&
                run.fault.kind == FaultKind::none &&
                run.yielded == (point.yield && machine.accesses != 0) &&
                run.progress == Progress{.instructions = before.instructions + retired,
                                         .estimated_cycles = before.estimated_cycles + retired} &&
                machine.core.registers() == expected && machine.x == expected_x &&
                machine.y == expected_y,
            "Cold/warm whole and partitioned native transfers preserve independent complete "
            "registers, RAM, flags, loop restoration and exact progress");
    if (point.yield && machine.accesses != 0) { break; }
  }
  require(point.hole != fixture::ram_words || machine.accesses == 0,
          "Initialized ordinary RAM never invokes an owner callback");
  require(machine.observed, "Owner callbacks see the exact prior instruction/cycle prefix");
  machine.x = x;
  machine.y = y;
}

void prefixes(fixture::Machine &machine, Point point) {
  for (const Word prefix :
       {1U, (point.count * fixture::length) - 1, point.count * fixture::length}) {
    point.prefix = prefix;
    for (const Word chunk : {0U, 1U, 3U}) {
      point.chunk = chunk;
      for (const bool deadline : {false, true}) {
        point.deadline = deadline;
        check(machine, point);
        check(machine, point);
      }
    }
  }
}

void kernels() {
  using isa::Opcode;
  constexpr std::array kernels{
      fixture::Kernel{},
      fixture::Kernel{.load_alu = Opcode::clr_a},
      fixture::Kernel{.store_alu = Opcode::clr_b},
      fixture::Kernel{.load_alu = Opcode::asr_a},
      fixture::Kernel{.store_alu = Opcode::add_y0_a},
      fixture::Kernel{.load_alu = Opcode::tfr_x0_a, .store_alu = Opcode::asr_b}};
  constexpr std::array modes{
      0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
  for (const auto kernel : kernels) {
    for (const bool shared : {false, true}) {
      fixture::Machine machine(kernel, shared);
      for (const auto mode : modes) {
        for (const Word count : {1U, 3U, 17U, 67U}) {
          prefixes(machine, {.mode = mode, .count = count, .shared = shared});
        }
      }
    }
  }
}

void observer_points(fixture::Machine &machine, const bool shared) {
  for (const auto mode : {0U, mode_down, mode_up}) {
    for (const Word hole : {0U, 1U, 5U}) {
      for (const bool yield : {false, true}) {
        const Point point{.mode = mode,
                          .count = 17,
                          .prefix = 34,
                          .hole = hole,
                          .shared = shared,
                          .yield = yield};
        check(machine, point);
        check(machine, point);
      }
    }
  }
}

void observers() {
  using isa::Opcode;
  constexpr std::array kernels{fixture::Kernel{},
                               fixture::Kernel{.load_alu = Opcode::clr_a},
                               fixture::Kernel{.store_alu = Opcode::clr_b},
                               fixture::Kernel{.load_alu = Opcode::asr_a},
                               fixture::Kernel{.store_alu = Opcode::add_y0_a}};
  for (const auto kernel : kernels) {
    for (const bool shared : {false, true}) {
      fixture::Machine machine(kernel, shared);
      observer_points(machine, shared);
    }
  }
}

}
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::kernels();
    dsp56300::test::observers();
    std::cout << dsp56300::test::checks << " independent native transfer-run checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
