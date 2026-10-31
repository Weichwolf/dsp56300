// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
namespace saved = checkpoint_test;
namespace staged = staged_test;
static_assert(CoreImplementation<Core>);
static_assert(CoreImplementation<BackendCore<Backend::reference>>);
static_assert(CoreImplementation<BackendCore<Backend::cached>>);
static_assert(CoreImplementation<BackendCore<Backend::jit>>);

constexpr std::array publications{InstructionPublication::retirement,
                                  InstructionPublication::address_updates,
                                  InstructionPublication::move_updates,
                                  InstructionPublication::alu_updates,
                                  InstructionPublication::unit_updates,
                                  InstructionPublication::pcu_updates};

PendingInstruction completed(Core &core) {
  constexpr unsigned effect_limit = 64;
  for (unsigned index = 0; index < effect_limit; ++index) {
    const auto pending = staged::pending(core);
    if (pending.effect.kind == EffectKind::retire) { return pending; }
    const auto progress = core.advance_instruction(pending.ticket);
    require(progress.fault.kind == FaultKind::none, "Native staged prefix must remain healthy");
  }
  throw std::runtime_error("Native staged instruction exceeded its effect limit");
}

void arithmetic(const InstructionPublication publication) {
  saved::Machine source(Backend::reference);
  source.core.set_registers(saved::seed());
  source.core.load(Space::program, start, std::array{saved::add_x0, Word{0}});
  static_cast<void>(
      source.core.begin_instruction({.address = start, .opcode = saved::add_x0}, publication));
  const auto retirement = completed(source.core);
  const auto visible = source.core.registers();
  const auto progress = source.core.progress();
  const auto candidate = source.core.candidate_checkpoint(retirement.ticket);
  require(source.core.registers() == visible && source.core.progress() == progress,
          "Candidate capture does not publish the original CPU state");
  saved::Machine successor(Backend::reference);
  successor.memory = source.memory;
  successor.core.restore(candidate);
  auto expected = saved::seed();
  saved::add(expected);
  ++expected.pc;
  require(successor.core.registers() == expected &&
              successor.core.progress() == Progress{.instructions = 1, .estimated_cycles = 1} &&
              !successor.core.pending_instruction(),
          "Candidate contains the literal completed ADD and no staged ticket");
  static_cast<void>(successor.core.begin_instruction({.address = start + 1, .opcode = 0}));
  const auto successor_retirement = completed(successor.core);
  saved::invalid(
      [&] { static_cast<void>(successor.core.candidate_checkpoint(retirement.ticket)); });
  require(source.core.advance_instruction(retirement.ticket).retired &&
              source.core.registers() == expected,
          "The original retirement remains independently owned");
  saved::invalid([&] { static_cast<void>(source.core.candidate_checkpoint(retirement.ticket)); });
  require(successor.core.advance_instruction(successor_retirement.ticket).retired,
          "A foreign ticket does not consume the successor retirement");
  ++expected.pc;
  require(successor.core.registers() == expected &&
              successor.core.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
          "The speculative successor executes its actual following NOP");
}

void memory() {
  staged::Bus bus;
  bus.events = {{}};
  Core source(bus);
  const auto initial = staged::seed();
  source.set_registers(initial);
  static_cast<void>(source.begin_instruction({.address = staged::boot, .opcode = staged::read_x0}));
  const auto access = staged::pending(source);
  saved::invalid([&] { static_cast<void>(source.candidate_checkpoint(access.ticket)); });
  const auto retirement = completed(source);
  require(bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "The candidate follows one real accepted peripheral read");
  const auto candidate = source.candidate_checkpoint(retirement.ticket);
  staged::Bus other_bus;
  Core successor(other_bus);
  successor.restore(candidate);
  auto expected = initial;
  expected.x = staged::datum;
  ++expected.r[0];
  ++expected.pc;
  require(successor.registers() == expected && other_bus.calls == 0 && bus.calls == 1,
          "Candidate retains the actual I/O response without replaying the bus");
  saved::invalid([&] { static_cast<void>(successor.candidate_checkpoint(retirement.ticket)); });
  require(source.advance_instruction(retirement.ticket).retired && source.registers() == expected,
          "The source retains its actual retirement ticket after capture");
}

void repeat() {
  saved::Machine source(Backend::reference);
  source.core.set_registers(saved::seed());
  source.core.load(Space::program, start, std::array{saved::repeat_four, saved::add_x0, Word{0}});
  static_cast<void>(source.core.begin_instruction({.address = start, .opcode = saved::repeat_four},
                                                  InstructionPublication::pcu_updates));
  const auto retirement = completed(source.core);
  const auto candidate = source.core.candidate_checkpoint(retirement.ticket);
  saved::Machine successor(Backend::reference);
  successor.memory = source.memory;
  successor.core.restore(candidate);
  auto expected = saved::seed();
  expected.pc = start + 1;
  expected.lc = 4;
  require(successor.core.registers() == expected && !successor.core.interruptible(),
          "Candidate retains REP count and hidden interrupt exclusion");
  for (unsigned index = 0; index < 4; ++index) {
    saved::step(successor.core);
    saved::add(expected);
    expected.lc = index == 3 ? 7U : 3U - index;
    if (index == 3) { ++expected.pc; }
  }
  require(successor.core.registers() == expected &&
              successor.core.progress() == Progress{.instructions = 5, .estimated_cycles = 9},
          "Candidate retains the actual four REP iterations and restoration of LC");
  require(source.core.advance_instruction(retirement.ticket).retired,
          "Speculative REP continuation does not consume the original ticket");
}
}
}

int main() {
  try {
    for (const auto publication : dsp56300::test::publications) {
      dsp56300::test::arithmetic(publication);
    }
    dsp56300::test::memory();
    dsp56300::test::repeat();
    std::cout << "Candidate checkpoints preserve native execution boundaries.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
