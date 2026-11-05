// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
using dsp56300::test::staged_test::pending;
constexpr Word vector = 0x40, foreground = 0x100;
// FM MOVEM X0,P:(R0) and short MOVE #7/#9,R2. These are authored diagnostics.
constexpr Word program_store = 0x076084, original_second = 0x320700, changed_second = 0x320900;
constexpr unsigned effect_bound = 32;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};

class Bus final : public Io {
public:
  Word first = 0, second = original_second;
  unsigned reads = 0, writes = 0;
  bool exact = true, refuse_second = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    const auto expected = vector + reads;
    ++reads;
    exact = exact && space == Space::program && address == expected && pc == vector && reads <= 2;
    if (!exact || (address == vector + 1 && refuse_second)) { return false; }
    value = address == vector ? first : second;
    return true;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    ++writes;
    exact = exact && reads == 2 && space == Space::program && address == vector + 1 &&
            pc == vector && value == changed_second && writes == 1;
    if (!exact) { return false; }
    second = value;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    exact = false;
    return false;
  }
};

Registers initial() {
  auto result = dsp56300::test::initial_state();
  result.pc = foreground;
  result.sr = 0;
  result.x = changed_second;
  result.r[0] = vector + 1;
  return result;
}

void drain_to_retire(Core &core) {
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    const auto selected = pending(core);
    if (selected.effect.kind == EffectKind::retire) { return; }
    const auto result = core.advance_instruction(selected.ticket);
    require(result.fault.kind == FaultKind::none && !result.retired,
            "Native vector effects stay healthy before their distinct retirement");
  }
  throw std::runtime_error("Native vector exceeded its bounded effects");
}

void retire(Core &core) {
  drain_to_retire(core);
  require(core.advance_instruction(pending(core).ticket).retired,
          "An actual native vector instruction retires once");
}

void first_read(Core &core, Bus &bus, const InstructionPublication policy) {
  static_cast<void>(core.begin_fetch(policy));
  const auto first = pending(core);
  require(first.effect.kind == EffectKind::fetch_opcode && first.effect.address == vector,
          "Native entry starts with the actual first vector read");
  static_cast<void>(core.advance_instruction(first.ticket));
  const auto second = pending(core);
  require(second.effect.kind == EffectKind::fetch_extension &&
              second.effect.space == Space::program && second.effect.address == vector + 1 &&
              bus.reads == 1 && bus.writes == 0 && core.registers() == initial() &&
              core.progress() == Progress{},
          "Both vector words must be read before the first vector instruction executes");
}

void second_instruction(Core &core, Bus &bus, const InstructionPublication policy) {
  const auto fetch = core.instruction_fetch();
  require(fetch.address == vector + 1 && fetch.exception_vector &&
              fetch.latched_opcode == original_second,
          "The second vector slot exposes its accepted opcode latch");
  bus.second = changed_second;
  static_cast<void>(core.begin_fetch(policy));
  retire(core);
  auto expected = initial();
  expected.r[2] = 7;
  require(
      core.registers() == expected && core.progress().instructions == 2 &&
          core.fault().kind == FaultKind::none && bus.exact && bus.reads == 2,
      "Second-slot execution retains seven despite changed program memory and performs no reread");
}

void owned_pair(const Backend backend, const InstructionPublication policy, const bool store) {
  Bus bus;
  bus.first = store ? program_store : 0;
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = 0});
  first_read(core, bus, policy);
  static_cast<void>(core.advance_instruction(pending(core).ticket));
  drain_to_retire(core);
  require(bus.exact && bus.reads == 2 && bus.writes == (store ? 1U : 0U) &&
              core.progress() == Progress{},
          "The complete pair precedes any real self-modifying P store and all retirements");
  const auto saved = core.checkpoint();
  Bus cloned_bus = bus;
  const auto other = backend == Backend::reference ? Backend::cached : Backend::reference;
  Core clone(cloned_bus, {.backend = other});
  clone.restore(saved);
  retire(core);
  retire(clone);
  require(core.registers() == initial() && clone.registers() == initial() &&
              core.progress().instructions == 1 && clone.progress() == core.progress(),
          "A genuine first vector instruction preserves registers across a fresh-owner restore");
  const auto stopped = core.checkpoint();
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = other});
  resumed.restore(stopped);
  second_instruction(core, bus, policy);
  second_instruction(clone, cloned_bus, policy);
  second_instruction(resumed, resumed_bus, policy);
  require(core.progress() == clone.progress() && core.progress() == resumed.progress() &&
              bus.writes == cloned_bus.writes && bus.writes == resumed_bus.writes,
          "Pre/post-retirement checkpoints retain the pair and never repeat an accepted P store");
}

void refused_pair(const Backend backend, const InstructionPublication policy, const bool store) {
  Bus bus;
  bus.first = store ? program_store : 0;
  bus.refuse_second = true;
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = 0});
  first_read(core, bus, policy);
  const auto saved = core.checkpoint();
  const auto trusted_bus = bus;
  const auto result = core.advance_instruction(pending(core).ticket);
  require(result.fault == Fault{.kind = FaultKind::memory_read,
                                .space = Space::program,
                                .address = vector + 1,
                                .pc = vector} &&
              !result.retired && core.registers() == initial() && core.progress() == Progress{} &&
              bus.reads == 2 && bus.writes == 0 && !core.instruction_outcome(),
          "A refused second vector read cannot execute the first opcode or publish a receipt");
  Core recovered(bus, {.backend = backend});
  recovered.restore(saved);
  bus = trusted_bus;
  bus.refuse_second = false;
  retire(recovered);
  second_instruction(recovered, bus, policy);
}

void atomic_pair(const Backend backend) {
  std::array<Word, dsp56300::test::memory_words> program{};
  std::array<Word, dsp56300::test::memory_words> x{};
  std::array<Word, dsp56300::test::memory_words> y{};
  program.at(vector) = program_store;
  program.at(vector + 1) = original_second;
  Core core({.program = program, .x = x, .y = y}, nullptr, {.backend = backend});
  core.set_registers(initial());
  core.interrupt(vector, 0);
  auto expected = initial();
  expected.r[2] = 7;
  require(core.registers() == expected && core.progress().instructions == 2 &&
              core.fault().kind == FaultKind::none && program.at(vector + 1) == changed_second,
          "Atomic native entry also latches the pair before its real self-modifying instruction");
}
}

int main() {
  try {
    unsigned cases = 0;
    for (const auto backend : dsp56300::test::staged_test::backends) {
      for (const auto policy : policies) {
        for (const bool store : {false, true}) {
          owned_pair(backend, policy, store);
          refused_pair(backend, policy, store);
          ++cases;
        }
      }
      atomic_pair(backend);
    }
    constexpr unsigned expected_cases = 36;
    require(cases == expected_cases, "Every backend/publication/vector combination executes");
    std::cout << "36 native vector-pair cases, refusal/restore and3 atomic self-modifying entries "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
