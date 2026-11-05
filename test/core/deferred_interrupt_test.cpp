// SPDX-License-Identifier: GPL-3.0-only
#include "backend_support.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
constexpr Word vector = 0x54, foreground = 0x100;
constexpr Word wait_opcode = 0x86, stop_opcode = 0x87, repeat_two = 0x0602a0;
constexpr Word move_seven = 0x320700, move_nine = 0x320900;
constexpr unsigned effect_bound = 32, level_shift = 8, maximum_level = 3;
constexpr std::array backends{
    Backend::reference, Backend::cached, Backend::jit, Backend::automatic};
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};
static_assert(CoreImplementation<BackendCore<Backend::automatic>>);

class Bus final : public Io {
public:
  std::array<Word, 2> words{0, move_seven};
  std::array<Word, 4> addresses{}, pcs{};
  std::size_t reads = 0;
  mutable std::size_t inspections = 0;
  std::optional<Word> next;
  std::uint64_t revision = 0;
  Core *owner = nullptr;
  bool reentry_rejected = false;

  std::optional<Word> inspect_program(const Word address) const noexcept override {
    ++inspections;
    return address == foreground ? next : std::nullopt;
  }

  std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (reads < addresses.size()) {
      addresses[reads] = address;
      pcs[reads] = pc;
    }
    ++reads;
    if (owner != nullptr) {
      try {
        owner->request_interrupt(vector, 0);
      } catch (const std::logic_error &) { reentry_rejected = true; } catch (...) {
        return false;
      }
    }
    if (space != Space::program || pc != vector || address < vector || address > vector + 1) {
      return false;
    }
    value = words[address - vector];
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

Registers initial() {
  auto result = dsp56300::test::register_model::seed(0);
  result.pc = foreground;
  result.vba = 0;
  return result;
}

void finish(Core &core, InstructionProgress result) {
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    require(result.fault.kind == FaultKind::none, "An admitted vector has no fabricated fault");
    if (result.retired) { return; }
    const auto pending = core.pending_instruction();
    if (!pending) { throw std::runtime_error("An unfinished instruction lost its owned effect"); }
    result = core.advance_instruction(pending->ticket);
  }
  throw std::runtime_error("Deferred vector exceeded its bounded effects");
}

void captured(Core &core, const Word opcode) {
  finish(core, core.begin_instruction({.address = core.registers().pc, .opcode = opcode}));
}

void prepare(Core &core, const bool waiting) {
  auto state = initial();
  if (waiting) { --state.pc; }
  core.set_registers(state);
  if (waiting) { captured(core, wait_opcode); }
  require(core.registers() == initial() &&
              core.progress() ==
                  (waiting ? Progress{.instructions = 1, .estimated_cycles = 10} : Progress{}) &&
              core.processing_state() ==
                  (waiting ? ProcessingState::waiting : ProcessingState::running),
          "Real WAIT retires ten base cycles and preserves the full foreground state");
}

void complete_vector(Core &core,
                     Bus &bus,
                     const InstructionPublication policy,
                     const Progress base) {
  const auto reads = bus.reads;
  bus.owner = &core;
  finish(core, core.begin_fetch(policy));
  require(bus.reads == reads + 2 && bus.addresses[reads] == vector &&
              bus.addresses[reads + 1] == vector + 1 && bus.pcs[reads] == vector &&
              bus.pcs[reads + 1] == vector && core.registers() == initial() &&
              core.progress() == Progress{.instructions = base.instructions + 1,
                                          .estimated_cycles = base.estimated_cycles + 1} &&
              bus.reentry_rejected,
          "The first real vector fetch accepts both words once, rejects reentry and retires NOP");
  bus.owner = nullptr;
  bus.words[1] = move_nine;
  finish(core, core.begin_fetch(policy));
  auto expected = initial();
  expected.r[2] = 7;
  require(core.registers() == expected && bus.reads == reads + 2 &&
              core.progress() == Progress{.instructions = base.instructions + 2,
                                          .estimated_cycles = base.estimated_cycles + 2} &&
              !core.instruction_fetch().exception_vector,
          "The accepted second vector word survives RAM mutation and resumes the exact foreground");
  const auto refused = core.run(1);
  require(refused.dispatches == 0 && refused.fault.kind == FaultKind::memory_read &&
              refused.fault.space == Space::program && refused.fault.address == foreground &&
              refused.fault.pc == foreground && bus.reads == reads + 3 &&
              core.progress() == Progress{.instructions = base.instructions + 2,
                                          .estimated_cycles = base.estimated_cycles + 2},
          "Unknown foreground performs a genuine refused P read only after the vector returns");
}

void admission(const Backend backend, const InstructionPublication policy, const bool waiting) {
  Bus bus;
  Core core(bus, {.backend = backend});
  prepare(core, waiting);
  const auto base = core.progress();
  const auto inspections = bus.inspections;
  core.request_interrupt(vector, 0);
  require(
      bus.reads == 0 && bus.inspections == inspections && core.registers() == initial() &&
          core.progress() == base && core.processing_state() == ProcessingState::running &&
          core.fault().kind == FaultKind::none && core.instruction_fetch().exception_vector &&
          core.instruction_fetch().address == vector,
      "State-only admission wakes WAIT without reading code, acknowledging a device or retiring");
  complete_vector(core, bus, policy, base);
}

template <class Exception, class Action> void rejects(Core &core, Bus &bus, Action action) {
  const auto registers = core.registers();
  const auto progress = core.progress();
  const auto state = core.processing_state();
  const auto fault = core.fault();
  const bool owned = core.pending_instruction().has_value();
  const auto fetch = owned ? InstructionFetch{} : core.instruction_fetch();
  const auto reads = bus.reads;
  bool rejected = false;
  try {
    action();
  } catch (const Exception &) { rejected = true; }
  require(rejected && core.registers() == registers && core.progress() == progress &&
              core.processing_state() == state && core.fault() == fault && bus.reads == reads &&
              core.pending_instruction().has_value() == owned &&
              (owned || (core.instruction_fetch().address == fetch.address &&
                         core.instruction_fetch().exception_vector == fetch.exception_vector)),
          "Rejected admission preserves actual CPU state and performs no bus effect");
}

void priorities(const Backend backend) {
  for (unsigned mask = 0; mask <= maximum_level; ++mask) {
    for (unsigned level = 0; level <= maximum_level + 1; ++level) {
      Bus bus;
      Core core(bus, {.backend = backend});
      auto state = initial();
      state.sr |= mask << level_shift;
      core.set_registers(state);
      if (level > maximum_level || level < mask) {
        rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, level); });
      } else {
        core.request_interrupt(vector, level);
        require(core.registers() == state && core.progress() == Progress{} && bus.reads == 0 &&
                    core.instruction_fetch().exception_vector,
                "Every admissible priority queues only the actual vector");
      }
    }
  }
}

void exclusions(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  prepare(core, false);
  for (const auto address : {vector + 1, word_mask + 1}) {
    rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(address, 0); });
  }
  const auto held = core.begin_instruction({.address = foreground, .opcode = 0});
  rejects<std::logic_error>(core, bus, [&] { core.request_interrupt(vector, 0); });
  finish(core, held);
  core.request_interrupt(vector, 0);
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  core.reset();
  core.set_registers(initial());
  captured(core, stop_opcode);
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  core.reset();
  core.set_registers(initial());
  core.request_debug();
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  core.reset();
  core.set_registers(initial());
  static_cast<void>(core.run(1));
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
}

void repeat_tail(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = initial();
  state.pc -= 2;
  core.set_registers(state);
  captured(core, repeat_two);
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  captured(core, 0);
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  captured(core, 0);
  require(core.registers().pc == foreground && core.progress().instructions == 3,
          "Two actual REP iterations reach the genuine completed-repeat foreground");
  rejects<std::logic_error>(core, bus, [&] { core.request_interrupt(vector, 0); });
  bus.next = repeat_two;
  ++bus.revision;
  rejects<std::invalid_argument>(core, bus, [&] { core.request_interrupt(vector, 0); });
  bus.next = 0;
  ++bus.revision;
  core.request_interrupt(vector, 0);
  require(bus.reads == 0 && core.progress().instructions == 3 &&
              core.instruction_fetch().exception_vector,
          "Completed REP requires genuine known non-REP metadata and performs no bus read");
}

void synchronous(const Backend backend) {
  Bus bus;
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.interrupt(vector, 0);
  auto expected = initial();
  expected.r[2] = 7;
  require(
      core.registers() == expected && bus.reads == 2 &&
          core.progress() == Progress{.instructions = 2, .estimated_cycles = 2} &&
          !core.instruction_fetch().exception_vector,
      "The existing synchronous API still fetches and executes the full fast vector immediately");
  Bus typed_bus;
  BackendCore<Backend::automatic> typed(typed_bus);
  typed.set_registers(initial());
  typed.request_interrupt(vector, 0);
  require(typed_bus.reads == 0 && typed.progress() == Progress{} &&
              typed.instruction_fetch().exception_vector,
          "Typed automatic backend forwards the genuinely deferred state-only API");
}
} // namespace

int main() {
  try {
    for (const auto backend : backends) {
      for (const auto policy : policies) {
        for (const bool waiting : {false, true}) { admission(backend, policy, waiting); }
      }
      priorities(backend);
      exclusions(backend);
      repeat_tail(backend);
      synchronous(backend);
    }
    std::cout << dsp56300::test::checks << " deferred interrupt checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "deferred interrupt: " << error.what() << '\n';
    return 1;
  }
}
