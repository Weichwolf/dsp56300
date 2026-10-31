// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test::program_fetch_test {
namespace {
using staged_test::backends;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
constexpr Word immediate_x0 = 0x44f400, payload = 0x800001;
constexpr Word repeat_twice = 0x0602a0, add_x0 = 0x200040, clear_a = 0x200013;
constexpr auto policy = InstructionPublication::alu_updates;

Registers initial(const Word pc) {
  auto state = staged_test::seed();
  state.pc = pc;
  return state;
}

void held(Core &core, const Registers &state) {
  require(core.registers() == state && core.progress() == Progress{},
          "Logical program reads and result publication hold unrelated CPU state and retirement");
}

void boundary(Core &core, const Word pc, const unsigned ordinal) {
  auto state = initial(pc);
  if (ordinal > 2) { state.x = payload; }
  held(core, state);
  const auto action = pending(core);
  if (ordinal < 2) {
    require(action.effect.kind ==
                    (ordinal == 0 ? EffectKind::fetch_opcode : EffectKind::fetch_extension) &&
                action.effect.space == Space::program && action.effect.pc == pc &&
                action.effect.address == (ordinal == 0 ? pc : (pc + 1) & word_mask),
            "Native opcode/extension tickets retain real PC, space and modulo24 address");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal == 2) {
    const MoveWriteback result{
        .selector = 4,
        .count = 1,
        .updates = {
            {{.storage = RegisterStorage::x, .mask = word_mask, .previous = 1, .value = payload}}}};
    require(action.effect.kind == EffectKind::move_update && action.effect.move_update == result,
            "Only a real accepted extension computes the native immediate MOVE");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else {
    retire(core);
    state.pc = (pc + 2) & word_mask;
    require(core.registers() == state && core.progress().instructions == 1 &&
                core.fault().kind == FaultKind::none,
            "Complete independently expected instruction state survives P-address wrap");
  }
}

struct BackendPair {
  Backend from, to;
};

void restore(const Word pc, const BackendPair backend, const unsigned cut) {
  Bus bus;
  bus.pc = pc;
  bus.events = {{.space = Space::program, .address = pc, .value = immediate_x0},
                {.space = Space::program, .address = (pc + 1) & word_mask, .value = payload}};
  Core original(bus, {.backend = backend.from});
  original.set_registers(initial(pc));
  static_cast<void>(original.begin_fetch(policy));
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { boundary(original, pc, ordinal); }
  const auto saved = original.checkpoint();
  const auto stale = pending(original);
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = backend.to});
  resumed.restore(saved);
  rejected([&] { static_cast<void>(resumed.advance_instruction(stale.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(stale.ticket)); });
  for (unsigned ordinal = cut; ordinal < 4; ++ordinal) {
    boundary(original, pc, ordinal);
    boundary(resumed, pc, ordinal);
  }
  require(bus.calls == 2 && resumed_bus.calls == 2 && bus.accepted == 2 &&
              resumed_bus.accepted == 2 && bus.exact && resumed_bus.exact,
          "Restoring every read/result boundary never rereads accepted program words");
}

void failure(const Backend backend, const bool extension, const Word pc) {
  Bus bus;
  bus.pc = pc;
  bus.events = {
      {.space = Space::program, .address = pc, .value = immediate_x0, .accept = extension}};
  if (extension) {
    bus.events.push_back(
        {.space = Space::program, .address = (pc + 1) & word_mask, .accept = false});
  }
  Core core(bus, {.backend = backend});
  const auto state = initial(pc);
  core.set_registers(state);
  static_cast<void>(core.begin_fetch(policy));
  if (extension) { boundary(core, pc, 0); }
  const auto request = pending(core);
  const auto failed = core.advance_instruction(request.ticket);
  require(failed.fault == Fault{.kind = FaultKind::memory_read,
                                .space = Space::program,
                                .address = extension ? (pc + 1) & word_mask : pc,
                                .pc = pc} &&
              !failed.retired && !failed.pending,
          "A real refused opcode/extension read produces the exact native memory fault, without "
          "dummy data");
  held(core, state);
  require(bus.calls == (extension ? 2U : 1U) && bus.accepted == (extension ? 1U : 0U) && bus.exact,
          "Failed fetch retains only actually accepted external reads");
}

void complete(Core &core) {
  while (const auto request = core.pending_instruction()) {
    if (core.advance_instruction(request->ticket).retired) { return; }
  }
  throw std::runtime_error("Expected real instruction retirement");
}

void repeat(const Backend backend) {
  Bus bus;
  bus.events = {{.space = Space::program, .address = staged_test::boot, .value = repeat_twice},
                {.space = Space::program, .address = staged_test::boot + 1, .value = add_x0},
                {.space = Space::program, .address = staged_test::boot + 1, .value = clear_a}};
  Core core(bus, {.backend = backend});
  auto state = initial(staged_test::boot);
  core.set_registers(state);
  static_cast<void>(core.begin_fetch(policy));
  complete(core);
  bus.pc = staged_test::boot + 1;
  static_cast<void>(core.begin_fetch(policy));
  complete(core);
  require(bus.calls == 2 && bus.exact,
          "REP and the first body instruction each use their genuine opcode read");
  static_cast<void>(core.begin_fetch(policy));
  require(
      pending(core).effect.kind == EffectKind::alu_update,
      "Subsequent REP execution uses the native opcode latch and performs no speculative P reread");
  complete(core);
  state.a = std::uint64_t{2} << word_bits;
  state.sr = flag_u;
  state.pc += 2;
  require(core.registers() == state && core.progress().instructions == 3 && bus.calls == 2 &&
              bus.exact,
          "Original REP opcode and independent data/CCR/PC/LC state survive a changed potential P "
          "response");
  require(
      core.interruptible_opcode({.address = state.pc, .opcode = immediate_x0}) && bus.calls == 2,
      "REP-tail interrupt eligibility requires only the actual opcode, not an invented extension");
  rejected([&] {
    static_cast<void>(core.interruptible({.address = state.pc, .opcode = immediate_x0}));
  });
}

void run() {
  for (const auto pc : {staged_test::boot, word_mask}) {
    for (const auto from : backends) {
      for (const auto to : backends) {
        for (unsigned cut = 0; cut < 4; ++cut) { restore(pc, {.from = from, .to = to}, cut); }
      }
      failure(from, false, pc);
      failure(from, true, pc);
    }
  }
  for (const auto backend : backends) { repeat(backend); }
}
}
} // namespace dsp56300::test::program_fetch_test

int main() {
  try {
    dsp56300::test::program_fetch_test::run();
    std::cout << dsp56300::test::checks << " native program fetch assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
