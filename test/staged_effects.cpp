// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/backend.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace dsp56300::test::staged_test {
namespace {
constexpr Word y_address = 0x240, y_datum = 0x654321;
// Independently encoded X0/(R0)+ and Y0/(R4)+ parallel transfers, no ALU.
constexpr Word pair_base = 0xb01800, x_load = 0x008000, y_load = 0x400000;

struct PairScenario {
  Backend origin, destination;
  unsigned direction;
};

void pair_case(const PairScenario scenario) {
  const bool read_x = (scenario.direction & 1U) != 0;
  const bool read_y = (scenario.direction & 2U) != 0;
  Bus bus;
  bus.events = {
      {.kind = read_x ? EffectKind::read : EffectKind::write, .value = read_x ? raw_datum : 1U},
      {.kind = read_y ? EffectKind::read : EffectKind::write,
       .space = Space::y,
       .address = y_address,
       .value = read_y ? y_datum : 2U}};
  // Functional execution samples input operands before committing output operands.
  // Physical parallel bus phases belong to the consumer's later timing model.
  if (!read_x && read_y) { std::swap(bus.events[0], bus.events[1]); }
  Core core(bus, {.backend = scenario.origin});
  auto initial = seed();
  initial.y = 2;
  initial.r[4] = y_address;
  core.set_registers(initial);
  const auto opcode = pair_base | (read_x ? x_load : 0) | (read_y ? y_load : 0);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}));
  const auto first = pending(core);
  static_cast<void>(core.advance_instruction(first.ticket));
  const auto second = pending(core);
  require(second.effect.space == bus.events[1].space &&
              second.effect.address == bus.events[1].address && core.registers() == initial &&
              bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "The first actual transfer occurs once before the second owned bus boundary");
  rejected([&] { static_cast<void>(core.advance_instruction(first.ticket)); });
  const auto saved = core.checkpoint();
  Bus clone_bus = bus; // The consumer restores matching external state separately.
  Core clone(clone_bus, {.backend = scenario.destination});
  clone.restore(saved);
  rejected([&] { static_cast<void>(clone.advance_instruction(second.ticket)); });
  const auto rebound = pending(clone);
  static_cast<void>(clone.advance_instruction(rebound.ticket));
  require(clone_bus.calls == 2 && clone_bus.accepted == 2 && clone_bus.exact &&
              clone.registers() == initial && clone.progress() == Progress{},
          "Restoring between paired effects never repeats the accepted first read or write");
  auto expected = initial;
  ++expected.pc;
  ++expected.r[0];
  ++expected.r[4];
  if (read_x) { expected.x = datum; }
  if (read_y) { expected.y = y_datum; }
  require(pending(clone).computed == expected,
          "Both independently specified transfer results remain held before retirement");
  retire(clone);
  require(clone.registers() == expected && clone_bus.calls == 2,
          "Retirement publishes both genuine AGU and transfer results without physical replay");
}

template <Backend Kind> void typed() {
  Bus bus;
  BackendCore<Kind> core(bus);
  auto expected = seed();
  core.set_registers(expected);
  require(core.instruction_fetch().address == boot &&
              core.interruptible({.address = boot, .opcode = add_x0}),
          "Typed external owner forwards real fetch and admission metadata");
  const auto result = core.begin_instruction({.address = boot, .opcode = add_x0});
  if (!result.pending) { throw std::runtime_error("Typed computation lost retirement"); }
  const auto saved = core.checkpoint();
  core.restore(saved);
  rejected([&] { static_cast<void>(core.advance_instruction(result.pending->ticket)); });
  const auto rebound = core.pending_instruction();
  if (!rebound) { throw std::runtime_error("Typed restoration lost pending computation"); }
  require(core.advance_instruction(rebound->ticket).retired,
          "Typed adapter preserves owned retirement after restoration");
  expected.a = std::uint64_t{1} << word_bits;
  expected.sr = flag_u;
  ++expected.pc;
  require(core.registers() == expected && bus.calls == 0,
          "Typed external execution publishes literal arithmetic without hidden fetches");
}
} // namespace

void multiple_effects() {
  for (const auto origin : backends) {
    for (const auto destination : backends) {
      for (unsigned direction = 0; direction < 4; ++direction) {
        pair_case({.origin = origin, .destination = destination, .direction = direction});
      }
    }
  }
  typed<Backend::reference>();
  typed<Backend::cached>();
  typed<Backend::jit>();
}
} // namespace dsp56300::test::staged_test
