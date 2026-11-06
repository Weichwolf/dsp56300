// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "support.hpp"

namespace dsp56300::test::external_run {
namespace {
constexpr Word short_x0 = 0x240700;
constexpr Word immediate_x1 = 0x45f400;
// FM immediate-short MOVE to a data register aligns its byte to bits23:16.
constexpr Word short_datum = 0x070000;

void cache_case(const Backend backend, const unsigned width, const bool burst) {
  Bus bus;
  bus.program(origin, {immediate_x0, datum, 0, 0, program_store, origin});
  const Options options{.backend = backend,
                        .cacheable_begin = origin,
                        .cacheable_end = origin + 4,
                        .cache_sector_words = width};
  Core core(bus, options);
  auto expected = initial_state();
  expected.sr = ce;
  expected.omr = burst ? be : 0U;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc += 2;
  expected.x = datum;
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "Real external P cache fill publishes the independent immediate MOVE result");
  const auto initial_reads = burst ? 4U : 2U;
  require(
      bus.reads == initial_reads && bus.events.size() == initial_reads,
      "External instruction storage performs exactly the documented single/burst source requests");
  for (unsigned index = 0; index < initial_reads; ++index) {
    require(bus.events[index] ==
                Event{.operation = Operation::read, .address = origin + index, .pc = origin},
            "Every real cache-line read carries its actual address and owning instruction PC");
  }
  bus.memory[{Space::program, origin}] = immediate_x1; // Authored immediate X1, replacing RAM only.
  bus.memory[{Space::program, origin + 1}] = other;
  expected.pc = origin;
  core.set_registers(expected);
  const auto replay = core.run(1);
  expected.pc += 2;
  require(
      replay.dispatches == 1 && replay.fault.kind == FaultKind::none &&
          core.registers() == expected && bus.reads == initial_reads,
      "Retained architectural code/data override later RAM contents without a fake bus callback");
  auto before = expected;
  before.pc = origin + 4;
  before.x = short_x0;
  core.set_registers(before);
  const auto store = core.run(1);
  require(store.dispatches == 1 && store.fault.kind == FaultKind::none && bus.writes == 1 &&
              bus.memory.at({Space::program, origin}) == short_x0,
          "External owners execute a genuine P store into a retained code line");
  const auto reads = bus.reads;
  before.pc = origin;
  core.set_registers(before);
  const auto changed = core.run(1);
  before.pc += 1;
  before.x = short_datum;
  require(changed.dispatches == 1 && changed.fault.kind == FaultKind::none &&
              core.registers() == before && bus.reads == reads && core.progress().instructions == 4,
          "Accepted P write-through updates cached instructions once");
}

void cache_refusal(const Backend backend, const unsigned width) {
  Bus bus;
  bus.program(origin, {immediate_x0, datum});
  const Options options{.backend = backend,
                        .cacheable_begin = origin,
                        .cacheable_end = origin + 4,
                        .cache_sector_words = width};
  Core core(bus, options);
  auto expected = initial_state();
  expected.sr = ce;
  expected.omr = be;
  core.set_registers(expected);
  const auto result = core.run(1);
  require(result.dispatches == 0 && result.progress == Progress{} && core.registers() == expected &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::program,
                                    .address = origin + 2,
                                    .pc = origin},
          "A missing real burst-fill word faults at its own address before ISA execution or "
          "retirement");
  const auto blocked = core.run(1);
  require(blocked.dispatches == 0 && blocked.fault == result.fault &&
              core.registers() == expected && bus.reads == 3,
          "A faulted architectural cache fill remains blocked without additional source reads");
}
}

void cache_checks() {
  for (const auto backend : backends) {
    for (const unsigned width : {128U, 256U}) {
      for (const bool burst : {false, true}) { cache_case(backend, width, burst); }
      cache_refusal(backend, width);
    }
  }
}
}
