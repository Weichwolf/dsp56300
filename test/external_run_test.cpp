// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>

namespace dsp56300::test::external_run {
namespace {
constexpr Word bit_ram = 0x20, saved_program = 0x200;
constexpr Word raw_datum = 0xab123456, raw_port = 0xab000001;
constexpr Word load_y_to_x0 = 0x4cf000, movep_x0 = 0x044480;
constexpr Word set_ram = 0x0a2020, set_port = 0x010020, reset = 0x84;
constexpr Word flow_words = 12, flow_instructions = 8;
constexpr std::array pcs{
    origin, origin + 2, origin + 4, origin + 6, origin + 8, origin + 9, origin + 10, origin + 11};

Bus flow_bus() {
  Bus bus;
  bus.program(origin,
              {immediate_x0,
               raw_datum,
               store_x0,
               address,
               load_y_to_x0,
               address,
               program_store,
               saved_program,
               movep_x0,
               set_ram,
               set_port,
               reset});
  bus.memory[{Space::y, address}] = other;
  bus.memory[{Space::x, bit_ram}] = 0;
  bus.memory[{Space::x, port}] = raw_port;
  return bus;
}

constexpr std::size_t flow_event_count = 20;

std::array<Event, flow_event_count> flow_events() {
  using enum Operation;
  using enum Space;
  return {{{.operation = read, .address = origin, .pc = pcs[0]},
           {.operation = read, .address = origin + 1, .pc = pcs[0]},
           {.operation = read, .address = origin + 2, .pc = pcs[1]},
           {.operation = read, .address = origin + 3, .pc = pcs[1]},
           {.operation = write, .space = x, .address = address, .pc = pcs[1], .value = datum},
           {.operation = read, .address = origin + 4, .pc = pcs[2]},
           {.operation = read, .address = origin + 5, .pc = pcs[2]},
           {.operation = read, .space = y, .address = address, .pc = pcs[2]},
           {.operation = read, .address = origin + 6, .pc = pcs[3]},
           {.operation = read, .address = origin + 7, .pc = pcs[3]},
           {.operation = write, .address = saved_program, .pc = pcs[3], .value = other},
           {.operation = read, .address = origin + 8, .pc = pcs[4]},
           {.operation = read, .space = x, .address = port, .pc = pcs[4]},
           {.operation = read, .address = origin + 9, .pc = pcs[5]},
           {.operation = read, .space = x, .address = bit_ram, .pc = pcs[5]},
           {.operation = write, .space = x, .address = bit_ram, .pc = pcs[5], .value = 1},
           {.operation = read, .address = origin + 10, .pc = pcs[6]},
           {.operation = bit, .space = x, .address = port, .pc = pcs[6], .action = BitAction::set},
           {.operation = read, .address = origin + 11, .pc = pcs[7]},
           {.operation = reset, .pc = pcs[7]}}};
}

void successful_flow(const Backend backend) {
  auto bus = flow_bus();
  Core core(bus, {.backend = backend});
  core.set_registers(initial_state());
  const auto zero = core.run(0);
  require(zero.dispatches == 0 && zero.progress == Progress{} && bus.events.empty(),
          "A zero external instruction budget performs no logical bus operation");
  const auto result = core.run(flow_instructions);
  auto expected = initial_state();
  expected.pc += flow_words;
  expected.x = 1;
  expected.sr = flag_c;
  const auto events = flow_events();
  require(result.dispatches == flow_instructions && result.fault.kind == FaultKind::none &&
              core.registers() == expected && result.progress.instructions == flow_instructions,
          "Native P/X/Y/peripheral/bit/RESET instructions publish independent complete state");
  require(std::ranges::equal(bus.events, events) && bus.reads == 15 && bus.writes == 3 &&
              bus.bits == 1 && bus.resets == 1,
          "Literal logical access order includes every opcode, extension and real transfer exactly "
          "once");
  require(bus.memory.at({Space::x, address}) == datum &&
              bus.memory.at({Space::program, saved_program}) == other &&
              bus.memory.at({Space::x, bit_ram}) == 1,
          "Actual RAM and program stores contain only independently expected 24-bit values");
}

FaultKind kind(const Event event) {
  switch (event.operation) {
    case Operation::read:
      return event.space != Space::program && event.address >= port ? FaultKind::io_read
                                                                    : FaultKind::memory_read;
    case Operation::write:
      return event.space != Space::program && event.address >= port ? FaultKind::io_write
                                                                    : FaultKind::memory_write;
    case Operation::bit: return FaultKind::io_bit;
    case Operation::reset: return FaultKind::host_event;
    default: throw std::runtime_error("Unknown authored external-Io failure kind");
  }
}

void refused_flow(const Backend backend, const std::size_t rejected) {
  auto bus = flow_bus();
  const auto events = flow_events();
  const auto event = events.at(rejected);
  switch (event.operation) {
    case Operation::read: bus.refuse_read = Bus::Key{event.space, event.address}; break;
    case Operation::write: bus.refuse_write = Bus::Key{event.space, event.address}; break;
    case Operation::bit: bus.refuse_bit = true; break;
    case Operation::reset: bus.refuse_reset = true; break;
    default: throw std::runtime_error("Unknown authored external-Io rejection");
  }
  Core core(bus, {.backend = backend});
  core.set_registers(initial_state());
  const auto result = core.run(flow_instructions);
  const auto *const position = std::ranges::find(pcs, event.pc);
  require(position != pcs.end(), "Every rejected callback belongs to its literal instruction");
  const auto completed = static_cast<Word>(position - pcs.begin());
  const auto instruction_words = completed < 4 ? 2U : 1U;
  auto expected = initial_state();
  expected.pc = event.pc + (event.operation == Operation::read && event.space == Space::program
                                ? 0U
                                : instruction_words);
  constexpr std::array<Word, flow_instructions> previous_x{0, datum, datum, other, other, 1, 1, 1};
  expected.x = previous_x.at(completed);
  expected.sr = completed >= 7 ? flag_c : 0U;
  require(result.dispatches == completed && result.progress.instructions == completed &&
              core.registers() == expected,
          std::string("A refused opcode/extension/data/bit/reset operation has literal nonretiring "
                      "CPU prefix state, event=") +
              std::to_string(rejected));
  const auto fault_address = event.operation == Operation::reset ? event.pc : event.address;
  require(result.fault.kind == kind(event) && result.fault.space == event.space &&
              result.fault.address == fault_address && result.fault.pc == event.pc,
          "External refusals retain actual operation class, address, space and instruction PC");
  require(std::ranges::equal(bus.events, std::span(events).first(rejected + 1)),
          "A real refusal preserves the accepted physical prefix and prevents all later callbacks");
  const auto saved = core.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, {.backend = backend});
  clone.restore(saved);
  for (auto *owner : {&core, &clone}) {
    const auto stopped = owner->run(flow_instructions);
    require(stopped.dispatches == 0 && stopped.fault == result.fault &&
                owner->registers() == expected && owner->progress() == result.progress,
            "Faulted original/fresh diagnostic owners remain blocked without replay or retirement");
  }
  require(bus.events.size() == rejected + 1 && clone_bus.events == bus.events,
          "Diagnostic restore never turns a failed physical operation into a synthetic success");
}
}
}

int main() {
  try {
    using namespace dsp56300::test::external_run;
    for (const auto backend : backends) {
      successful_flow(backend);
      for (std::size_t index = 0; index < flow_events().size(); ++index) {
        refused_flow(backend, index);
      }
      for (const auto sample : workloads) {
        static_cast<void>(workload(sample, backend, false, false));
        static_cast<void>(workload(sample, backend, false, true));
        static_cast<void>(workload(sample, backend, true, true));
        static_cast<void>(captured_workload(sample, backend));
      }
    }
    cache_checks();
    continuation_checks();
    std::cout << dsp56300::test::checks << " external-memory execution assertions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
