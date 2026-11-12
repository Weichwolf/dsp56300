// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word start = 16, destination = 48, vector = 32;
constexpr std::array loader{op<isa::Opcode::movem_absolute_short> | (destination << 8) | 4U,
                            op<isa::Opcode::jmp_absolute_short> | destination};
constexpr std::array loaded{op<isa::Opcode::stop>};
constexpr std::array inserted{op<isa::Opcode::nop>, op<isa::Opcode::nop>};
constexpr CodeTransition transfer{.from = 0, .to = 1, .entry = destination};

NativeImage compile(const std::span<const CodeTransition> transitions,
                    const bool indirect = false,
                    const bool alternative = false) {
  auto code = loader;
  if (indirect) { code.back() = op<isa::Opcode::jmp_effective> | (4U << 11); }
  const std::array first{CodeSpan{.address = start, .words = code}};
  const std::array second{CodeSpan{.address = destination, .words = loaded},
                          CodeSpan{.address = vector, .words = inserted}};
  constexpr std::array alternate_body{op<isa::Opcode::nop>, op<isa::Opcode::stop>};
  const std::array alternate{CodeSpan{.address = destination,
                                      .words = alternative ? std::span<const Word>(alternate_body)
                                                           : std::span<const Word>(loaded)}};
  constexpr std::array boot_entry{start};
  constexpr std::array runtime_entry{destination};
  const std::array variants{
      CodeVariant{.id = 0, .code = first, .entries = boot_entry, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = second, .entries = runtime_entry, .failed_fetches = {}},
      CodeVariant{.id = 2, .code = alternate, .entries = runtime_entry, .failed_fetches = {}}};
  constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
  return Core::compile({.variants = variants},
                       {.memory = {},
                        .interrupts = interrupts,
                        .transitions = transitions,
                        .callbacks = {},
                        .bindings = {}});
}

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.failure == right.failure &&
         left.execution == right.execution && left.progress == right.progress;
}

struct Machine {
  explicit Machine(const NativeImage &image, const bool indirect = false)
      : core(Memory::separate({.program = program, .x = x, .y = y})) {
    std::ranges::copy(loader, program.begin() + start);
    if (indirect) { program[start + 1] = op<isa::Opcode::jmp_effective> | (4U << 11); }
    initial = test::register_model::seed(0);
    initial.pc = start;
    initial.sr = 0;
    initial.x = (initial.x & ~std::uint64_t{word_mask}) | loaded[0];
    initial.r[0] = destination + 1;
    core.set_registers(initial);
    core.install(image, 0);
  }

  std::array<Word, 128> program{};
  std::array<Word, 1> x{}, y{};
  Registers initial;
  Core core;
};

void check_boundary(Machine &machine) {
  auto expected = machine.initial;
  expected.pc = destination;
  const auto actual = machine.core.state();
  require(actual.registers == expected && actual.failure == Fault{} &&
              actual.execution == ExecutionState::running &&
              actual.progress == Progress{.instructions = 2, .estimated_cycles = 9} &&
              machine.program[destination] == loaded[0],
          "The real loader store and jump finish exactly once before the prepared target binds");
}

void deadlines(const NativeImage &image) {
  for (std::uint64_t deadline = 0; deadline <= 12; ++deadline) {
    Machine machine(image);
    const auto result = machine.core.run_until(deadline);
    const auto count =
        static_cast<std::uint32_t>(deadline != 0) + static_cast<std::uint32_t>(deadline > 6);
    constexpr std::array<std::uint64_t, 3> elapsed{0, 6, 9};
    const auto cycles = elapsed[count];
    require(result.retired == count && result.fault == Fault{} && result.yielded == (count == 2) &&
                result.progress == Progress{.instructions = count, .estimated_cycles = cycles},
            "Exact and overshooting deadlines retain the real transition boundary");
    if (count == 2) {
      check_boundary(machine);
      machine.core.install(image, 1);
      require(machine.core.run(1).retired == 1, "A timed transfer can resume without replay");
    }
  }
}

void unknown_target() {
  const auto image = compile(std::span(&transfer, 1), true);
  Machine machine(image, true);
  const auto result = machine.core.run();
  require(result.retired == 2 && !result.yielded &&
              result.fault == Fault{.kind = FaultKind::unprepared_execution,
                                    .space = Space::program,
                                    .address = destination + 1,
                                    .value = 0,
                                    .pc = destination + 1} &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 9},
          "An undeclared dynamic target cannot become a successful image transition");
}

void prefixes(const NativeImage &image) {
  for (std::uint32_t count = 0; count <= 4; ++count) {
    Machine machine(image);
    const auto result = machine.core.run(count);
    require(result.retired == std::min(count, 2U) && result.fault == Fault{},
            "A prepared transfer preserves its actual instruction quota prefix");
    if (count < 2) { continue; }
    check_boundary(machine);
    const auto boundary = machine.core.state();
    const auto pending = machine.core.run();
    require(pending.retired == 0 && pending.yielded && pending.fault == Fault{} &&
                same(boundary, machine.core.state()),
            "A declined transfer waits without faulting, replaying the jump or advancing time");
    machine.core.install(image, 1);
    require(same(boundary, machine.core.state()), "Installation preserves the completed transfer");
    const auto done = machine.core.run();
    auto expected = machine.initial;
    expected.pc = destination + 1;
    constexpr std::uint64_t cycles = 9 + 10;
    require(done.retired == 1 && done.fault == Fault{} && done.state == ExecutionState::stopped &&
                machine.core.registers() == expected &&
                done.progress == Progress{.instructions = 3, .estimated_cycles = cycles},
            "Only the actual loaded STOP executes after selecting the declared destination");
  }
}

void failed_install(const NativeImage &image) {
  Machine machine(image);
  require(machine.core.run().yielded, "The loader exposes the cross-variant owner boundary");
  check_boundary(machine);
  const auto boundary = machine.core.state();
  bool rejected = false;
  try {
    machine.core.install(image, 2);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(boundary, machine.core.state()),
          "Identical destination PCs and words cannot select an undeclared target variant");
  machine.core.load(Space::program, destination, std::array{op<isa::Opcode::nop>});
  rejected = false;
  try {
    machine.core.install(image, 1);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(boundary, machine.core.state()) && machine.core.run().retired == 0,
          "Missing resident source refuses installation transactionally without replay");
  machine.core.load(Space::program, destination, loaded);
  machine.core.install(image, 1);
  require(machine.core.run().retired == 1,
          "A corrected resident binding can finish the same transfer");
}

void interrupts(const NativeImage &image) {
  Machine machine(image);
  static_cast<void>(machine.core.run());
  const auto boundary = machine.core.state();
  require(!machine.core.interrupt_ready() &&
              !machine.core.interrupt_ready({.address = destination, .opcode = loaded[0]}),
          "Unbound transfer targets cannot acknowledge an interrupt");
  bool rejected = false;
  try {
    machine.core.request_interrupt(vector, 0);
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && same(boundary, machine.core.state()), "IRQ refusal preserves the transfer");
  rejected = false;
  try {
    machine.core.set_registers(machine.initial);
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && same(boundary, machine.core.state()),
          "Register initialization cannot clear a transfer");
  rejected = false;
  try {
    static_cast<void>(machine.core.run_instruction({.address = destination, .opcode = loaded[0]}));
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && same(boundary, machine.core.state()),
          "Captured execution cannot bypass a pending image installation");
  machine.core.install(image, 1);
  machine.core.request_interrupt(vector, 0);
  const auto entered = machine.core.run(2);
  require(entered.retired == 2 && entered.fault == Fault{} &&
              machine.core.registers() == boundary.registers &&
              entered.progress == Progress{.instructions = 4, .estimated_cycles = 11},
          "The runtime-only prepared vector executes after exact resident installation");
}

void reset_transfer(const NativeImage &image) {
  Machine machine(image);
  require(machine.core.run().yielded, "A real transfer precedes the explicit reset");
  machine.core.reset();
  machine.core.set_registers(machine.initial);
  machine.core.install(image, 0);
  require(machine.core.progress() == Progress{} && machine.core.run().retired == 2,
          "Only explicit reset cancels the pending transfer and restarts the loader");
  check_boundary(machine);
}

void initial_target(const NativeImage &image) {
  Machine machine(image);
  auto registers = machine.initial;
  registers.pc = destination;
  machine.core.set_registers(registers);
  const auto result = machine.core.run();
  require(result.retired == 0 && !result.yielded &&
              result.fault.kind == FaultKind::unprepared_execution &&
              result.fault.address == destination && machine.core.progress() == Progress{},
          "An unbound initial PC cannot invent a completed code-version transfer");
}

void multiple_targets() {
  constexpr std::uint64_t prefix_cycles = 9;
  constexpr std::uint64_t stop_cycles = 10;
  constexpr std::array targets{transfer, CodeTransition{.from = 0, .to = 2, .entry = destination}};
  const auto image = compile(targets, false, true);
  const auto other_bank = compile(targets, true, true);
  for (const std::uint64_t target : {1U, 2U}) {
    Machine machine(image);
    const Word payload = target == 1 ? op<isa::Opcode::stop> : op<isa::Opcode::nop>;
    machine.initial.x = (machine.initial.x & ~std::uint64_t{word_mask}) | payload;
    machine.program[destination + 1] = op<isa::Opcode::stop>;
    machine.core.set_registers(machine.initial);
    const auto prefix = machine.core.run();
    auto expected = machine.initial;
    expected.pc = destination;
    require(prefix.retired == 2 && prefix.yielded && prefix.fault == Fault{} &&
                machine.core.registers() == expected && machine.program[destination] == payload &&
                prefix.progress == Progress{.instructions = 2, .estimated_cycles = prefix_cycles},
            "A real store/jump reaches one shared destination without selecting a code version");
    const auto boundary = machine.core.state();
    const std::uint64_t mismatched = target == 1 ? 2 : 1;
    for (const auto wrong : {std::uint64_t{0}, mismatched}) {
      bool rejected = false;
      try {
        machine.core.install(image, wrong);
      } catch (const std::invalid_argument &) { rejected = true; }
      const auto waiting = machine.core.run();
      require(rejected && waiting.retired == 0 && waiting.yielded &&
                  same(boundary, machine.core.state()),
              "Undeclared or byte-mismatched targets preserve the pending native transfer");
    }
    bool foreign = false;
    try {
      machine.core.install(other_bank, target);
    } catch (const std::invalid_argument &) { foreign = true; }
    require(foreign && same(boundary, machine.core.state()),
            "Equal destination bytes in another native bank cannot steal a pending transfer");
    machine.core.install(image, target);
    require(same(boundary, machine.core.state()), "Selecting a valid payload changes no ISA state");
    const auto done = machine.core.run();
    expected.pc = destination + static_cast<Word>(target);
    require(done.retired == target && done.fault == Fault{} && !done.yielded &&
                done.state == ExecutionState::stopped && machine.core.registers() == expected &&
                done.progress ==
                    Progress{.instructions = 2 + target,
                             .estimated_cycles = prefix_cycles + stop_cycles + target - 1},
            "Both prepared native payloads execute their distinct words once without replay");
  }
}

void invalid_contracts() {
  const auto rejected = [](const std::span<const CodeTransition> declarations) {
    bool refused = false;
    try {
      static_cast<void>(compile(declarations));
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused, "Absent, unknown, duplicate or non-entry transitions fail cold preparation");
  };
  rejected({});
  for (const auto declaration : {CodeTransition{.from = 7, .to = 1, .entry = destination},
                                 CodeTransition{.from = 0, .to = 7, .entry = destination},
                                 CodeTransition{.from = 0, .to = 0, .entry = start},
                                 CodeTransition{.from = 0, .to = 1, .entry = destination + 1}}) {
    rejected(std::span(&declaration, 1));
  }
  const std::array duplicate{transfer, transfer};
  rejected(duplicate);
}
}

int main() {
  try {
    multiple_targets();
    invalid_contracts();
    const auto image = compile(std::span(&transfer, 1));
    prefixes(image);
    deadlines(image);
    failed_install(image);
    interrupts(image);
    reset_transfer(image);
    initial_target(image);
    unknown_target();
    std::cout << test::checks << " public prepared code-transition checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
