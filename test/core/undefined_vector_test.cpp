// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word start = 0x200, trapping = 0x220, vba = 0x800;
constexpr Word illegal_vector = vba + 4, trap_vector = vba + 8;
constexpr Word undefined = 0x000143; // Authored member of an audited reserved domain.
constexpr std::array body{undefined, op<isa::Opcode::nop>, op<isa::Opcode::stop>};
constexpr std::array trap_body{op<isa::Opcode::trap>, op<isa::Opcode::nop>, op<isa::Opcode::stop>};
constexpr std::array first{undefined, op<isa::Opcode::nop>};
constexpr std::array second{op<isa::Opcode::nop>, undefined};
constexpr std::array clean{op<isa::Opcode::nop>, op<isa::Opcode::nop>};
constexpr std::array vectors{first, second, clean};

std::array<CodeSpan, 4> streams(const unsigned variant) {
  return {CodeSpan{.address = start, .words = body},
          CodeSpan{.address = trapping, .words = trap_body},
          CodeSpan{.address = illegal_vector, .words = vectors.at(variant)},
          CodeSpan{.address = trap_vector, .words = first}};
}

const NativeImage &image() {
  static const NativeImage result = [] {
    const std::array code{streams(0), streams(1), streams(2)};
    constexpr std::array entries{start, trapping};
    const std::array versions{
        CodeVariant{.id = 0, .code = code[0], .entries = entries, .failed_fetches = {}},
        CodeVariant{.id = 1, .code = code[1], .entries = entries, .failed_fetches = {}},
        CodeVariant{.id = 2, .code = code[2], .entries = entries, .failed_fetches = {}}};
    constexpr std::array requests{InterruptRequest{.vector = illegal_vector, .level = 3},
                                  InterruptRequest{.vector = trap_vector, .level = 3}};
    return Core::compile(
        {.variants = versions},
        {.memory = {}, .interrupts = requests, .transitions = {}, .callbacks = {}, .bindings = {}});
  }();
  return result;
}

struct Machine {
  explicit Machine(const bool installed)
      : core(Memory::separate({.program = program, .x = x, .y = y})), prepared(installed) {}

  void reset(const unsigned variant, const Registers &initial) {
    core.reset();
    for (const auto span : streams(variant)) {
      core.load(Space::program, span.address, span.words);
    }
    core.set_registers(initial);
    if (prepared) { core.install(image(), variant); }
  }

  void run(const Registers &expected, const unsigned count = 1) {
    const auto result = core.run(count);
    require(result.retired == count && result.fault == Fault{} && core.registers() == expected,
            "Nested fast-vector III preserves all registers and caller stack without returning");
  }

  void fetch(const Word expected) const {
    const auto next = core.instruction_fetch();
    require(next.exception_vector && next.address == expected,
            "Undefined vector slots request the actual III vector instead of the foreground");
  }

  std::array<Word, 4096> program{};
  std::array<Word, 8> x{}, y{};
  Core core;
  bool prepared;
};

void recover(Machine &machine, const Registers &expected) {
  const auto progress = machine.core.progress();
  machine.core.load(Space::program, illegal_vector, clean);
  if (machine.prepared) { machine.core.install(image(), 2); }
  require(machine.core.registers() == expected && machine.core.progress() == progress,
          "Selecting already prepared vector contents performs no guest execution or restoration");
  machine.run(expected, 2);
  require(!machine.core.instruction_fetch().exception_vector,
          "A newly fetched two-NOP vector returns to the retained original continuation");
  auto continued = expected;
  ++continued.pc;
  machine.run(continued);
  ++continued.pc;
  machine.run(continued);
  require(machine.core.state().execution == ExecutionState::stopped,
          "The unchanged original guest continuation reaches STOP after vector recovery");
}

struct Input {
  unsigned slot;
  bool trap;
};

void contexts(Machine &machine, const Input input) {
  for (const Word mask : {0U, 0x100U, 0x200U, 0x300U}) {
    for (const Word depth : {0U, 2U}) {
      auto initial = test::register_model::seed(0);
      initial.pc = input.trap ? trapping : start;
      initial.vba = vba;
      constexpr Word condition_codes = 0xd5;
      initial.sr = mask | condition_codes;
      initial.sp = depth;
      initial.sc = depth;
      machine.reset(input.slot, initial);
      ++initial.pc;
      machine.run(initial);
      if (input.trap) {
        machine.fetch(trap_vector);
        machine.run(initial);
      }
      for (unsigned attempt = 0; attempt < 3; ++attempt) {
        machine.fetch(illegal_vector);
        machine.run(initial);
        machine.fetch(illegal_vector + input.slot);
        if (input.slot != 0) { machine.run(initial); }
      }
      recover(machine, initial);
    }
  }
}

void nested() {
  for (const bool prepared : {true, false}) {
    Machine machine(prepared);
    for (const unsigned slot : {0U, 1U}) {
      for (const bool trap : {false, true}) { contexts(machine, {.slot = slot, .trap = trap}); }
    }
  }
}
}

int main() try {
  nested();
  std::cout << test::checks << " nested undefined-vector checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
