// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "executable_mappings.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string_view>
#include <unistd.h>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | (5U << 4);
constexpr Word add = isa::opcode<isa::Opcode::add_x0_a>()[0];
constexpr Word clear = isa::opcode<isa::Opcode::clr_a>()[0];
constexpr std::array code{mac, add, mac, clear};

struct Machine {
  Machine() : core(Memory::separate({.program = p, .x = x, .y = y})) {
    core.load(Space::program, start, code);
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct CodeImage {
  ExecutableRange range;
  std::vector<std::byte> bytes;
};

std::vector<CodeImage> image() {
  std::vector<CodeImage> result;
  const auto mappings = jit_executable_mappings();
  for (const auto publication : codegen::published_images()) {
    const auto first = std::bit_cast<std::uintptr_t>(publication.entry);
    const auto last = first + publication.bytes;
    const bool live = std::ranges::any_of(
        mappings, [&](const auto range) { return range.first <= first && range.last >= last; });
    require(live, "Every publication of the live Core remains mapped read/execute");
    const auto bytes = std::as_bytes(std::span(publication.entry, publication.bytes));
    result.push_back({.range = {.first = first, .last = last},
                      .bytes = std::vector<std::byte>(bytes.begin(), bytes.end())});
  }
  require(!result.empty(), "A real native translation has executable storage");
  return result;
}

void preserved(const std::span<const CodeImage> original) {
  const auto mappings = jit_executable_mappings();
  for (const auto &part : original) {
    const bool readable = std::ranges::any_of(mappings, [&](const auto range) {
      return range.first <= part.range.first && range.last >= part.range.last;
    });
    require(readable, "Every earlier executable address range remains mapped read/execute");
    const auto *const first = std::bit_cast<const std::byte *>(part.range.first);
    const std::span<const std::byte> current(first, part.bytes.size());
    require(std::ranges::equal(current, part.bytes),
            "Every byte of each published native image remains immutable");
  }
}

void run(Machine &machine, const Word budget) {
  const auto initial = register_model::seed(0);
  auto expected = initial;
  const auto memory = std::array{machine.p, machine.x, machine.y};
  for (Word index = 0; index < budget; ++index) {
    const auto old = expected;
    sequence_model::alu(expected, old, machine.p[start + index]);
    ++expected.pc;
  }
  machine.core.set_registers(initial);
  const auto before = machine.core.state().progress;
  const auto actual = machine.core.run(budget);
  require(actual.retired == budget && actual.fault.kind == FaultKind::none && !actual.yielded &&
              actual.state == ExecutionState::running && machine.core.registers() == expected &&
              actual.progress == Progress{.instructions = before.instructions + budget,
                                          .estimated_cycles = before.estimated_cycles + budget},
          "Retained versions execute independent arithmetic, all registers and exact base cycles");
  require(std::array{machine.p, machine.x, machine.y} == memory,
          "Guest execution leaves every program/X/Y word unchanged");
}

void reset() {
  Machine machine;
  run(machine, 3);
  const auto original = image();
  const auto memory = std::array{machine.p, machine.x, machine.y};
  machine.core.reset();
  Registers expected;
  expected.m.fill(word_mask);
  require(machine.core.registers() == expected && machine.core.state().progress == Progress{} &&
              machine.core.state().failure.kind == FaultKind::none &&
              std::array{machine.p, machine.x, machine.y} == memory,
          "Architectural reset clears CPU progress/state while preserving all borrowed memory");
  preserved(original);
  run(machine, 3);
  preserved(original);
}

void program_write() {
  Machine machine;
  run(machine, 1);
  const auto original = image();
  machine.core.load(Space::program, start, std::span(&clear, 1));
  preserved(original);
  run(machine, 1);
  preserved(original);
  const auto second = image();
  machine.core.load(Space::program, start, std::span(&mac, 1));
  run(machine, 1);
  preserved(second);
}

void larger_grant() {
  Machine machine;
  run(machine, 1);
  const auto first = image();
  run(machine, 3);
  preserved(first);
  const auto second = image();
  run(machine, 4);
  preserved(second);
}

void shared_run(Machine &machine, const Word budget) {
  constexpr Word ending = 2;
  const auto initial = register_model::seed(0);
  auto expected = initial;
  Word cycles = 0;
  const auto memory = std::array{machine.p, machine.x, machine.y};
  for (Word index = 0; index < budget; ++index) {
    if (expected.pc == start + ending) {
      expected.pc = start;
      cycles += 3;
    } else {
      const auto old = expected;
      sequence_model::alu(expected, old, machine.p[expected.pc]);
      ++expected.pc;
      ++cycles;
    }
  }
  machine.core.set_registers(initial);
  const auto before = machine.core.state().progress;
  const auto result = machine.core.run(budget);
  require(result.retired == budget && result.fault.kind == FaultKind::none && !result.yielded &&
              result.state == ExecutionState::running && machine.core.registers() == expected &&
              result.progress == Progress{.instructions = before.instructions + budget,
                                          .estimated_cycles = before.estimated_cycles + cycles} &&
              std::array{machine.p, machine.x, machine.y} == memory,
          "Both shared entries preserve complete independent arithmetic, branch, RAM and clocks");
}

void shared_entries() {
  const auto original_bytes = jit_executable_bytes();
  const auto page = sysconf(_SC_PAGESIZE);
  require(page > 0, "Shared native storage needs the actual system page size");
  const auto initial_publications = codegen::publications();
  Machine machine;
  constexpr Word ending = 2;
  constexpr Word jump = isa::opcode<isa::Opcode::jmp_absolute_short>()[0];
  machine.p[start + ending] = jump | start;
  shared_run(machine, 1);
  const auto published = codegen::publications();
  const auto bytes = published.bytes - initial_publications.bytes;
  const auto reserved = jit_executable_bytes() - original_bytes;
  require(published.count == initial_publications.count + 1 && bytes > 0 &&
              bytes <= static_cast<std::uint64_t>(page) && reserved >= bytes &&
              reserved % static_cast<std::uint64_t>(page) == 0,
          "Two small native entries share one compact published image inside an RX arena");
  const auto original = image();
  for (const Word budget : {2U, 3U, 4U, 5U, 6U, 7U, 9U, 10U, 11U, 1U}) {
    shared_run(machine, budget);
    require(codegen::publications() == published &&
                jit_executable_bytes() == original_bytes + reserved,
            "Changing entry paths neither maps new pages nor duplicates code");
    preserved(original);
  }
}

void captured_literals() {
  Machine machine;
  std::vector<CodeImage> original;
  for (const Word literal : {1U, 2U, 3U}) {
    const auto initial = register_model::seed(0);
    machine.core.set_registers(initial);
    auto expected = initial;
    // An eight-bit data immediate occupies the high byte of the fractional word.
    constexpr unsigned fraction_shift = word_bits - 8;
    expected.x =
        (initial.x & ~std::uint64_t{word_mask}) | (std::uint64_t{literal} << fraction_shift);
    ++expected.pc;
    const auto before = machine.core.state().progress;
    const auto opcode = isa::opcode<isa::Opcode::move_immediate_x0>()[0] | (literal << 8);
    const auto actual = machine.core.run_instruction({.address = initial.pc, .opcode = opcode});
    require(actual.retired == 1 && actual.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                actual.progress == Progress{.instructions = before.instructions + 1,
                                            .estimated_cycles = before.estimated_cycles + 1},
            "Changed captured literals execute their real values and preserve complete state");
    preserved(original);
    original = image();
  }
}

void repeat_versions() {
  Machine machine;
  constexpr Word count = 3;
  constexpr Word setup = isa::opcode<isa::Opcode::rep_immediate>()[0] | (count << 8);
  std::vector<CodeImage> original;
  for (const Word opcode : {mac, add}) {
    const auto initial = register_model::seed(0);
    machine.core.set_registers(initial);
    const auto before = machine.core.state().progress;
    const auto entered = machine.core.run_instruction({.address = initial.pc, .opcode = setup});
    auto expected = initial;
    ++expected.pc;
    expected.lc = count;
    require(entered.retired == 1 && entered.fault.kind == FaultKind::none &&
                machine.core.registers() == expected,
            "Actual captured REP establishes the independently expected loop count and state");
    for (Word index = 0; index < count; ++index) {
      const auto actual =
          machine.core.run_instruction({.address = initial.pc + 1, .opcode = opcode});
      const auto old = expected;
      sequence_model::alu(expected, old, opcode);
      expected.pc = initial.pc + (index == count - 1 ? 2U : 1U);
      expected.lc = index == count - 1 ? initial.lc : count - index - 1;
      require(actual.retired == 1 && actual.fault.kind == FaultKind::none &&
                  machine.core.registers() == expected &&
                  actual.progress ==
                      Progress{.instructions = before.instructions + 2 + index,
                               .estimated_cycles = before.estimated_cycles + 6 + index},
              "Every real REP version keeps exact arithmetic, restored LC and original base costs");
    }
    preserved(original);
    original = image();
  }
}

struct Scenario {
  std::string_view name;
  void (*execute)();
};
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    const auto original = jit_executable_mappings();
    constexpr std::array scenarios{
        Scenario{.name = "reset", .execute = reset},
        Scenario{.name = "program write", .execute = program_write},
        Scenario{.name = "larger grant", .execute = larger_grant},
        Scenario{.name = "shared entries", .execute = shared_entries},
        Scenario{.name = "captured literals", .execute = captured_literals},
        Scenario{.name = "REP versions", .execute = repeat_versions}};
    unsigned failures = 0;
    for (const auto scenario : scenarios) {
      codegen::track_images();
      try {
        scenario.execute();
        require(jit_executable_mappings() == original,
                "Core destruction releases every owned executable mapping");
      } catch (const std::exception &error) {
        std::cerr << scenario.name << ": " << error.what() << '\n';
        ++failures;
      }
    }
    std::cout << scenarios.size() - failures << '/' << scenarios.size()
              << " native code-lifetime scenarios; " << checks << " checks\n";
    return failures == 0 ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
