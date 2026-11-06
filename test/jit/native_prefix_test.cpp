// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word length = 64, blocks = 40;
constexpr std::size_t words = 4096;
constexpr std::uint64_t cache_bytes = std::uint64_t{8} * 1024 * 1024;
constexpr Word mac = 0x2000d2, add = 0x200040, bit_xor = 0x200043, complement = 0x200017;
constexpr std::array operations{mac, add, bit_xor, complement};

struct Machine {
  Machine() : core({.program = p, .x = x, .y = y}) {}

  std::array<Word, words> p{}, x{}, y{};
  Core core;
};

Registers prefix(const Registers &initial, const Word count) {
  auto result = initial;
  for (Word index = 0; index < count; ++index) {
    const auto operation = operations[index % operations.size()];
    const auto old = result;
    if (operation == bit_xor || operation == complement) {
      bfu_model::apply(result,
                       old,
                       {.function = operation == bit_xor ? bfu_model::Function::bit_xor
                                                         : bfu_model::Function::complement});
    } else {
      sequence_model::alu(result, old, operation);
    }
    ++result.pc;
  }
  return result;
}

void shorter_warm_entries() {
  Machine machine;
  for (Word index = 0; index < length; ++index) {
    machine.p[start + index] = operations[index % operations.size()];
  }
  const auto initial = register_model::seed(0);
  machine.core.set_registers(initial);
  require(machine.core.run(length).dispatches == length, "Warm the actual complete native block");
  for (unsigned pass = 0; pass < 2; ++pass) {
    const auto previous_blocks = machine.core.jit_statistics().compiled_blocks;
    for (Word budget = length - 1; budget != 0; --budget) {
      machine.core.set_registers(initial);
      const auto before = machine.core.jit_statistics();
      const auto progress = machine.core.progress();
      const auto run = machine.core.run(budget);
      const auto after = machine.core.jit_statistics();
      require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                  machine.core.registers() == prefix(initial, budget) &&
                  run.progress == Progress{.instructions = progress.instructions + budget,
                                           .estimated_cycles = progress.estimated_cycles + budget},
              "Every warm short prefix exposes all independent arithmetic/BFU flags and registers");
      require(after.native_alu_instructions == before.native_alu_instructions + budget &&
                  after.helper_instructions == before.helper_instructions,
              "A short cached entry must execute actual native code without an interpreter tail");
    }
    if (pass != 0) {
      require(machine.core.jit_statistics().compiled_blocks == previous_blocks,
              "Previously compiled prefixes are reused instead of allocating/recompiling each run");
    }
  }
}

void bounded_storage() {
  Machine machine;
  bool evicted = false;
  std::uint64_t previous_bytes = 0;
  for (Word block = 0; block < blocks; ++block) {
    auto initial = register_model::seed(0);
    initial.pc = start + block * length;
    for (Word budget = length; budget != 0; --budget) {
      machine.core.set_registers(initial);
      auto expected = initial;
      expected.pc += budget;
      const auto before = machine.core.jit_statistics();
      const auto run = machine.core.run(budget);
      const auto after = machine.core.jit_statistics();
      evicted = evicted || after.executable_bytes < previous_bytes;
      previous_bytes = after.executable_bytes;
      require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                  machine.core.registers() == expected && after.executable_bytes <= cache_bytes &&
                  after.native_alu_instructions == before.native_alu_instructions + budget &&
                  after.helper_instructions == 0,
              "Parent/prefix eviction preserves genuine state and the documented executable bound");
    }
  }
  require(
      evicted && machine.core.jit_statistics().compiled_blocks > cache_bytes / words,
      "The test crosses real executable capacity and observes eviction while retaining a parent");
}
}
}

int main() {
  try {
    dsp56300::test::shorter_warm_entries();
    dsp56300::test::bounded_storage();
    std::cout << dsp56300::test::checks << " native short-prefix/cache-bound checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
