// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "executable_mappings.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <ranges>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word length = 64, blocks = 32;
constexpr Word jump = isa::opcode<isa::Opcode::jmp_absolute_short>()[0];
constexpr std::size_t words = 4096;
constexpr std::uint64_t former_cache_bytes = std::uint64_t{8} * 1024 * 1024;
constexpr Word mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | (5U << 4);
constexpr Word add = isa::opcode<isa::Opcode::add_x0_a>()[0];
constexpr Word bit_xor = isa::opcode<isa::Opcode::eor_x0_a>()[0];
constexpr Word complement = isa::opcode<isa::Opcode::not_a>()[0];
constexpr std::array operations{mac, add, bit_xor, complement};

struct Machine {
  Machine() : core(Memory::separate({.program = p, .x = x, .y = y})) {}

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

struct GrantInput {
  Word mode = 0, flags = 0;
  bool descending = false;
};

void grant_case(const GrantInput input) {
  Machine machine;
  for (Word index = 0; index < length; ++index) {
    machine.p[start + index] = operations[index % operations.size()];
  }
  machine.p[start + length] = jump | start;
  auto initial = register_model::seed(input.mode);
  initial.sr = input.mode | input.flags;
  machine.core.set_registers(initial);
  const auto warm = input.descending ? length : 1U;
  require(machine.core.run(warm).retired == warm, "Compile the logical block from either grant");
  const auto compiled = codegen::publications();
  for (Word index = 1; index <= length; ++index) {
    const auto budget = input.descending ? length + 1 - index : index;
    machine.core.set_registers(initial);
    const auto progress = machine.core.state().progress;
    const auto run = machine.core.run(budget);
    const auto after = codegen::publications();
    require(run.retired == budget && run.fault.kind == FaultKind::none && !run.yielded &&
                run.state == ExecutionState::running &&
                machine.core.registers() == prefix(initial, budget) &&
                run.progress == Progress{.instructions = progress.instructions + budget,
                                         .estimated_cycles = progress.estimated_cycles + budget},
            "Every short grant exposes independent arithmetic/BFU flags and complete registers");
    require(after == compiled,
            "Growing and shrinking grants never compile fragments or replace the logical block");
  }
}

void shorter_warm_entries() {
  constexpr std::array modes{
      0U, mode_down, mode_up, mode_rm, mode_down | mode_rm, mode_up | mode_rm};
  for (const auto mode : modes) {
    for (const auto flags : {0U, status_byte}) {
      grant_case({.mode = mode, .flags = flags});
      grant_case({.mode = mode, .flags = flags, .descending = true});
    }
  }
}

struct PrefixInput {
  Word address = 0;
  Word mode = 0;
};

void nop_prefix(Machine &machine, const PrefixInput input, const std::array<Word, words> &program) {
  constexpr Word budget = 1;
  auto initial = register_model::seed(input.mode);
  initial.pc = input.address;
  auto expected = initial;
  expected.pc += budget;
  machine.core.set_registers(initial);
  const auto before = codegen::publications();
  const auto progress = machine.core.state().progress;
  const auto run = machine.core.run(budget);
  const auto after = codegen::publications();
  constexpr std::array<Word, words> unchanged{};
  require(
      run.retired == budget && run.fault.kind == FaultKind::none && !run.yielded &&
          run.state == ExecutionState::running && machine.core.registers() == expected &&
          run.progress == Progress{.instructions = progress.instructions + budget,
                                   .estimated_cycles = progress.estimated_cycles + budget} &&
          machine.p == program && machine.x == unchanged && machine.y == unchanged,
      "Every retained prefix preserves independent complete registers, all RAM and base cycles");
  require(after.bytes >= before.bytes, "Every published prefix retains earlier executable storage");
}

void captured_repeat_entries(Machine &machine) {
  constexpr Word count = 4;
  constexpr Word repeat = isa::opcode<isa::Opcode::rep_immediate>()[0] | (count << 8);
  const auto before = codegen::publications();
  const auto progress = machine.core.state().progress;
  auto initial = register_model::seed(0);
  machine.core.set_registers(initial);
  const auto captured = machine.core.run_instruction({.address = initial.pc, .opcode = mac});
  require(captured.retired == 1 && captured.fault.kind == FaultKind::none &&
              machine.core.registers() == prefix(initial, 1),
          "Captured native work keeps its own opcode and exact arithmetic above the old capacity");
  initial.pc = start + (blocks * length) + 1;
  machine.core.set_registers(initial);
  auto expected = initial;
  ++expected.pc;
  expected.lc = count;
  const auto setup = machine.core.run_instruction({.address = initial.pc, .opcode = repeat});
  require(setup.retired == 1 && setup.fault.kind == FaultKind::none &&
              machine.core.registers() == expected,
          "Captured REP setup retains original registers, count and instruction ownership");
  for (Word iteration = 0; iteration < count; ++iteration) {
    const auto body = machine.core.run_instruction({.address = initial.pc + 1, .opcode = mac});
    const auto old = expected;
    sequence_model::alu(expected, old, mac);
    expected.pc = initial.pc + (iteration == count - 1 ? 2U : 1U);
    expected.lc = iteration == count - 1 ? initial.lc : count - iteration - 1;
    require(body.retired == 1 && body.fault.kind == FaultKind::none && !body.yielded &&
                body.state == ExecutionState::running && machine.core.registers() == expected &&
                body.progress ==
                    Progress{.instructions = progress.instructions + 3 + iteration,
                             .estimated_cycles = progress.estimated_cycles + 7 + iteration},
            "Repeated captures preserve independent arithmetic, hidden REP completion and cycles");
  }
  const auto after = codegen::publications();
  require(after.bytes > before.bytes,
          "Captured and REP entries publish real code without evicting earlier translations");
}

Word numeric_mode(const unsigned context) {
  constexpr std::array<unsigned, 7> positions{10, 11, 13, 14, 17, 20, 21};
  Word result = 0;
  for (unsigned bit = 0; bit < positions.size(); ++bit) {
    result |= ((context >> bit) & 1U) << positions[bit];
  }
  return result;
}

void retained_storage() {
  const auto original_mappings = anonymous_executable_bytes();
  const auto initial_publications = codegen::publications();
  {
    Machine machine;
    for (Word block = 0; block < blocks; ++block) {
      const auto address = start + (3 * block);
      machine.p[address + 1] = jump | address;
    }
    const auto program = machine.p;
    std::vector<PrefixInput> entries;
    constexpr unsigned contexts = 1U << 7;
    for (unsigned context = 0; context < contexts; ++context) {
      for (Word block = 0; block < blocks; ++block) {
        const PrefixInput input{.address = start + (3 * block), .mode = numeric_mode(context)};
        nop_prefix(machine, input, program);
        entries.push_back(input);
      }
      if (anonymous_executable_bytes() - original_mappings > former_cache_bytes) { break; }
    }
    require(anonymous_executable_bytes() - original_mappings > former_cache_bytes,
            "Actual retained complete-block code crosses the former eight-MiB capacity threshold");
    captured_repeat_entries(machine);
    const auto warm = codegen::publications();
    require(anonymous_executable_bytes() ==
                original_mappings + (warm.bytes - initial_publications.bytes),
            "Executable accounting matches real anonymous read/execute mappings");
    for (unsigned pass = 0; pass < 2; ++pass) {
      for (const auto entry : entries | std::views::reverse) {
        nop_prefix(machine, entry, program);
        const auto after = codegen::publications();
        require(after == warm,
                "Revisiting every earlier complete-block entry allocates and recompiles nothing");
      }
    }
    require(anonymous_executable_bytes() ==
                original_mappings + (warm.bytes - initial_publications.bytes),
            "Warm reuse retains the exact same executable mapping extent");
  }
  require(anonymous_executable_bytes() == original_mappings,
          "Destroying the Core releases all of its actual executable mappings");
}

}
}

int main() {
  try {
    dsp56300::test::shorter_warm_entries();
    dsp56300::test::retained_storage();
    std::cout << dsp56300::test::checks << " native short-prefix/code-retention checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
