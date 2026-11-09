// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/index.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word low = 0x100, middle = 0x300, high = 0x10000;
constexpr std::size_t program_words = high + 2;
constexpr Word maximum_grant = 33;
constexpr Word add = isa::opcode<isa::Opcode::add_x0_a>()[0];
constexpr Word negate = isa::opcode<isa::Opcode::neg_b>()[0];
constexpr Word transfer = isa::opcode<isa::Opcode::tfr_y1_b>()[0];
constexpr Word jump = isa::opcode<isa::Opcode::jmp_absolute_short>()[0];
constexpr std::array addresses{low, middle, high};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm};

std::uint32_t forbidden(void * /*owner*/,
                        const detail::BlockInstruction * /*instruction*/) noexcept {
  return 0;
}

struct Harness {
  std::vector<Word> program = std::vector<Word>(program_words);
  std::array<Word, 8> x{}, y{};
  std::array<const void *, addresses.size()> entries{};
  detail::NativeTable table{3};
  detail::ExecutableArena arena;
  detail::JitRuntime runtime{
      .execute = forbidden,
      .moves = forbidden,
      .memory = {{{.data = x.data(), .words = x.size()}, {.data = y.data(), .words = y.size()}}}};

  explicit Harness(const Word mode) {
    program[low] = add;
    program[low + 1] = jump | middle;
    program[middle] = negate;
    program[middle + 1] = jump | low;
    program[high] = transfer;
    program[high + 1] = jump | low;
    for (std::size_t slot = 0; slot < addresses.size(); ++slot) {
      const auto pc = addresses[slot];
      auto state = register_model::seed(mode);
      state.pc = pc;
      const auto operations = detail::parse_block(program, state, runtime);
      require(operations.size() == 2, "Natural native blocks retain arithmetic and their jump");
      const auto *entry = arena.append(detail::emit_block(operations, runtime, state.m));
      entries[slot] = entry;
      table.insert(pc, entry);
    }
  }
};

struct Expected {
  Registers registers;
  Progress progress;
};

Expected predict(const Registers initial, const Word budget) {
  Expected result{.registers = initial, .progress = {}};
  for (Word step = 0; step < budget; ++step) {
    auto &state = result.registers;
    const auto old = state;
    if (state.pc == high || state.pc == low || state.pc == middle) {
      auto operation = negate;
      if (state.pc == high) { operation = transfer; }
      if (state.pc == low) { operation = add; }
      sequence_model::alu(state, old, operation);
      ++state.pc;
      ++result.progress.estimated_cycles;
    } else {
      state.pc = state.pc == low + 1 ? middle : low;
      result.progress.estimated_cycles += 3;
    }
    ++result.progress.instructions;
  }
  return result;
}

struct RunInput {
  Word mode = 0, budget = 0, retired = 0;
};

void run(Harness &harness, const RunInput input) {
  auto initial = register_model::seed(input.mode);
  initial.pc = high;
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  detail::NativeContext context{.runtime = harness.runtime};
  const auto expected = predict(initial, input.retired);
  const auto retired =
      detail::enter_blocks(harness.entries[2], cpu, context, input.budget, harness.table.index());
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(retired == input.retired && cpu.reg == expected.registers &&
              Progress{.instructions = cpu.instructions, .estimated_cycles = cpu.cycles} ==
                  expected.progress &&
              context.boundary == detail::NativeBoundary::none && cpu.repeat_remaining == 0 &&
              !cpu.repeat_loaded && !cpu.repeat_completed && !cpu.interrupt_inhibited &&
              cpu.processing == ExecutionState::running && cpu.debug_flags == 0,
          "One native call preserves complete registers, exact progress and partial-block grants");
  require(harness.x == std::array<Word, 8>{} && harness.y == std::array<Word, 8>{},
          "Linked arithmetic leaves both borrowed data memories untouched");
}

void cases(const Word mode) {
  Harness harness(mode);
  const auto program = harness.program;
  for (Word budget = 0; budget <= maximum_grant; ++budget) {
    run(harness, {.mode = mode, .budget = budget, .retired = budget});
  }
  harness.table.insert(low, nullptr);
  run(harness, {.mode = mode, .budget = 9, .retired = 2});
  harness.table.insert(low, harness.entries[0]);
  run(harness, {.mode = mode, .budget = 9, .retired = 9});
  require(harness.program == program, "Native dispatch never rewrites its original program");
}
}
}

int main() {
  try {
    for (const auto mode : dsp56300::test::modes) { dsp56300::test::cases(mode); }
    std::cout << "Native linked blocks: sparse index, exact grants, full state and memory passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
