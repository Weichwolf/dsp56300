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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
// Check all SysV nonvolatile GPRs independently of the compiler's allocation.
// Both bridges must protect their caller even when guest blocks overwrite them.
extern "C" std::uint64_t
dsp56300_test_block_abi(const void *, detail::Cpu *, detail::NativeContext *) noexcept;
extern "C" std::uint64_t dsp56300_test_run_abi(const void *,
                                               detail::Cpu *,
                                               detail::NativeContext *,
                                               Word,
                                               const void *const *,
                                               const void *const *) noexcept;
asm(".pushsection .text\n"
    ".macro dsp56300_test_abi name,target\n"
    ".globl \\name\n"
    ".hidden \\name\n"
    ".type \\name,@function\n"
    "\\name:\n"
    "endbr64\n"
    "push %rbx\n"
    "push %rbp\n"
    "push %r12\n"
    "push %r13\n"
    "push %r14\n"
    "push %r15\n"
    "sub $8,%rsp\n"
    "movabs $0x1020304050607080,%rbx\n"
    "movabs $0x1020304050607081,%rbp\n"
    "movabs $0x1020304050607082,%r12\n"
    "movabs $0x1020304050607083,%r13\n"
    "movabs $0x1020304050607084,%r14\n"
    "movabs $0x1020304050607085,%r15\n"
    "call \\target\n"
    "mov %eax,%r10d\n"
    "movabs $0x1020304050607080,%r11\n"
    "cmp %r11,%rbx\n"
    "jne 1f\n"
    "inc %r11\n"
    "cmp %r11,%rbp\n"
    "jne 1f\n"
    "inc %r11\n"
    "cmp %r11,%r12\n"
    "jne 1f\n"
    "inc %r11\n"
    "cmp %r11,%r13\n"
    "jne 1f\n"
    "inc %r11\n"
    "cmp %r11,%r14\n"
    "jne 1f\n"
    "inc %r11\n"
    "cmp %r11,%r15\n"
    "jne 1f\n"
    "mov %r10d,%eax\n"
    "jmp 2f\n"
    "1:\n"
    "mov %r10d,%eax\n"
    "bts $32,%rax\n"
    "2:\n"
    "add $8,%rsp\n"
    "pop %r15\n"
    "pop %r14\n"
    "pop %r13\n"
    "pop %r12\n"
    "pop %rbp\n"
    "pop %rbx\n"
    "ret\n"
    ".size \\name,.-\\name\n"
    ".endm\n"
    "dsp56300_test_abi dsp56300_test_block_abi,dsp56300_enter_generated\n"
    "dsp56300_test_abi dsp56300_test_run_abi,dsp56300_run_blocks\n"
    ".purgem dsp56300_test_abi\n"
    ".popsection\n");
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
  bool direct = false;
};

void run(Harness &harness, const RunInput input) {
  auto initial = register_model::seed(input.mode);
  initial.pc = high;
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  detail::NativeContext context{.runtime = harness.runtime, .budget = input.budget};
  const auto expected = predict(initial, input.retired);
  const auto result = input.direct ? dsp56300_test_block_abi(harness.entries[2], &cpu, &context)
                                   : dsp56300_test_run_abi(harness.entries[2],
                                                           &cpu,
                                                           &context,
                                                           input.budget,
                                                           harness.table.index().data(),
                                                           nullptr);
  require((result >> 32) == 0, "Direct and linked entries preserve all host nonvolatile GPRs");
  const auto retired = static_cast<Word>(result);
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
    run(harness, {.mode = mode, .budget = budget, .retired = std::min(budget, 2U), .direct = true});
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
