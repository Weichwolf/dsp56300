// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
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

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word entry = 16, transfer = entry + 2, target = 32;
constexpr Word move_x = op<isa::Opcode::move_x_absolute_short> | (1U << 15);
constexpr Word indirect = 4U << 11;
constexpr Word pointer_mask = 63, irq_mask = 0x300, saved_status = 0x1234, loaded_word = 0x654321;
constexpr std::size_t code_words = target - entry + 4;
enum class Kind : std::uint8_t { jump, call, returned, equal, bit };

struct Case {
  Word opcode, extension;
  Kind kind;
  unsigned words, cycles;
  Word prefix = op<isa::Opcode::nop>;
  bool register_only = false;
};

constexpr std::array cases{
    Case{.opcode = op<isa::Opcode::jmp_absolute_short> | target,
         .extension = 0,
         .kind = Kind::jump,
         .words = 1,
         .cycles = 3},
    Case{.opcode = op<isa::Opcode::jmp_effective> | indirect,
         .extension = 0,
         .kind = Kind::jump,
         .words = 1,
         .cycles = 3},
    Case{.opcode = op<isa::Opcode::jsr_absolute_short> | target,
         .extension = 0,
         .kind = Kind::call,
         .words = 1,
         .cycles = 3},
    Case{.opcode = op<isa::Opcode::jsr_effective> | indirect,
         .extension = 0,
         .kind = Kind::call,
         .words = 1,
         .cycles = 3},
    Case{.opcode = op<isa::Opcode::rts>,
         .extension = 0,
         .kind = Kind::returned,
         .words = 1,
         .cycles = 3},
    Case{.opcode = op<isa::Opcode::jcc_absolute_short> | (10U << 12) | target,
         .extension = 0,
         .kind = Kind::equal,
         .words = 1,
         .cycles = 4},
    Case{.opcode = op<isa::Opcode::jcc_effective> | indirect | 10U,
         .extension = 0,
         .kind = Kind::equal,
         .words = 1,
         .cycles = 4},
    Case{.opcode = op<isa::Opcode::bra_relative_long>,
         .extension = target - transfer,
         .kind = Kind::jump,
         .words = 2,
         .cycles = 4},
    Case{.opcode = op<isa::Opcode::bsr_relative_long>,
         .extension = target - transfer,
         .kind = Kind::call,
         .words = 2,
         .cycles = 5},
    Case{.opcode = op<isa::Opcode::jclr_register> | (4U << 8),
         .extension = target,
         .kind = Kind::bit,
         .words = 2,
         .cycles = 4},
    Case{.opcode = op<isa::Opcode::brclr_register> | (4U << 8),
         .extension = target - transfer,
         .kind = Kind::bit,
         .words = 2,
         .cycles = 5},
    Case{.opcode = op<isa::Opcode::jsr_absolute_short> | target,
         .extension = 0,
         .kind = Kind::call,
         .words = 1,
         .cycles = 3,
         .prefix = op<isa::Opcode::tst_a>},
    Case{.opcode = op<isa::Opcode::rts>,
         .extension = 0,
         .kind = Kind::returned,
         .words = 1,
         .cycles = 3,
         .prefix = op<isa::Opcode::tst_a>},
    Case{.opcode = op<isa::Opcode::jsr_absolute_short> | target,
         .extension = 0,
         .kind = Kind::call,
         .words = 1,
         .cycles = 3,
         .prefix = op<isa::Opcode::tst_a>,
         .register_only = true},
    Case{.opcode = op<isa::Opcode::rts>,
         .extension = 0,
         .kind = Kind::returned,
         .words = 1,
         .cycles = 3,
         .prefix = op<isa::Opcode::tst_a>,
         .register_only = true}};

std::array<Word, code_words> program(const Case item) {
  std::array<Word, code_words> words;
  words.fill(op<isa::Opcode::stop>);
  words[0] = item.register_only ? test::parallel_none | (6U << 13) | (4U << 8) : move_x;
  words[1] = item.prefix;
  words[2] = item.opcode;
  if (item.words == 2) { words[3] = item.extension; }
  words[target - entry] =
      item.register_only ? test::parallel_none | (7U << 13) | (4U << 8) : move_x | (1U << 8);
  words[target - entry + 1] = op<isa::Opcode::nop>;
  words[target - entry + 2] =
      item.kind == Kind::call ? op<isa::Opcode::rts> : op<isa::Opcode::stop>;
  return words;
}

std::vector<State> prefixes(const Registers &initial, const Case item, const Word datum) {
  std::vector<State> states{State{
      .registers = initial, .failure = {}, .execution = ExecutionState::running, .progress = {}}};
  auto state = states.back();
  const auto save = [&](const Word next, const unsigned cycles) {
    state.registers.pc = next;
    ++state.progress.instructions;
    state.progress.estimated_cycles += cycles;
    states.push_back(state);
  };
  const auto load = [&](const Word value) {
    state.registers.x = (state.registers.x & ~std::uint64_t{word_mask}) | value;
  };
  load(datum);
  save(entry + 1, 1);
  const auto before = state.registers;
  test::sequence_model::alu(state.registers, before, item.prefix);
  save(transfer, 1);
  const bool taken = (item.kind != Kind::equal || (initial.sr & test::flag_z) != 0) &&
                     (item.kind != Kind::bit || datum % 2 == 0);
  const Word resume = transfer + item.words;
  if (item.kind == Kind::call) {
    state.registers.sp = (state.registers.sp + 1) % 64;
    ++state.registers.sc;
    state.registers.stack[state.registers.sp % stack_entries] =
        (std::uint64_t{resume} << test::word_bits) | state.registers.sr;
  } else if (item.kind == Kind::returned) {
    state.registers.sp = (state.registers.sp + pointer_mask) % 64;
    --state.registers.sc;
  }
  save(taken ? target : resume, item.cycles);
  if (taken) {
    load(loaded_word);
    save(target + 1, 1);
    save(target + 2, 1);
    if (item.kind == Kind::call) {
      state.registers.sp = (state.registers.sp + pointer_mask) % 64;
      --state.registers.sc;
      save(resume, 3);
    }
  }
  state.execution = ExecutionState::stopped;
  save(state.registers.pc + 1, 10);
  return states;
}

void check_boundaries(Core &core, const Registers &initial, const std::vector<State> &expected) {
  const auto check = [&](const RunResult &result, const std::size_t index) {
    const auto actual = core.state();
    const auto &wanted = expected[index];
    require(actual.registers == wanted.registers && actual.progress == wanted.progress &&
                actual.execution == wanted.execution && actual.failure == Fault{} &&
                result.retired == wanted.progress.instructions &&
                result.progress == wanted.progress && result.state == wanted.execution &&
                result.fault == Fault{} && !result.yielded,
            "Mixed native/control prefixes preserve all banks, stack, flags and progress");
  };
  for (std::size_t count = 0; count <= expected.size(); ++count) {
    core.reset();
    core.set_registers(initial);
    check(core.run(static_cast<std::uint32_t>(count)), std::min(count, expected.size() - 1));
  }
  for (std::uint64_t deadline = 0; deadline <= expected.back().progress.estimated_cycles + 1;
       ++deadline) {
    core.reset();
    core.set_registers(initial);
    std::size_t index = 0;
    while (index + 1 < expected.size() && expected[index].progress.estimated_cycles < deadline) {
      ++index;
    }
    check(core.run_until(deadline), index);
  }
}

void fault_boundary(Core &core, Registers initial, const Case item, const Word datum) {
  if (item.kind != Kind::call && item.kind != Kind::returned) { return; }
  auto expected = prefixes(initial, item, datum)[2];
  initial.sc = item.kind == Kind::call ? static_cast<Word>(stack_entries - 1) : 0U;
  expected.registers.sc = initial.sc;
  expected.registers.pc = item.kind == Kind::call ? target : transfer + item.words;
  expected.failure = {.kind = FaultKind::stack,
                      .space = Space::program,
                      .address = transfer,
                      .value = initial.sp,
                      .pc = transfer};
  core.reset();
  core.set_registers(initial);
  const auto result = core.run(8);
  const auto actual = core.state();
  require(actual.registers == expected.registers && actual.progress == expected.progress &&
              actual.failure == expected.failure && actual.execution == expected.execution &&
              result.retired == expected.progress.instructions &&
              result.progress == expected.progress && result.fault == expected.failure &&
              !result.yielded,
          "Stack faults preserve the complete preceding native prefix and original fault boundary");
}

void boundaries(const Case item) {
  std::array<Word, 64> p{};
  std::array<Word, 2> x{};
  std::array<Word, 2> y{};
  constexpr std::array modes{0U,
                             test::mode_down,
                             test::mode_up,
                             test::mode_rm,
                             test::mode_down | test::mode_rm,
                             test::mode_up | test::mode_rm};
  const auto words = program(item);
  std::ranges::copy(words, p.begin() + entry);
  const auto image = test::compile_program(
      words, entry, {.program = p, .x = x, .y = y}, std::span<const Word>(modes));
  Core core(Memory::separate({.program = p, .x = x, .y = y}));
  core.install(image, 0);
  x[1] = loaded_word;
  for (const Word mode : modes) {
    for (const Word flags : {0U, test::flag_z, 0xfbU, 0xffU}) {
      for (const Word pointer : {0U, 15U, 63U}) {
        for (const Word datum : {0x800004U, 0x800005U}) {
          x[0] = datum;
          auto initial = test::register_model::seed(mode);
          if (item.register_only) {
            initial.y = (std::uint64_t{loaded_word} << test::word_bits) | datum;
          }
          initial.pc = entry;
          initial.sr = mode | flags | irq_mask;
          initial.r[0] = target;
          initial.sp = pointer;
          initial.sc = 2;
          initial.stack[pointer % stack_entries] =
              (std::uint64_t{target} << test::word_bits) | saved_status;
          const auto expected = prefixes(initial, item, datum);
          check_boundaries(core, initial, expected);
          fault_boundary(core, initial, item, datum);
        }
      }
    }
  }
}
}

int main() {
  try {
    for (const auto item : cases) { boundaries(item); }
    std::cout << test::checks << " mixed native/control prefix checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
