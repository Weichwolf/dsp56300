// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
using isa::Opcode;
constexpr Word entry = 0x100, address = 6, target = 0x80;
constexpr unsigned indirect_register = 3, bits = 24;
constexpr Word datum = 0x69a53c, raw_high = 0x55000000;
constexpr std::uint64_t accumulator_a = 0x123456789abc, accumulator_b = 0xabcdef012345;
constexpr Word initial_status = 0x300, carry_status = 0x301, cleared_status = 0x315;
constexpr Word full_status = 0xc0037f, absolute_ea = 0x30;
constexpr unsigned warm_instructions = 50, run_instructions = 5000;
enum class Form : std::uint8_t { short_address, absolute, indirect };
enum class Operation : std::uint8_t {
  clear,
  set,
  change,
  test,
  jump_clear,
  jump_set,
  relative_clear,
  relative_set
};

struct Shape {
  Form form = Form::short_address;
  Operation operation = Operation::test;
  bool y = false;
  unsigned bit = 0;
};

Word opcode(const Opcode name) {
  return static_cast<Word>(name);
}

bool branching(const Shape shape) {
  return shape.operation >= Operation::jump_clear;
}

bool modifies(const Shape shape) {
  return shape.operation < Operation::test;
}

unsigned length(const Shape shape) {
  return branching(shape) || shape.form == Form::absolute ? 2U : 1U;
}

unsigned cycles(const Shape shape) {
  if (branching(shape)) { return shape.operation >= Operation::relative_clear ? 5U : 4U; }
  return shape.form == Form::absolute ? 3U : 2U;
}

Word encoding(const Shape shape) {
  constexpr std::array short_names{Opcode::bclr_absolute_short,
                                   Opcode::bset_absolute_short,
                                   Opcode::bchg_absolute_short,
                                   Opcode::btst_absolute_short,
                                   Opcode::jclr_absolute_short,
                                   Opcode::jset_absolute_short,
                                   Opcode::brclr_absolute_short,
                                   Opcode::brset_absolute_short};
  constexpr std::array effective_names{Opcode::bclr_effective,
                                       Opcode::bset_effective,
                                       Opcode::bchg_effective,
                                       Opcode::btst_effective,
                                       Opcode::jclr_effective,
                                       Opcode::jset_effective,
                                       Opcode::brclr_effective,
                                       Opcode::brset_effective};
  const auto kind = static_cast<unsigned>(shape.operation);
  auto selector = address;
  if (shape.form == Form::absolute) { selector = absolute_ea; }
  if (shape.form == Form::indirect) { selector = 0x20U | indirect_register; }
  return opcode((shape.form == Form::short_address ? short_names : effective_names).at(kind)) |
         (selector << 8U) | (shape.y ? 64U : 0U) | shape.bit;
}

Word extension(const Shape shape) {
  if (!branching(shape)) { return address; }
  return shape.operation >= Operation::relative_clear
             ? (target + word_mask + 1U - entry) & word_mask
             : target;
}

Registers seed() {
  Registers state{.pc = entry, .sr = initial_status, .lc = 9, .sp = 5, .sc = 2};
  state.a = accumulator_a;
  state.b = accumulator_b;
  state.m.fill(word_mask);
  state.r[indirect_register] = address;
  return state;
}

Word changed(const Word original, const Shape shape) {
  if (!modifies(shape)) { return original; }
  auto value = original & word_mask;
  const auto unit = Word{1} << shape.bit;
  const bool previous = (value / unit) % 2U != 0;
  if (previous && shape.operation != Operation::set) { value -= unit; }
  if (!previous && shape.operation != Operation::clear) { value += unit; }
  return value;
}

Registers expected(Registers state, const Word original, const Shape shape) {
  const bool previous = ((original >> shape.bit) & 1U) != 0;
  state.pc = entry + length(shape);
  if (branching(shape)) {
    const bool wanted =
        shape.operation == Operation::jump_set || shape.operation == Operation::relative_set;
    if (previous == wanted) { state.pc = target; }
  } else {
    state.sr = (state.sr & ~1U) | (previous ? 1U : 0U);
  }
  return state;
}

struct Owner final : Io {
  std::array<Word, entry + 8> program{};
  std::array<std::array<Word, 64>, 2> data{};
  std::array<std::array<std::uint8_t, 64>, 2> known{};
  std::array<std::array<MemoryWindow, 1>, 2> windows{};
  unsigned reads = 0, writes = 0;

  Owner() {
    for (unsigned bank = 0; bank < data.size(); ++bank) {
      known[bank].fill(1);
      windows[bank][0] = {.words = data[bank], .known = known[bank], .writable = true};
    }
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc < program.size() ? std::optional{program[pc]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return windows[space == Space::x ? 0 : 1];
  }

  bool read(const Space space, const Word at, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program) {
      if (at >= program.size()) { return false; }
      value = program[at];
      return true;
    }
    ++reads;
    const auto bank = space == Space::x ? 0U : 1U;
    if (at >= data[bank].size() || known[bank][at] == 0) { return false; }
    value = data[bank][at];
    return true;
  }

  bool write(const Space space, const Word at, Word /*pc*/, const Word value) noexcept override {
    ++writes;
    const auto bank = space == Space::x ? 0U : 1U;
    if (space == Space::program || at >= data[bank].size() || !windows[bank][0].writable) {
      return false;
    }
    data[bank][at] = value;
    known[bank][at] = 1;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void one(const Shape shape, const bool granted) {
  Owner owner;
  owner.program[entry] = encoding(shape);
  if (length(shape) == 2) { owner.program[entry + 1] = extension(shape); }
  Core core(granted ? Memory::external()
                    : Memory::separate(
                          {.program = owner.program, .x = owner.data[0], .y = owner.data[1]}),
            &owner);
  detail::Cpu cpu;
  unsigned helpers = 0;
  detail::JitRuntime runtime{.owner = &helpers, .execute = forbidden, .memory = {}};
  for (unsigned bank = 0; bank < 2; ++bank) {
    if (granted) {
      runtime.windows[bank] = owner.windows[bank];
    } else {
      runtime.memory[bank] = {.data = owner.data[bank].data(), .words = owner.data[bank].size()};
    }
  }
  detail::Jit native(cpu, owner.program, runtime);
  const auto bank = shape.y ? 1U : 0U;
  for (const Word status : {initial_status, carry_status, full_status}) {
    for (const bool set : {false, true}) {
      const auto unit = Word{1} << shape.bit;
      const auto old = raw_high | (datum & ~unit) | (set ? unit : 0U);
      auto initial = seed();
      initial.sr = status;
      const auto truth = expected(initial, old, shape);
      auto memory_truth = owner.data;
      memory_truth[bank][address] = changed(old, shape);
      owner.data[bank][address] = old;
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result = core.run(1);
      require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                  core.registers() == truth &&
                  result.progress.estimated_cycles == before.estimated_cycles + cycles(shape) &&
                  owner.data == memory_truth && owner.reads == 0 && owner.writes == 0,
              "Public RAM bit operations preserve full state, memory, carry, PC and literal cycles "
              "without callbacks");
      owner.data[bank][address] = old;
      cpu.reg = initial;
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      const auto previous = cpu.cycles;
      require(native.run(1) == 1 && helpers == 0,
              "Known ordinary bit accesses execute without the instruction helper");
      cpu.reg.a >>= detail::padding_bits;
      cpu.reg.b >>= detail::padding_bits;
      require(cpu.reg == truth && owner.data == memory_truth &&
                  cpu.cycles == previous + cycles(shape),
              "Native-only RAM bit execution matches independent state and memory expectations");
    }
  }
}

void operations() {
  for (const bool granted : {false, true}) {
    for (const bool y : {false, true}) {
      for (unsigned kind = 0; kind < 8; ++kind) {
        for (const auto form : {Form::short_address, Form::absolute, Form::indirect}) {
          Shape shape{.form = form, .operation = static_cast<Operation>(kind), .y = y};
          if (branching(shape) && form == Form::absolute) { continue; }
          for (unsigned bit = 0; bit < bits; ++bit) {
            shape.bit = bit;
            one(shape, granted);
          }
        }
      }
    }
  }
}

void fault_prefixes() {
  for (const bool unknown : {false, true}) {
    Owner owner;
    const Shape shape{.operation = Operation::set};
    owner.program[entry] = isa::opcode<Opcode::clr_a>().front();
    owner.program[entry + 1] = encoding(shape);
    owner.data[0][address] = datum;
    owner.known[0][address] = unknown ? 0 : 1;
    owner.windows[0][0].writable = unknown;
    Core core(Memory::external(), &owner);
    auto initial = seed();
    initial.sr = carry_status;
    core.set_registers(initial);
    auto truth = initial;
    truth.pc += 2;
    truth.a = 0;
    truth.sr = cleared_status;
    const auto result = core.run(2);
    require(result.retired == 1 &&
                result.fault.kind == (unknown ? FaultKind::memory_read : FaultKind::memory_write) &&
                result.fault.pc == entry + 1 && result.fault.address == address &&
                result.fault.space == Space::x && core.registers() == truth &&
                owner.data[0][address] == datum && owner.reads == 1 &&
                owner.writes == (unknown ? 0U : 1U),
            "Unknown or protected RAM preserves the complete preceding ALU state and original "
            "owner fault");
  }
}

void deadline_cases() {
  constexpr std::array<std::uint64_t, 5> completion{2, 6, 9, 11, 15};
  constexpr std::array pcs{entry + 1, entry + 3, entry, entry + 1, entry};
  constexpr std::array<Word, 5> words{1, 1, 1, 0, 0};
  constexpr std::array<Word, 5> carries{0, 0, 0, 1, 1};
  for (unsigned deadline = 1; deadline <= completion.back(); ++deadline) {
    Owner owner;
    owner.program[entry] = encoding({.operation = Operation::change});
    owner.program[entry + 1] = encoding({.operation = Operation::jump_clear});
    owner.program[entry + 2] = entry;
    owner.program[entry + 3] = opcode(Opcode::jmp_absolute_short) | entry;
    Core core(Memory::external(), &owner);
    core.set_registers(seed());
    const auto result = core.run_until(deadline);
    std::size_t index = 0;
    while (completion[index] < deadline) { ++index; }
    auto truth = seed();
    truth.pc = pcs[index];
    truth.sr |= carries[index];
    require(result.fault.kind == FaultKind::none && result.retired == index + 1 &&
                result.progress.estimated_cycles == completion[index] &&
                core.registers() == truth && owner.data[0][address] == words[index] &&
                owner.reads == 0 && owner.writes == 0,
            "Every deadline inside a real bit loop stops at its first complete instruction with "
            "exact state");
  }
}

void repeated_and_warm() {
  Owner owner;
  const Shape toggle{.operation = Operation::change};
  owner.program[entry] = opcode(Opcode::rep_immediate) | (3U << 8U);
  owner.program[entry + 1] = encoding(toggle);
  Core repeated(Memory::external(), &owner);
  repeated.set_registers(seed());
  require(repeated.run(1).retired == 1, "Real REP setup precedes native bit iteration");
  require(
      repeated.run(3).retired == 3 && repeated.registers().lc == 9 &&
          repeated.registers().pc == entry + 2 && repeated.registers().sr == initial_status &&
          repeated.progress().estimated_cycles == 11 && owner.data[0][address] == 1 &&
          owner.reads == 0 && owner.writes == 0,
      "Repeated native bit changes retain exact iteration counts, restored LC and genuine carry");
  Owner loop;
  loop.program[entry] = encoding(toggle);
  loop.program[entry + 1] = encoding({.operation = Operation::jump_clear});
  loop.program[entry + 2] = entry;
  loop.program[entry + 3] = opcode(Opcode::jmp_absolute_short) | entry;
  Core core(Memory::external(), &loop);
  core.set_registers(seed());
  require(core.run(warm_instructions).retired == warm_instructions,
          "Warm every authored bit-loop path before checking retention");
  const auto publication = codegen::publications();
  const auto before = core.progress();
  const auto result = core.run(run_instructions);
  require(
      result.retired == run_instructions && result.fault.kind == FaultKind::none &&
          core.registers().pc == entry && core.registers().sr == carry_status &&
          loop.data[0][address] == 0 && loop.reads == 0 && loop.writes == 0 &&
          result.progress.estimated_cycles ==
              before.estimated_cycles + (std::uint64_t{3} * run_instructions) &&
          publication == codegen::publications(),
      "A warm native bit loop retains code and real state without data callbacks or recompilation");
}
}
}

int main() {
  try {
    dsp56300::test::operations();
    dsp56300::test::fault_prefixes();
    dsp56300::test::deadline_cases();
    dsp56300::test::repeated_and_warm();
    std::cout << "Native RAM bit operations, flow, faults and repeat checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
