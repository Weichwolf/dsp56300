// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
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
constexpr Word address = 17, datum = 0xa97643;
constexpr unsigned words = 64, first_address_register = 16, last_offset_register = 31;
constexpr Word selector_mask = 0x370000, high_selector_mask = 24, load_bit = 0x8000;
constexpr unsigned high_selector_shift = 17;
constexpr auto nop = isa::nop.front();

struct Input {
  unsigned selector = first_address_register;
  unsigned mode = 0; // Absolute short, post-increment by one, or (R0).
  bool y = false, store = false;
};

Word encoding(const Input input, const Word location = address) {
  const auto base =
      input.mode == 0 ? isa::Opcode::move_x_absolute_short : isa::Opcode::move_x_effective;
  const auto operand = input.mode == 0 ? location << 8U : input.mode << 11U;
  return (static_cast<Word>(base) & ~selector_mask) | ((input.selector & 7U) << 16U) |
         ((input.selector & high_selector_mask) << high_selector_shift) | operand |
         (input.store ? 0U : load_bit) | (input.y ? 0x080000U : 0U);
}

Registers initial() {
  auto state = register_model::seed(0);
  state.pc = 0;
  state.sr = status_byte;
  state.r[0] = address;
  state.m.fill(word_mask);
  return state;
}

Registers expected(const Input input, Word &memory) {
  auto state = initial();
  if (input.mode == 3) { ++state.r[0]; }
  if (input.store) {
    memory = register_model::read(state, input.selector);
  } else {
    register_model::write(state, input.selector, memory);
  }
  ++state.pc;
  return state;
}

struct Image final : Io {
  std::array<Word, 3> program{};
  std::array<std::array<Word, words>, 2> data{};
  std::array<std::array<std::uint8_t, words>, 2> known{};
  std::array<MemoryWindow, 2> windows;
  unsigned reads = 0, writes = 0;

  Image()
      : windows{{{.words = data[0], .known = known[0], .writable = true},
                 {.words = data[1], .known = known[1], .writable = true}}} {
    for (auto &bank : known) { bank.fill(1); }
    for (auto &bank : data) { bank[address] = datum; }
    program.fill(nop);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word index) const noexcept override {
    return index < program.size() ? std::optional{program[index]} : std::nullopt;
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return std::span<const MemoryWindow>{windows}.subspan(space == Space::x ? 0U : 1U, 1);
  }

  bool read(const Space space, const Word index, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program) {
      const auto result = inspect_program(index);
      if (!result) { return false; }
      value = *result;
      return true;
    }
    ++reads;
    if (index >= words || (space != Space::x && space != Space::y)) { return false; }
    const auto bank = space == Space::x ? 0U : 1U;
    if (known[bank][index] == 0) { return false; }
    value = data[bank][index];
    return true;
  }

  bool write(const Space space, const Word index, Word /*pc*/, const Word value) noexcept override {
    ++writes;
    if (index >= words || (space != Space::x && space != Space::y)) { return false; }
    const auto bank = space == Space::x ? 0U : 1U;
    if (!windows[bank].writable) { return false; }
    data[bank][index] = value;
    known[bank][index] = 1;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*index*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

void external_case(const Input input) {
  Image image;
  image.program[0] = encoding(input);
  const auto bank = input.y ? 1U : 0U;
  if (input.store) { image.known[bank][address] = 0; }
  auto wanted_data = image.data;
  const auto wanted = expected(input, wanted_data[bank][address]);
  Core core(Memory::external(), &image);
  for (unsigned invocation = 0; invocation < 2; ++invocation) {
    core.set_registers(initial());
    const auto before = core.progress();
    const auto result = core.run(1);
    // FM table A-1/A-6: absolute-short stores cost two; these other forms cost one.
    const auto cost = input.store && input.mode == 0 ? 2U : 1U;
    require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                result.progress == Progress{.instructions = before.instructions + 1,
                                            .estimated_cycles = before.estimated_cycles + cost} &&
                core.registers() == wanted && image.data == wanted_data &&
                image.known[bank][address] == 1 && image.reads == 0 && image.writes == 0,
            "Cold/warm R/N transfers use borrowed RAM directly with exact full state and costs");
  }
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

void direct_case(const Input input) {
  Image image;
  auto wanted_data = image.data;
  const auto wanted = expected(input, wanted_data[input.y ? 1U : 0U][address]);
  Probe probe;
  const detail::JitRuntime runtime{
      .owner = &probe,
      .execute = forbidden,
      .moves = forbidden,
      .memory = {detail::JitMemory{.data = image.data[0].data(), .words = words},
                 detail::JitMemory{.data = image.data[1].data(), .words = words}}};
  detail::BlockEmitter emitter(runtime);
  const std::array operations{
      detail::BlockInstruction{.instruction = detail::decode(encoding(input))}};
  const native::Direct code(emitter.generate(operations));
  detail::Cpu cpu;
  cpu.reg = initial();
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  bool exit = false;
  detail::NativeContext context{.runtime = runtime, .exit_requested = &exit, .budget = 1};
  const auto retired = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  const auto cost = input.store && input.mode == 0 ? 2U : 1U;
  require(retired == 1 && probe.calls == 0 && cpu.reg == wanted && image.data == wanted_data &&
              cpu.instructions == 1 && cpu.cycles == cost,
          "Native R/N lowering invokes neither the opcode executor nor the transfer helper");
}

void fault_case(const Input input) {
  Image image;
  const auto bank = input.y ? 1U : 0U;
  image.program[1] = encoding(input);
  if (input.store) {
    image.windows[bank].writable = false;
  } else {
    image.known[bank][address] = 0;
  }
  const auto untouched = image.data;
  Core core(Memory::external(), &image);
  core.set_registers(initial());
  auto wanted = initial();
  ++wanted.r[0];
  wanted.pc = 2;
  const auto value = input.store ? register_model::read(wanted, input.selector) : 0U;
  const Fault fault{.kind = input.store ? FaultKind::memory_write : FaultKind::memory_read,
                    .space = input.y ? Space::y : Space::x,
                    .address = address,
                    .value = value,
                    .pc = 1};
  const auto result = core.run(10);
  require(result.retired == 1 && result.fault == fault &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
              core.registers() == wanted && image.data == untouched &&
              image.reads == (input.store ? 0U : 1U) && image.writes == (input.store ? 1U : 0U),
          "Refused R/N transfer preserves its retired prefix and one real AGU update");
}

void mixed_prefixes(const unsigned selector) {
  Image image;
  const Input load{.selector = selector};
  const Input store{.selector = selector, .store = true};
  constexpr Word destination = address + 1;
  const std::array program{isa::opcode<isa::Opcode::clr_a>().front(),
                           isa::opcode<isa::Opcode::clr_b>().front(),
                           encoding(load),
                           isa::opcode<isa::Opcode::neg_a>().front(),
                           encoding(store, destination),
                           nop};
  Probe probe;
  const detail::JitRuntime runtime{
      .owner = &probe,
      .execute = forbidden,
      .moves = forbidden,
      .memory = {detail::JitMemory{.data = image.data[0].data(), .words = words},
                 detail::JitMemory{.data = image.data[1].data(), .words = words}}};
  std::array<detail::BlockInstruction, program.size()> operations;
  for (std::size_t index = 0; index < program.size(); ++index) {
    operations[index] = {.instruction = detail::decode(program[index]),
                         .pc = static_cast<Word>(index)};
  }
  for (const auto bounds : {detail::ExecutionBounds::precise, detail::ExecutionBounds::whole}) {
    detail::BlockEmitter emitter(runtime);
    const native::Direct code(emitter.generate(operations, bounds));
    require(emitter.packed_retirement(), "R/N memory transfers retain packed native accounting");
    const auto first =
        bounds == detail::ExecutionBounds::whole ? static_cast<Word>(program.size()) : 1U;
    for (Word budget = first; budget <= program.size(); ++budget) {
      image.data[0][destination] = 0;
      auto wanted_data = image.data;
      auto wanted = initial();
      for (Word index = 0; index < budget; ++index) {
        const auto before = wanted;
        sequence_model::alu(wanted, before, program[index]);
        if (index == 2) { register_model::write(wanted, selector, datum); }
        if (index == 4) { wanted_data[0][destination] = register_model::read(wanted, selector); }
        ++wanted.pc;
      }
      detail::Cpu cpu;
      cpu.reg = initial();
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      bool exit = false;
      detail::NativeContext context{.runtime = runtime, .exit_requested = &exit, .budget = budget};
      const auto retired = code.enter(cpu, context);
      cpu.reg.a >>= detail::padding_bits;
      cpu.reg.b >>= detail::padding_bits;
      require(
          retired == budget && cpu.reg == wanted && image.data == wanted_data &&
              cpu.instructions == budget && cpu.cycles == budget + (budget >= 5 ? 1U : 0U) &&
              probe.calls == 0,
          "Every native prefix preserves dirty accumulators, flags, R/N values and exact costs");
    }
  }
}

void region_safety() {
  Image image;
  const detail::JitRuntime runtime{
      .memory = {detail::JitMemory{.data = image.data[0].data(), .words = words},
                 detail::JitMemory{.data = image.data[1].data(), .words = words}}};
  constexpr std::size_t count = 9;
  std::array<detail::BlockInstruction, count> operations;
  const auto load = isa::opcode<isa::Opcode::move_x_effective>().front() | load_bit | (3U << 11U);
  for (std::size_t index = 0; index < count; ++index) {
    operations[index] = {.instruction = detail::decode(load), .pc = static_cast<Word>(index)};
  }
  operations.back().instruction = detail::decode(encoding({.mode = 0}));
  require(!detail::resident_plan(operations, runtime.memory, initial().m).valid,
          "Loaded R/N values cannot bypass later per-access address/stride guards");
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (unsigned selector = first_address_register; selector <= last_offset_register; ++selector) {
      mixed_prefixes(selector);
      for (const unsigned mode : {0U, 3U, 4U}) {
        for (const bool y : {false, true}) {
          for (const bool store : {false, true}) {
            const Input input{.selector = selector, .mode = mode, .y = y, .store = store};
            direct_case(input);
            external_case(input);
            if (mode == 3) { fault_case(input); }
          }
        }
      }
    }
    region_safety();
    std::cout << "Native R/N RAM transfers: " << checks << " independent checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
