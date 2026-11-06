// SPDX-License-Identifier: GPL-3.0-only
// Serial MSB-first carry oracle versus public execution and helper-forbidden RX.
#include "address_model.hpp"
#include "architecture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "native/direct.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word datum = 0x654321, source = 0xa5c37e, high_reverse = 0xab0000;
constexpr std::array<Word, 8> cursors{0, 1, 7, 15, 63, 255, 511, 1023};
constexpr std::array<Word, 8> operands{0, 1, 3, sign24 - 1, sign24, word_mask, 0x555555, 0xaaaaaa};

struct Shape {
  unsigned mode = 0, index = 0;
  bool y = false, store = false;
};

Word encoding(const Shape shape) {
  const auto prefix = shape.y ? isa::Opcode::move_y_effective : isa::Opcode::move_x_effective;
  constexpr Word load_bit = Word{1} << 15;
  const Word register_field = shape.y ? Word{2} << 16 : 0;
  return static_cast<Word>(prefix) | register_field | (shape.store ? 0 : load_bit) |
         (shape.mode << 11) | (shape.index << 8);
}

struct Machine {
  Machine() : core({.program = p, .x = x, .y = y}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
  std::array<std::uint64_t, 4> latched{};
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Machine *>(owner)->calls;
  return 0;
}

struct Operand {
  Word cursor = 0, amount = 0, modifier = 0;
};

void check(Machine &machine, const native::Direct &code, const Shape shape, const Operand operand) {
  const auto next = address_model::updated({.modifier = operand.modifier,
                                            .cursor = operand.cursor,
                                            .offset = operand.amount,
                                            .mode = shape.mode});
  const auto address = shape.mode == 5 || shape.mode == 7 ? next : operand.cursor;
  if (address >= memory_words) { return; }
  auto state = register_model::seed(status_byte);
  state.r[shape.index] = operand.cursor;
  state.n[shape.index] = operand.amount;
  state.m[shape.index] = operand.modifier;
  auto &bank = shape.y ? state.y : state.x;
  bank = (bank & ~std::uint64_t{word_mask}) | source;
  auto expected = state;
  ++expected.pc;
  if (shape.mode != 5) { expected.r[shape.index] = next; }
  if (!shape.store) {
    auto &output = shape.y ? expected.y : expected.x;
    output = (output & ~std::uint64_t{word_mask}) | datum;
  }
  const auto cost = shape.mode == 5 || shape.mode == 7 ? 2U : 1U;
  for (const bool direct : {true, false}) {
    machine.x.fill(0);
    machine.y.fill(0);
    auto &data = shape.y ? machine.y : machine.x;
    data[address] = datum;
    auto expected_data = data;
    if (shape.store) { expected_data[address] = source; }
    machine.calls = 0;
    if (direct) {
      detail::Cpu cpu;
      cpu.reg = state;
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      bool dirty = false;
      detail::NativeContext context{.runtime = {.owner = &machine,
                                                .latched = &machine.latched,
                                                .moves = forbidden,
                                                .memory = {}},
                                    .exit_requested = &dirty,
                                    .budget = 1};
      require(code.enter(cpu, context) == 1 && machine.calls == 0,
              "Reverse-addressed transfers execute actual RX without an ISA/move helper");
      auto padded = expected;
      padded.a <<= detail::padding_bits;
      padded.b <<= detail::padding_bits;
      require(cpu.reg == padded && cpu.instructions == 1 && cpu.cycles == cost,
              "Native reverse transfer retains full independent state and exact costs");
    } else {
      machine.core.set_registers(state);
      const auto before = machine.core.progress();
      const auto result = machine.core.run(1);
      require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                  machine.core.registers() == expected &&
                  result.progress == Progress{.instructions = before.instructions + 1,
                                              .estimated_cycles = before.estimated_cycles + cost},
              "Public Core execution matches the independent complete reverse-address result");
    }
    const auto &untouched = shape.y ? machine.x : machine.y;
    require(data == expected_data && untouched == std::array<Word, memory_words>{},
            "Reverse transfer changes only its independently declared memory word");
  }
}

void shape(const Shape input) {
  Machine machine;
  const auto opcode = encoding(input);
  machine.core.write(Space::program, start, opcode);
  const detail::JitRuntime runtime{
      .owner = &machine,
      .latched = &machine.latched,
      .execute = forbidden,
      .moves = forbidden,
      .memory = {{{.data = machine.x.data(), .words = machine.x.size()},
                  {.data = machine.y.data(), .words = machine.y.size()}}}};
  const std::array operations{
      detail::BlockInstruction{.instruction = detail::decode(opcode), .pc = start}};
  std::array<Word, agu_registers> modifiers{};
  modifiers.fill(word_mask);
  detail::BlockEmitter emitter(runtime, modifiers);
  const native::Direct code(emitter.generate(operations));
  for (const Word modifier : {0U, high_reverse}) {
    for (const auto cursor : cursors) {
      for (const auto amount : operands) {
        check(machine, code, input, {.cursor = cursor, .amount = amount, .modifier = modifier});
      }
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        check(machine,
              code,
              input,
              {.cursor = cursor, .amount = Word{1} << bit, .modifier = modifier});
      }
    }
  }
}
}
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (const unsigned mode : {0U, 1U, 2U, 3U, 5U, 7U}) {
        for (const bool y : {false, true}) {
          for (const bool store : {false, true}) {
            test::shape({.mode = mode, .index = index, .y = y, .store = store});
          }
        }
      }
    }
    std::cout << test::checks << " independent native reverse-transfer checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
