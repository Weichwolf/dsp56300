// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
constexpr std::array scales{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array raw{std::uint64_t{0},
                         std::uint64_t{1},
                         saturation_model::positive,
                         saturation_model::positive + 1,
                         saturation_model::negative - 1,
                         saturation_model::negative,
                         sign56 - 1,
                         sign56,
                         accumulator_mask};
constexpr std::array banks{std::uint64_t{0},
                           std::uint64_t{0x800000800000},
                           std::uint64_t{0x7fffff7fffff},
                           std::uint64_t{0xffffff000001}};
constexpr Word short_mask = 63;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

struct Case {
  Word opcode = 0, extension = 0, mode = 0;
  std::optional<bfu_model::Input> logical = std::nullopt;
};

struct Invocation {
  Word flags = 0;
  std::size_t index = 0;
  bool admitted = true;
};

unsigned words(const Case point) {
  return point.opcode < parallel_none && (point.opcode & 0x40U) != 0 ? 2U : 1U;
}

void model(Registers &state, const Case point) {
  if (point.logical) {
    const auto old = state;
    bfu_model::apply(state, old, *point.logical);
  } else if (point.opcode < parallel_none) {
    auto old = state;
    old.x = words(point) == 2 ? point.extension : (point.opcode >> 8) & short_mask;
    saturation_model::alu(state, old, 0x40U | (point.opcode & 15U));
  } else {
    saturation_model::alu(state, state, point.opcode);
  }
  state.pc = (state.pc + words(point)) & word_mask;
}

void check(const void *const entry, const Case point, const Invocation input) {
  auto expected = register_model::seed(point.mode);
  expected.pc = start;
  expected.sr = point.mode | input.flags;
  expected.a = raw.at((input.flags + input.index) % raw.size());
  expected.b = raw.at((input.flags + input.index + 3) % raw.size());
  expected.x = banks.at(input.index);
  expected.y = banks.at((input.index + 1) % banks.size());
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = cpu.cycles = maximum;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  unsigned helpers = 0;
  detail::NativeContext context{
      .runtime = {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
      .budget = input.admitted ? 1U : 0U};
  if (input.admitted) { model(expected, point); }
  expected.a <<= detail::padding_bits;
  expected.b <<= detail::padding_bits;
  const auto count = detail::enter_block(entry, cpu, context);
  if (cpu.reg != expected) {
    std::cerr << std::hex << "opcode=" << point.opcode << " mode=" << point.mode
              << " flags=" << input.flags << " index=" << input.index << " actual_a=" << cpu.reg.a
              << " expected_a=" << expected.a << " actual_b=" << cpu.reg.b
              << " expected_b=" << expected.b << " actual_sr=" << cpu.reg.sr
              << " expected_sr=" << expected.sr << std::dec << '\n';
  }
  require(helpers == 0 && count == static_cast<unsigned>(input.admitted) && cpu.reg == expected,
          "A64 arithmetic differs from independent raw-width state or called an ISA helper");
  require(cpu.instructions == (input.admitted ? 0 : maximum) &&
              cpu.cycles == (input.admitted ? words(point) - 1 : maximum) &&
              cpu.repeat_completed != input.admitted && cpu.interrupt_inhibited != input.admitted &&
              context.boundary == detail::NativeBoundary::none &&
              context.failure.kind == FaultKind::none && cpu.repeat_remaining == 0 &&
              !cpu.repeat_loaded && cpu.processing == ExecutionState::running &&
              cpu.debug_flags == 0,
          "A64 arithmetic must retain exact grants, wrapping progress and non-arithmetic state");
}

void probe(const Case point) {
  auto instruction = detail::decode(point.opcode);
  instruction.extension = point.extension;
  const std::array operations{
      detail::BlockInstruction{.instruction = instruction, .pc = start, .mode = point.mode}};
  detail::a64::Emitter emitter;
  detail::ExecutableArena arena;
  const auto *entry = arena.append(emitter.generate(operations));
  check(entry, point, {.admitted = false});
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (std::size_t index = 0; index < banks.size(); ++index) {
      check(entry, point, {.flags = flags, .index = index});
    }
  }
}

void prefixes() {
  constexpr std::array steps{Case{.opcode = 0x200036},
                             Case{.opcode = 0x200021},
                             Case{.opcode = 0x2000d3},
                             Case{.opcode = 0x200019},
                             Case{.opcode = 0x0140c0, .extension = word_mask},
                             Case{.opcode = 0x200034},
                             Case{.opcode = 0x2000fb},
                             Case{.opcode = 0x200011},
                             Case{.opcode = 0x20001b}};
  for (const auto scale : scales) {
    for (const Word saturation : {0U, saturation_model::mode}) {
      const auto mode = scale | saturation;
      std::array<detail::BlockInstruction, steps.size()> operations{};
      Word address = word_mask - 3;
      for (std::size_t index = 0; index < steps.size(); ++index) {
        auto decoded = detail::decode(steps[index].opcode);
        decoded.extension = steps[index].extension;
        operations[index] = {.instruction = decoded, .pc = address, .mode = mode};
        address = (address + words(steps[index])) & word_mask;
      }
      detail::ExecutableArena arena;
      detail::a64::Emitter emitter;
      const auto *entry = arena.append(emitter.generate(operations));
      for (Word budget = 0; budget <= steps.size(); ++budget) {
        auto expected = register_model::seed(mode);
        expected.pc = operations.front().pc;
        expected.sr = mode | status_byte;
        expected.a = saturation_model::positive + 1;
        expected.b = sign56;
        expected.x = banks[1];
        expected.y = banks[2];
        detail::Cpu cpu;
        cpu.reg = expected;
        cpu.reg.a <<= detail::padding_bits;
        cpu.reg.b <<= detail::padding_bits;
        cpu.instructions = cpu.cycles = maximum;
        cpu.repeat_completed = cpu.interrupt_inhibited = true;
        Word cycles = 0;
        for (Word index = 0; index < budget; ++index) {
          model(expected, steps[index]);
          cycles += words(steps[index]);
        }
        expected.a <<= detail::padding_bits;
        expected.b <<= detail::padding_bits;
        unsigned helpers = 0;
        detail::NativeContext context{
            .runtime = {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
            .budget = budget};
        const auto count = detail::enter_block(entry, cpu, context);
        require(count == budget && cpu.reg == expected && helpers == 0 &&
                    cpu.instructions == (budget != 0 ? budget - 1 : maximum) &&
                    cpu.cycles == (cycles != 0 ? cycles - 1 : maximum) &&
                    cpu.repeat_completed == (budget == 0) &&
                    cpu.interrupt_inhibited == (budget == 0),
                "Retained A64 blocks preserve every prefix, changing flags and logical PC wrap");
      }
    }
  }
}

void arithmetic_matrix() {
  constexpr std::array<Word, 33> forms{0x01, 0x11, 0x13, 0x03, 0x26, 0x36, 0x41, 0x51, 0x61,
                                       0x71, 0x10, 0x14, 0x20, 0x24, 0x30, 0x34, 0x40, 0x44,
                                       0x50, 0x54, 0x60, 0x64, 0x70, 0x74, 0x21, 0x25, 0x31,
                                       0x35, 0x05, 0x45, 0x55, 0x65, 0x75};
  for (const auto scale : scales) {
    for (const Word saturation : {0U, saturation_model::mode}) {
      const auto mode = scale | saturation;
      for (const auto form : forms) {
        for (const Word destination : {0U, 8U}) {
          probe({.opcode = parallel_none | form | destination, .mode = mode});
        }
      }
      constexpr Word first_product = 0x80;
      for (Word product = first_product; product <= status_byte; ++product) {
        probe({.opcode = parallel_none | product, .mode = mode});
      }
      constexpr Word short_prefix = 0x014080;
      constexpr Word long_prefix = 0x0140c0;
      for (const Word kind : {0U, 4U, 5U, 8U, 12U, 13U}) {
        probe({.opcode = short_prefix | (short_mask << 8) | kind, .mode = mode});
        for (const Word value : {0U, 1U, sign24 - 1, sign24, word_mask}) {
          probe({.opcode = long_prefix | kind, .extension = value, .mode = mode});
        }
      }
    }
  }
}

void logical_immediates(const Word mode,
                        const bfu_model::Function function,
                        const bool destination_b) {
  if (function == bfu_model::Function::complement) { return; }
  for (const auto form : {bfu_model::Form::short_immediate, bfu_model::Form::long_immediate}) {
    const auto sign = form == bfu_model::Form::short_immediate ? 32U : sign24;
    const auto mask = form == bfu_model::Form::short_immediate ? short_mask : word_mask;
    for (const Word value : {0U, 1U, sign - 1, sign, mask}) {
      const bfu_model::Input input{
          .function = function, .form = form, .value = value, .destination_b = destination_b};
      probe(
          {.opcode = bfu_model::opcode(input), .extension = value, .mode = mode, .logical = input});
    }
  }
}

void logic_matrix() {
  for (const auto scale : scales) {
    for (const Word saturation : {0U, saturation_model::mode}) {
      for (const auto function : {bfu_model::Function::bit_and,
                                  bfu_model::Function::bit_or,
                                  bfu_model::Function::bit_xor,
                                  bfu_model::Function::complement}) {
        for (const bool destination_b : {false, true}) {
          for (unsigned source = 0; source < banks.size(); ++source) {
            const bfu_model::Input input{
                .function = function, .source = source, .destination_b = destination_b};
            probe(
                {.opcode = bfu_model::opcode(input), .mode = scale | saturation, .logical = input});
          }
          logical_immediates(scale | saturation, function, destination_b);
        }
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::arithmetic_matrix();
    dsp56300::test::logic_matrix();
    dsp56300::test::prefixes();
    std::cout << "A64 arithmetic, MAC, rounding, SM and BFU expectations passed: "
              << dsp56300::test::checks << " checks\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
