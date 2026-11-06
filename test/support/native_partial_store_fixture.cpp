// SPDX-License-Identifier: GPL-3.0-only
#include "native_partial_store_fixture.hpp"

#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace dsp56300::test::native_partial {
namespace {
constexpr std::uint64_t input_x = 0x020304010203, input_y = 0x040506030405;
constexpr Word normal_mode = 0x305, xor_input = 0x123456;
}

Word opcode(const Store input) {
  constexpr unsigned selector_shift = 16;
  constexpr unsigned upper_shift = 17;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned index_shift = 8;
  constexpr Word memory = 0x404000;
  constexpr Word high_selector = 0x18;
  constexpr Word y = 0x080000;
  return memory | ((input.selector & 7U) << selector_shift) |
         ((input.selector & high_selector) << upper_shift) | (input.mode << mode_shift) |
         (input.index << index_shift) | (input.space == Space::y ? y : 0U);
}

Registers seed(const Store input) {
  Registers state;
  state.a = input.a;
  state.b = input.a ^ accumulator_mask;
  state.x = input_x;
  state.y = input_y;
  state.omr = normal_mode;
  state.sr = input.status;
  state.r.at(input.index) = address;
  state.n.at(input.index) = offset;
  state.m.fill(word_mask);
  return state;
}

Word partial_word(const Registers &state, const unsigned selector) {
  const auto value = (selector & 1U) != 0 ? state.b : state.a;
  constexpr unsigned main = 24;
  constexpr unsigned extension = 48;
  constexpr Word extension_sign = 128;
  constexpr Word extension_fill = 0xffff00;
  if (selector < 10) { return static_cast<Word>(value & word_mask); }
  if (selector >= 12) { return static_cast<Word>((value >> main) & word_mask); }
  const auto top = static_cast<Word>(value >> extension);
  return top < extension_sign ? top : top | extension_fill;
}

Word effective_address(const Store input) {
  if (input.mode == 5) { return address + offset; }
  if (input.mode == 7) { return address - 1; }
  return address;
}

Registers after(const Store input) {
  auto state = seed(input);
  state.pc = input.mode == 6 ? 2U : 1U;
  auto &cursor = state.r.at(input.index);
  if (input.mode == 0) { cursor -= offset; }
  if (input.mode == 1) { cursor += offset; }
  if (input.mode == 2 || input.mode == 7) { --cursor; }
  if (input.mode == 3) { ++cursor; }
  return state;
}

namespace {
constexpr unsigned stream_pairs = 2048, intervals = 32;
constexpr unsigned stream_words = 2 * stream_pairs;
constexpr Word mac = 0x2000d2, add = 0x200040, bit_xor = 0x200043;
constexpr Word store_partial = 0x546000, store_word = 0x446000;

struct Stream {
  Word operation = 0;
  Registers initial, expected;
  Word output = 0;
};

Stream model(const std::string_view domain, const bool partial) {
  Stream result;
  result.initial.m.fill(word_mask);
  if (domain == domains[0]) {
    result.operation = mac;
    result.initial.x = result.initial.y = 0x100000;
  } else if (domain == domains[1]) {
    result.operation = add;
    result.initial.x = 1;
  } else if (domain == domains[2]) {
    result.operation = bit_xor;
    result.initial.x = xor_input;
  } else {
    throw std::invalid_argument("Unknown authored partial-store stream");
  }
  result.expected = result.initial;
  for (unsigned index = 0; index < stream_pairs; ++index) {
    const auto previous = result.expected;
    if (result.operation == bit_xor) {
      bfu_model::apply(result.expected, previous, {.function = bfu_model::Function::bit_xor});
    } else {
      sequence_model::alu(result.expected, previous, result.operation);
    }
  }
  result.expected.pc = stream_words;
  result.output =
      partial ? partial_word(result.expected, 12) : static_cast<Word>(result.initial.x & word_mask);
  return result;
}
}

Timing workload(const std::string_view domain, const bool partial, const unsigned repetitions) {
  constexpr unsigned maximum_repetitions = 256;
  if (repetitions == 0 || repetitions > maximum_repetitions) {
    throw std::invalid_argument("Expected partial-store repetitions in [1,256]");
  }
  const auto sample = model(domain, partial);
  std::array<Word, stream_words> program{};
  for (std::size_t index = 0; index < stream_pairs; ++index) {
    program.at(2 * index) = sample.operation;
    program.at((2 * index) + 1) = partial ? store_partial : store_word;
  }
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core cpu({.program = program, .x = x, .y = y});
  Timing result;
  for (unsigned interval = 0; interval <= repetitions * intervals; ++interval) {
    cpu.set_registers(sample.initial);
    x[0] = sentinel;
    const auto before = cpu.progress();
    const auto counters = cpu.jit_statistics();
    const auto began = std::chrono::steady_clock::now();
    const auto run = cpu.run(stream_words);
    const auto finished = std::chrono::steady_clock::now();
    require(run.dispatches == stream_words && run.fault.kind == FaultKind::none &&
                cpu.registers() == sample.expected && x[0] == sample.output && y[0] == 0 &&
                cpu.progress() ==
                    Progress{.instructions = before.instructions + stream_words,
                             .estimated_cycles = before.estimated_cycles + stream_words},
            "Independent three-domain partial/data-word stream state, RAM and progress");
    if (interval == 0) { continue; } // Untimed native compilation and semantic warmup.
    const auto current = cpu.jit_statistics();
    result.seconds += std::chrono::duration<double>(finished - began).count();
    result.instructions += stream_words;
    result.native += current.native_alu_instructions - counters.native_alu_instructions;
    result.helpers += current.helper_instructions - counters.helper_instructions;
    result.moves += current.native_move_instructions - counters.native_move_instructions;
  }
  return result;
}
}
