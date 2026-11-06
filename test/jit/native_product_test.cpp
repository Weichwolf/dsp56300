// SPDX-License-Identifier: GPL-3.0-only
// Independent raw56 witnesses for native product coordinates and old-zero sums.
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
#include "saturation_model.hpp"
#include "support.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array raw_values{std::uint64_t{0},
                                std::uint64_t{1},
                                sign56 - 1,
                                sign56,
                                accumulator_mask,
                                saturation_model::positive,
                                saturation_model::negative,
                                std::uint64_t{0x7e123456abcdef}};
constexpr std::array<Word, 4> flags{0, 0x55, 0xaa, status_byte};
constexpr std::array<Word, 6> extrema{0, 1, sign24 - 1, sign24, word_mask - 1, word_mask};
constexpr Word clear = 0x13, register_move = 0x200000;
constexpr unsigned source_shift = 13, destination_shift = 8;
constexpr std::uint64_t initial_instructions = 17, initial_cycles = 23;

struct Step {
  Word code;
  bool move = false;
  unsigned source = 0, destination = 0;
};

enum class Shape : std::uint8_t { plain, zero, other_accumulator, crossed, repeated, biased_low };

std::vector<Step> steps(const Word product, const Shape shape) {
  if (shape == Shape::plain) { return {{.code = parallel_none | product}}; }
  if (shape == Shape::repeated) { return std::vector<Step>(3, {.code = parallel_none | product}); }
  if (shape == Shape::biased_low) {
    const auto group = (product >> 4) & 7U;
    constexpr Word square_x0 = 0x80;
    constexpr Word square_y0 = 0x90;
    const auto square = group == 1 || group == 3 || group == 6 ? square_y0 : square_x0;
    std::vector<Step> result(4, {.code = parallel_none | square});
    result.push_back({.code = parallel_none | product});
    return result;
  }
  const Word destination = (product & 8U) != 0 ? 15U : 14U;
  std::vector<Step> result{{.code = parallel_none | clear | (product & 8U)}};
  if (shape == Shape::zero) {
    result.push_back({.code = parallel_none | product});
  } else {
    const bool crossed = shape == Shape::crossed;
    const unsigned source = crossed ? 4U : 5U;
    const unsigned target = crossed ? 6U : destination ^ 1U;
    const Word move = register_move | (source << source_shift) | (target << destination_shift);
    result.push_back(
        {.code = move | product, .move = true, .source = source, .destination = target});
  }
  return result;
}

std::vector<detail::BlockInstruction> operations(const std::span<const Step> body,
                                                 const Word mode) {
  std::vector<detail::BlockInstruction> result;
  Word pc = start;
  for (const auto step : body) {
    const auto decoded = detail::decode(step.code);
    require(decoded.operation == detail::Operation::parallel && decoded.words == 1 &&
                decoded.cycles == 1,
            "Declared product and register-move encodings have independent unit costs");
    result.push_back({.instruction = decoded, .pc = pc, .mode = mode});
    ++pc;
  }
  return result;
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

void check(const native::Direct &code, const std::span<const Step> body, const Registers &initial) {
  auto expected = initial;
  for (const auto step : body) {
    const auto old = expected;
    if (step.move) {
      register_model::write(
          expected, step.destination, register_model::read(expected, step.source));
    }
    if ((initial.sr & saturation_model::mode) != 0) {
      saturation_model::alu(expected, old, step.code);
    } else {
      sequence_model::alu(expected, old, step.code);
    }
    ++expected.pc;
  }
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = initial_instructions;
  cpu.cycles = initial_cycles;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Probe probe;
  bool dirty = false;
  std::array<std::uint64_t, 4> latched{};
  detail::NativeContext context{.runtime = {.owner = &probe,
                                            .latched = &latched,
                                            .execute = forbidden,
                                            .moves = forbidden,
                                            .memory = {}},
                                .exit_requested = &dirty,
                                .budget = static_cast<Word>(body.size())};
  const auto retired = code.enter(cpu, context);
  expected.a <<= detail::padding_bits;
  expected.b <<= detail::padding_bits;
  require(probe.calls == 0 && retired == body.size(),
          "Every product executes actual helper-free RX");
  require(cpu.reg == expected,
          "Native product matches independent complete raw56 registers and CCR");
  require(cpu.instructions == initial_instructions + body.size() &&
              cpu.cycles == initial_cycles + body.size() && !cpu.repeat_completed &&
              !cpu.interrupt_inhibited,
          "Product simplification preserves exact retirement, costs and canonical exit state");
  require(!dirty, "Parallel register moves cannot request an external-effect side exit");
}

struct Inputs {
  std::array<Word, 4> words;
  unsigned index = 0;
};

Registers state(const Word mode, const Inputs input) {
  auto result = register_model::seed(mode);
  result.sr = mode | flags.at(input.index % flags.size());
  result.a = raw_values.at(input.index % raw_values.size());
  result.b = raw_values.at((input.index + 3) % raw_values.size());
  result.x = (std::uint64_t{input.words[1]} << word_bits) | input.words[0];
  result.y = (std::uint64_t{input.words[3]} << word_bits) | input.words[2];
  return result;
}

void probes(const native::Direct &code, const std::span<const Step> body, const Word mode) {
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const Word one = Word{1} << bit;
    for (unsigned rotation = 0; rotation < 4; ++rotation) {
      for (const bool complement : {false, true}) {
        std::array<Word, 4> words{};
        const std::array values{one, one ^ word_mask, sign24, sign24 - 1};
        for (unsigned index = 0; index < words.size(); ++index) {
          words[index] =
              values.at((index + rotation) % values.size()) ^ (complement ? word_mask : 0);
        }
        check(code, body, state(mode, {.words = words, .index = bit + rotation}));
      }
    }
  }
  for (const auto left : extrema) {
    for (const auto right : extrema) {
      check(code, body, state(mode, {.words = {left, right, right, left}, .index = left % 8}));
    }
  }
}

void matrix(const Word mode) {
  for (Word product = 0x80; product <= status_byte; ++product) {
    for (const auto shape : {Shape::plain,
                             Shape::zero,
                             Shape::other_accumulator,
                             Shape::crossed,
                             Shape::repeated,
                             Shape::biased_low}) {
      const auto body = steps(product, shape);
      const auto ops = operations(body, mode);
      for (const auto ranges : {detail::RangeMode::automatic, detail::RangeMode::disabled}) {
        for (const auto words : {detail::WordReuseMode::automatic,
                                 detail::WordReuseMode::vectors,
                                 detail::WordReuseMode::disabled}) {
          detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}},
                                       {},
                                       detail::RegionMode::ordinary,
                                       detail::LoopResidency::disabled,
                                       ranges,
                                       words);
          const native::Direct code(emitter.generate(ops));
          probes(code, body, mode);
        }
      }
    }
  }
}

void forwarding(const Word mode) {
  constexpr auto multiply = isa::opcode<isa::Opcode::mpy_register_a>()[0];
  constexpr auto transfer = isa::opcode<isa::Opcode::move_register_x0_x1>()[0];
  constexpr Word register_fields = (31U << source_shift) | (31U << destination_shift);
  constexpr unsigned register_base = 4;
  constexpr unsigned other_accumulator = 15;
  for (unsigned pair = 0; pair < 8; ++pair) {
    const Word product = multiply | (pair << 4);
    for (unsigned word = 0; word < 4; ++word) {
      const unsigned source = register_base + ((word + 1) % 4);
      std::vector<Step> body(3, {.code = product});
      // Unrelated transfers retain the cached product words. A later word write
      // changes only subsequent products; its own ALU still reads the old word.
      for (const auto destination : {other_accumulator, register_base + word}) {
        const auto move = (transfer & ~register_fields) | (source << source_shift) |
                          (destination << destination_shift);
        body.push_back({.code = move | (product & status_byte),
                        .move = true,
                        .source = source,
                        .destination = destination});
        body.insert(body.end(), 3, {.code = product});
      }
      const auto ops = operations(body, mode);
      for (const auto words : {detail::WordReuseMode::automatic,
                               detail::WordReuseMode::vectors,
                               detail::WordReuseMode::disabled}) {
        detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}},
                                     {},
                                     detail::RegionMode::ordinary,
                                     detail::LoopResidency::disabled,
                                     detail::RangeMode::disabled,
                                     words);
        const native::Direct code(emitter.generate(ops));
        probes(code, body, mode);
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::matrix(mode);
      dsp56300::test::matrix(mode | dsp56300::test::saturation_model::mode);
      dsp56300::test::forwarding(mode);
      dsp56300::test::forwarding(mode | dsp56300::test::saturation_model::mode);
    }
    std::cout << dsp56300::test::checks << " independent native product-coordinate checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
