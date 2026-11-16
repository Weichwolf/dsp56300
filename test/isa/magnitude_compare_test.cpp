// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
std::uint64_t cases = 0;
constexpr Word cmpm_a = static_cast<Word>(isa::Opcode::cmpm_b_a);
constexpr Word cmp_a = static_cast<Word>(isa::Opcode::cmp_b_a), destination_b = 8;
constexpr std::uint64_t fraction = std::uint64_t{1} << 47;
constexpr std::array modes{Word{0}, mode_down, mode_up};

struct Boundary {
  std::uint64_t destination = 0, source = 0;
  std::array<Word, 3> flags{};
  bool magnitude = true;
};

// FM13-47 and table5-1: literal normal/down/up E/U/N/Z/V/L outcomes.
// Mathematical magnitudes range through +2^55; -2^55 remains representable.
// These are manual-derived boundaries, not captured silicon observations.
constexpr std::array boundaries{
    Boundary{.source = sign56, .flags = {0x39, 0x39, 0x39}},
    Boundary{.destination = sign56, .flags = {0x7a, 0x7a, 0x7a}},
    Boundary{.destination = sign56, .source = sign56, .flags = {0x14, 0x14, 0x14}},
    Boundary{.destination = sign56, .source = 1, .flags = {0x30, 0x30, 0x30}},
    Boundary{.destination = 1, .source = sign56, .flags = {0x39, 0x39, 0x39}},
    Boundary{.destination = sign56, .source = sign56 - 1, .flags = {0x10, 0x10, 0x10}},
    Boundary{.destination = sign56 - 1, .source = sign56, .flags = {0x19, 0x19, 0x19}},
    Boundary{.source = sign56 - 1, .flags = {0x39, 0x39, 0x39}},
    Boundary{.destination = sign56 - 1, .flags = {0x30, 0x30, 0x30}},
    Boundary{.destination = fraction, .flags = {0x20, 0x00, 0x30}},
    Boundary{.source = fraction, .flags = {0x09, 0x19, 0x39}},
    Boundary{.destination = fraction, .source = fraction - 1, .flags = {0x10, 0x10, 0x10}},
    Boundary{.source = sign56, .flags = {0x7b, 0x7b, 0x7b}, .magnitude = false},
    Boundary{.destination = sign56, .flags = {0x38, 0x38, 0x38}, .magnitude = false},
    Boundary{
        .destination = sign56, .source = sign56, .flags = {0x14, 0x14, 0x14}, .magnitude = false}};

struct Input {
  Boundary boundary;
  std::size_t mode = 0;
  Word ccr = 0;
  bool accumulator_b = false;
};

Registers initial(const Input &input) {
  auto state = initial_state();
  state.a = input.accumulator_b ? input.boundary.source : input.boundary.destination;
  state.b = input.accumulator_b ? input.boundary.destination : input.boundary.source;
  constexpr std::uint64_t x_seed = 0x123456789abc;
  constexpr std::uint64_t y_seed = 0xabcdef543210;
  state.x = x_seed;
  state.y = y_seed;
  constexpr Word interrupt_mask = 0x300;
  constexpr Word register_seed = 0x120;
  constexpr Word offset = 3;
  state.sr = interrupt_mask | modes.at(input.mode) | input.ccr;
  state.sp = state.sc = offset;
  state.la = memory_words - 1;
  state.lc = register_seed;
  for (std::size_t index = 0; index < state.r.size(); ++index) {
    state.r[index] = register_seed + static_cast<Word>(index);
    state.n[index] = offset + static_cast<Word>(index);
  }
  for (std::size_t index = 0; index < state.stack.size(); ++index) {
    state.stack[index] = (std::uint64_t{register_seed} << word_bits) | index;
  }
  return state;
}

bool qualified_carry(const Boundary boundary) {
  const auto destination = signed56(boundary.destination);
  constexpr auto limit = static_cast<std::int64_t>(fraction);
  return !boundary.magnitude || (destination >= -limit && destination < limit);
}

void one(const Input &input, const NativeImage &image) {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const auto opcode =
      (input.boundary.magnitude ? cmpm_a : cmp_a) | (input.accumulator_b ? destination_b : 0);
  program[start] = opcode;
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  const auto variant = (input.boundary.magnitude ? 0U : 2U) + (input.accumulator_b ? 1U : 0U);
  core.install(image, variant);
  auto expected = initial(input);
  core.set_registers(expected);
  ++expected.pc;
  expected.sr = (expected.sr & ~status_byte) | (input.ccr & (flag_s | flag_l)) |
                input.boundary.flags.at(input.mode);
  const auto result = core.run(1);
  ++cases;
  require(result.retired == 1 && result.fault.kind == FaultKind::none,
          "Installed comparison retires the real instruction");
  auto observed = core.registers();
  if (!qualified_carry(input.boundary)) {
    // FM13-47 explicitly excludes improperly sign-extended CMPM destinations.
    expected.sr &= ~flag_c;
    observed.sr &= ~flag_c;
  }
  require(observed == expected,
          "Literal CMPM/CMP boundary preserves complete operands/registers/stack and exact flags");
  require(core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "All comparison paths retain the single-word nominal instruction cost");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    using namespace dsp56300::test;
    const std::array opcodes{cmpm_a, cmpm_a | destination_b, cmp_a, cmp_a | destination_b};
    const auto image = compile_operations(opcodes, start);
    for (const auto boundary : boundaries) {
      for (std::size_t mode = 0; mode < modes.size(); ++mode) {
        for (const auto ccr : {Word{0}, flag_c | flag_v, flag_s | flag_l, status_byte}) {
          for (const bool accumulator_b : {false, true}) {
            one({.boundary = boundary, .mode = mode, .ccr = ccr, .accumulator_b = accumulator_b},
                image);
          }
        }
      }
    }
    std::cout << checks << " literal comparison assertions across " << cases << " cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
