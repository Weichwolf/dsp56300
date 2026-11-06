// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
enum class Kind : std::uint8_t { fused, mac, round };
enum class Method : std::uint8_t { ordinary, captured };
constexpr Word down = 0x400, rm = 0x200000, changed = 0x3e;
constexpr std::uint64_t wrapped = 0x7fffffff000000, other = 0x123456789abcde;

constexpr std::array backends{Backend::jit};

struct Input {
  Kind kind;
  Word incoming;
  bool biased, destination_b;
};

Word opcode(const Input input) {
  constexpr Word mac = isa::opcode<isa::Opcode::mac_register_a>()[0] | (5U << 4);
  constexpr Word rnd = isa::opcode<isa::Opcode::rnd_a>()[0];
  const auto code = input.kind == Kind::round ? rnd : mac | (input.kind == Kind::fused ? 1U : 0U);
  return code | (input.destination_b ? 8U : 0U);
}

void captured(TestCore &cpu, const Input input) {
  const auto result = cpu.run_instruction({.address = start, .opcode = opcode(input)});
  require(result.retired == 1 && result.fault.kind == FaultKind::none,
          "The captured arithmetic retires its real complete fixed-point operation");
}

void check(const Input input, const Backend backend, const Method method) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  p[start] = opcode(input);
  TestCore cpu({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  auto before = initial_state();
  before.sr = down | input.incoming | (input.biased ? rm : 0U);
  before.x = 1;
  before.y = sign24;
  const auto value = input.kind == Kind::round ? wrapped : sign56;
  before.a = input.destination_b ? other : value;
  before.b = input.destination_b ? value : other;
  cpu.set_registers(before);
  auto wanted = before;
  ++wanted.pc;
  const auto output = input.kind == Kind::mac ? wrapped : sign56;
  (input.destination_b ? wanted.b : wanted.a) = output;
  constexpr Word fused_flags = 0x38;
  constexpr Word mac_flags = 0x72;
  constexpr Word round_flags = 0x7a;
  Word flags = fused_flags;
  if (input.kind == Kind::mac) { flags = mac_flags; }
  if (input.kind == Kind::round) { flags = round_flags; }
  wanted.sr = (before.sr & ~changed) | flags;
  if (method == Method::captured) {
    captured(cpu, input);
  } else {
    const auto run = cpu.run(1);
    require(run.retired == 1 && run.fault.kind == FaultKind::none,
            "The actual authored instruction retires without a synthetic result");
  }
  require(cpu.registers() == wanted &&
              cpu.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Literal fused cancellation and independent MAC/RND controls preserve complete state");
}

void cases() {
  for (const auto kind : {Kind::fused, Kind::mac, Kind::round}) {
    for (const Word incoming : {0U, flag_c | flag_v, status_byte}) {
      for (const bool biased : {false, true}) {
        for (const bool destination_b : {false, true}) {
          const Input input{
              .kind = kind, .incoming = incoming, .biased = biased, .destination_b = destination_b};
          for (const auto backend : backends) {
            check(input, backend, Method::ordinary);
            check(input, backend, Method::captured);
          }
        }
      }
    }
  }
}

struct Boundary {
  Word mode;
  std::uint64_t raw;
  Word count;
};

struct RoundingMachine {
  RoundingMachine() : core(Memory::separate({.program = p, .x = x, .y = y})) {}

  void check(const Boundary input) {
    const auto mode = input.mode;
    const auto raw = input.raw;
    const auto count = input.count;
    auto expected = register_model::seed(mode);
    expected.pc = 0;
    expected.sr = mode;
    expected.a = raw;
    expected.b = raw ^ accumulator_mask;
    core.set_registers(expected);
    const auto before = core.state().progress;
    for (const auto word : std::span<const Word>(p).first(count)) {
      const auto old = expected;
      if ((mode & saturation_model::mode) != 0) {
        saturation_model::alu(expected, old, word);
      } else {
        sequence_model::alu(expected, old, word);
      }
      ++expected.pc;
    }
    const auto run = core.run(count);
    require(run.retired == count && run.fault.kind == FaultKind::none && !run.yielded &&
                run.state == ExecutionState::running && core.registers() == expected &&
                run.progress == Progress{.instructions = before.instructions + count,
                                         .estimated_cycles = before.estimated_cycles + count},
            "Public RND boundaries retain ties, signed wrapping, saturation and complete status");
    require(p == std::array{isa::opcode<isa::Opcode::rnd_a>()[0],
                            isa::opcode<isa::Opcode::rnd_b>()[0]} &&
                x == std::array<Word, 1>{} && y == x,
            "Rounding leaves all P/X/Y memory intact");
  }

  std::array<Word, 2> p{isa::opcode<isa::Opcode::rnd_a>()[0], isa::opcode<isa::Opcode::rnd_b>()[0]};
  std::array<Word, 1> x{}, y{};
  Core core;
};

void round_boundaries() {
  RoundingMachine machine;
  for (const Word scale : {0U, mode_down, mode_up}) {
    const unsigned bits = word_bits + (scale == mode_down ? 1U : 0U) - (scale == mode_up ? 1U : 0U);
    const auto unit = std::int64_t{1} << bits;
    const auto maximum_whole = static_cast<std::int64_t>(sign56 - 1) / unit;
    const std::array<std::int64_t, 7> whole_values{
        -2, -1, 0, 1, 2, -maximum_whole - 1, maximum_whole};
    for (const Word rounding : {0U, mode_rm}) {
      for (const Word saturation : {0U, saturation_model::mode}) {
        for (const Word flags : {0U, flag_c | flag_v, status_byte}) {
          const auto mode = scale | rounding | saturation | flags;
          for (const auto whole : whole_values) {
            for (const auto remainder :
                 {std::int64_t{0}, (unit / 2) - 1, unit / 2, (unit / 2) + 1, unit - 1}) {
              const auto raw =
                  static_cast<std::uint64_t>((whole * unit) + remainder) & accumulator_mask;
              machine.check({.mode = mode, .raw = raw, .count = 1});
              machine.check({.mode = mode, .raw = raw, .count = 2});
            }
          }
        }
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::cases();
    dsp56300::test::round_boundaries();
    std::cout << dsp56300::test::checks << " independent rounded-product checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
