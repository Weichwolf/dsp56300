// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "scripted_io.hpp"
#include "shifted_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
enum class Path : std::uint8_t { dispatched, owned, captured };

struct Operands {
  std::uint64_t destination;
  std::uint64_t source;
};

constexpr std::array values{Operands{.destination = 0, .source = 0},
                            Operands{.destination = 1, .source = 1},
                            Operands{.destination = accumulator_mask, .source = 1},
                            Operands{.destination = sign56, .source = 1},
                            Operands{.destination = sign56 - 1, .source = accumulator_mask},
                            Operands{.destination = sign56 - 1, .source = sign56},
                            Operands{.destination = 0, .source = sign56},
                            Operands{.destination = sign56, .source = sign56},
                            Operands{.destination = saturation_model::positive, .source = 1},
                            Operands{.destination = saturation_model::negative, .source = 1},
                            Operands{.destination = 0x01800000000000, .source = 0},
                            Operands{.destination = 0x80800000000000, .source = accumulator_mask}};
// FM13 diagrams: ADDR/ADDL/SUBR/SUBL, D=A; D=B is bit3.
constexpr std::array operations{0x200002U, 0x200012U, 0x200006U, 0x200016U};

struct Input {
  Operands operands;
  Word opcode;
  Word status;
  Backend backend;
  Path path;
  bool destination_b;
};

void execute(Core &core, const Input input, const Word opcode) {
  const auto result = input.path == Path::captured
                          ? core.run_instruction({.address = start, .opcode = opcode})
                          : core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none,
          "Dispatched, externally fetched and captured execution retire the real instruction");
}

void one(const Input input) {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const Word opcode = input.opcode | (input.destination_b ? 8U : 0U);
  program[start] = opcode;
  scripted_io::Bus bus;
  bus.pc = start;
  if (input.path == Path::owned) {
    bus.events = {{.space = Space::program, .address = start, .value = opcode}};
  }
  Core core(input.path == Path::owned ? Memory::external()
                                      : Memory::separate({.program = program, .x = x, .y = y}),
            &bus,
            {.backend = input.backend});
  auto expected = register_model::seed(input.status);
  expected.sr = input.status;
  expected.a = input.destination_b ? input.operands.source : input.operands.destination;
  expected.b = input.destination_b ? input.operands.destination : input.operands.source;
  core.set_registers(expected);
  const bool qualified_carry = shifted_model::apply(expected, opcode);
  ++expected.pc;
  execute(core, input, opcode);
  auto actual = core.registers();
  if (!qualified_carry) {
    // FM13-9/174 qualifies C only when the initial left shift did not overflow.
    expected.sr &= ~flag_c;
    actual.sr &= ~flag_c;
  }
  require(actual == expected,
          "Signed mathematical shift, independent saturation/flags and whole state agree");
  require(core.progress() == Progress{.instructions = 1, .estimated_cycles = 1} && bus.exact &&
              bus.calls == (input.path == Path::owned ? 1U : 0U),
          "Shifted arithmetic retains its literal single-word base cost and exact fetch");
}

void variants(Input input) {
  for (const auto scale : {Word{0}, mode_down, mode_up}) {
    for (const auto ccr : {Word{0}, flag_c | flag_v, flag_s | flag_l, status_byte}) {
      for (const bool destination_b : {false, true}) {
        input.status = saturation_model::mode | scale | ccr;
        input.destination_b = destination_b;
        one(input);
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    using namespace dsp56300::test;
    for (const auto operands : values) {
      for (const auto opcode : operations) {
        for (const auto backend : scripted_io::backends) {
          for (const auto path : {Path::dispatched, Path::owned, Path::captured}) {
            variants({.operands = operands,
                      .opcode = opcode,
                      .status = 0,
                      .backend = backend,
                      .path = path,
                      .destination_b = false});
          }
        }
      }
    }
    std::cout << checks << " SM shifted-arithmetic checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
