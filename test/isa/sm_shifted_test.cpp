// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "shifted_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
std::uint64_t cases = 0;

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
constexpr std::array operations{static_cast<Word>(isa::Opcode::addr_b_a),
                                static_cast<Word>(isa::Opcode::addl_b_a),
                                static_cast<Word>(isa::Opcode::subr_b_a),
                                static_cast<Word>(isa::Opcode::subl_b_a)};

struct Input {
  Operands operands;
  Word opcode;
  Word status;
  bool destination_b;
};

void one(const Input input, const NativeImage &image, const std::size_t operation) {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const Word opcode = input.opcode | (input.destination_b ? 8U : 0U);
  program[start] = opcode;
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  core.install(image, (operation * 2) + (input.destination_b ? 1U : 0U));
  auto expected = register_model::seed(input.status);
  expected.sr = input.status;
  expected.a = input.destination_b ? input.operands.source : input.operands.destination;
  expected.b = input.destination_b ? input.operands.destination : input.operands.source;
  core.set_registers(expected);
  const bool qualified_carry = shifted_model::apply(expected, opcode);
  ++expected.pc;
  const auto result = core.run(1);
  ++cases;
  require(result.retired == 1 && result.fault.kind == FaultKind::none,
          "Installed shifted arithmetic retires the real instruction");
  auto actual = core.registers();
  if (!qualified_carry) {
    // FM13-9/174 qualifies C only when the initial left shift did not overflow.
    expected.sr &= ~flag_c;
    actual.sr &= ~flag_c;
  }
  require(actual == expected,
          "Signed mathematical shift, independent saturation/flags and whole state agree");
  require(core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Shifted arithmetic retains its literal single-word base cost");
}

void variants(Input input, const NativeImage &image, const std::size_t operation) {
  for (const auto scale : {Word{0}, mode_down, mode_up}) {
    for (const auto ccr : {Word{0}, flag_c | flag_v, flag_s | flag_l, status_byte}) {
      for (const bool destination_b : {false, true}) {
        input.status = saturation_model::mode | scale | ccr;
        input.destination_b = destination_b;
        one(input, image, operation);
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
    std::vector<Word> opcodes;
    for (const auto opcode : operations) {
      opcodes.push_back(opcode);
      opcodes.push_back(opcode | 8U);
    }
    const auto image = compile_operations(opcodes, start);
    for (const auto operands : values) {
      for (std::size_t operation = 0; operation < operations.size(); ++operation) {
        variants({.operands = operands,
                  .opcode = operations[operation],
                  .status = 0,
                  .destination_b = false},
                 image,
                 operation);
      }
    }
    std::cout << checks << " SM shifted-arithmetic assertions across " << cases
              << " cases passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
