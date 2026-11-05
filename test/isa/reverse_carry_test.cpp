// SPDX-License-Identifier: GPL-3.0-only
// Exhaustive eight-bit subdomains and full-width chains against a serial circuit.
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word update = 0x204000, positive = 0x0800, byte_mask = 0xff;
constexpr Word left_padding = 0xa55aa5, ignored_modifier = 0xab0000;
constexpr Word domain_size = 256;
std::uint64_t cases = 0;

struct Probe {
  Word cursor = 0, operand = 0;
};

class Matrix {
public:
  Matrix(const Backend backend, const bool subtract)
      : core_({.program = p_, .x = x_, .y = y_}, nullptr, {.backend = backend}),
        seed_(register_model::seed(0)),
        subtract_(subtract) {
    core_.write(Space::program, start, update | (subtract ? 0U : positive));
    seed_.sr = status_byte;
    seed_.m[0] = ignored_modifier;
  }

  void check(const Probe input) {
    auto initial = seed_;
    initial.r[0] = input.cursor;
    initial.n[0] = input.operand;
    auto expected = initial;
    expected.r[0] = address_model::reverse_carry(
        {.original = input.cursor, .operand = input.operand, .subtract = subtract_});
    ++expected.pc;
    core_.set_registers(initial);
    const auto result = core_.run(1);
    ++retired_;
    ++cases;
    require(
        result.dispatches == 1 && result.fault.kind == FaultKind::none &&
            result.progress == Progress{.instructions = retired_, .estimated_cycles = retired_} &&
            core_.registers() == expected && core_.interruptible(),
        "Reverse-carry update matches a serial MSB-first carry/borrow circuit and complete state");
  }

private:
  std::array<Word, memory_words> p_{};
  std::array<Word, memory_words> x_{};
  std::array<Word, memory_words> y_{};
  Core core_;
  Registers seed_;
  std::uint64_t retired_ = 0;
  bool subtract_ = false;
};

void dense(Matrix &matrix) {
  for (const unsigned position : {0U, 8U, 16U}) {
    const Word mask = byte_mask << position;
    for (Word left = 0; left < domain_size; ++left) {
      for (Word right = 0; right < domain_size; ++right) {
        matrix.check(
            {.cursor = (left_padding & ~mask) | (left << position), .operand = right << position});
      }
    }
  }
}

void chains(Matrix &matrix, const bool subtract) {
  for (unsigned width = 1; width <= word_bits; ++width) {
    const Word mask = (Word{1} << width) - 1;
    const Word increment = Word{1} << (width - 1);
    for (unsigned count = 0; count <= width; ++count) {
      const Word chain = ((Word{1} << count) - 1) << (width - count);
      const Word cursor = (left_padding & ~mask) | (subtract ? (~chain & mask) : chain);
      for (const Word operand : {increment, increment | 1U, mask}) {
        matrix.check({.cursor = cursor, .operand = operand});
      }
    }
  }
  for (const Word cursor : {0U, 1U, sign24 - 1, sign24, word_mask}) {
    for (const Word operand : {0U, 1U, sign24 - 1, sign24, sign24 + 1, word_mask}) {
      matrix.check({.cursor = cursor, .operand = operand});
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      for (const bool subtract : {false, true}) {
        dsp56300::test::Matrix matrix(backend, subtract);
        dsp56300::test::dense(matrix);
        dsp56300::test::chains(matrix, subtract);
      }
    }
    std::cout << dsp56300::test::cases << " independent reverse-carry architectural cases passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
