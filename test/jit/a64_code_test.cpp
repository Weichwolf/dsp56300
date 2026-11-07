// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/code.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#if defined(__aarch64__)
#include "jit/arena.hpp"

#include <bit>
#include <type_traits>
#endif

namespace {
using namespace dsp56300::detail::a64;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::uint32_t instruction(const Code &code, const std::size_t index) {
  constexpr unsigned byte_bits = 8;
  std::uint32_t result = 0;
  for (unsigned byte = 0; byte < sizeof(result); ++byte) {
    result |= std::uint32_t{code.bytes().at((index * sizeof(result)) + byte)} << (byte * byte_bits);
  }
  return result;
}

void encodings() {
  Code code;
  code.alu(Alu::add, Reg::x0, Reg::x1, Reg::x2);
  code.alu(Alu::subtract_flags, Reg::x0, Reg::x1, Reg::x2);
  code.alu(Alu::bit_clear, Reg::x0, Reg::x1, Reg::x2);
  code.shift(Shift::left, Reg::x0, Reg::x1, 8);
  code.shift(Shift::right, Reg::x0, Reg::x1, 8);
  code.shift(Shift::arithmetic, Reg::x0, Reg::x1, 8);
  code.shift(Shift::rotate, Reg::x0, Reg::x1, 8);
  code.reverse_half(Reg::x0, Reg::x1);
  code.extract(Reg::x0, Reg::x1, 0, 8, Width::w32, true);
  code.load(Reg::x0, Reg::x1, 5, Access::byte);
  code.load(Reg::x0, Reg::x1, 6, Access::half);
  code.load(Reg::x0, Reg::x1, 8, Access::doubleword);
  code.push(Reg::x19, Reg::x20);
  code.pop(Reg::x19, Reg::x20);
  code.set(Reg::x0, Condition::equal);
  code.select(Reg::x0, Reg::x1, Reg::x2, Condition::equal);
  code.read_flags(Reg::x0);
  code.write_flags(Reg::x0);
  code.multiply(Reg::x0, Reg::x1, Reg::x2);
  code.divide(Reg::x0, Reg::x1, Reg::x2, true);
  code.divide(Reg::x0, Reg::x1, Reg::x2, false);
  code.multiply_subtract(Reg::x0, Reg::x1, Reg::x2, Reg::x3);
  code.call(Reg::x16);
  code.return_();
  constexpr std::array<std::uint32_t, 24> expected{
      0x0b020020, 0x6b020020, 0x0a220020, 0x53185c20, 0x53087c20, 0x13087c20,
      0x13812020, 0x5ac00420, 0x13001c20, 0x39401420, 0x79400c20, 0xf9400420,
      0xa9bf53f3, 0xa8c153f3, 0x1a9f17e0, 0x1a820020, 0xd53b4200, 0xd51b4200,
      0x1b027c20, 0x1ac20c20, 0x1ac20820, 0x1b028c20, 0xd63f0200, 0xd65f03c0};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index], "A64 encoding differs from ISA reference");
  }
}

void barrel_encodings() {
  Code code;
  for (const auto width : {Width::w32, Width::x64}) {
    for (const auto operation : {Shift::left, Shift::right, Shift::arithmetic, Shift::rotate}) {
      code.shift(operation, Reg::x0, Reg::x1, Reg::x2, width);
    }
  }
  constexpr std::array<std::uint32_t, 8> expected{0x1ac22020,
                                                  0x1ac22420,
                                                  0x1ac22820,
                                                  0x1ac22c20,
                                                  0x9ac22020,
                                                  0x9ac22420,
                                                  0x9ac22820,
                                                  0x9ac22c20};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "A64 variable-shift encoding differs from ISA");
  }
}

void fixups() {
  Code code;
  Label begin;
  Label end;
  code.bind(begin);
  code.branch(end);
  code.branch(Condition::not_equal, begin);
  code.branch_zero(Reg::x2, end);
  code.branch_zero(Reg::x3, begin, Width::x64, true);
  code.bind(end);
  constexpr std::array<std::uint32_t, 4> expected{0x14000004, 0x54ffffe1, 0x34000042, 0xb5ffffa3};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "A64 forward/backward branch fixup failed");
  }
  bool rejected = false;
  try {
    code.bind(end);
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected, "A64 labels must not be rebound");
}

#if defined(__aarch64__)
extern "C" std::uint64_t
dsp56300_a64_test_enter(const void *, std::uint64_t, std::uint64_t) noexcept;
#if defined(__APPLE__)
asm(".text\n"
    ".globl _dsp56300_a64_test_enter\n"
    "_dsp56300_a64_test_enter:\n"
    "mov x16,x0\n"
    "mov x0,x1\n"
    "mov x1,x2\n"
    "br x16\n");
#else
asm(".text\n"
    ".globl dsp56300_a64_test_enter\n"
    ".type dsp56300_a64_test_enter,%function\n"
    "dsp56300_a64_test_enter:\n"
    "mov x16,x0\n"
    "mov x0,x1\n"
    "mov x1,x2\n"
    "br x16\n");
#endif

template <class Integer>
Integer reference_shift(const Shift operation, const Integer value, const unsigned count) {
  constexpr unsigned bits = sizeof(Integer) * 8;
  const auto amount = count % bits;
  switch (operation) {
    case Shift::left: return value << amount;
    case Shift::right: return value >> amount;
    case Shift::arithmetic:
      return std::bit_cast<Integer>(std::bit_cast<std::make_signed_t<Integer>>(value) >> amount);
    case Shift::rotate: return std::rotr(value, static_cast<int>(amount));
    default: throw std::logic_error("Unknown A64 test shift");
  }
}

void execute_barrels() {
  dsp56300::detail::ExecutableArena arena;
  constexpr std::array<std::uint64_t, 6> values{
      0, 1, 0x80000000, 0xffffffff, 0x8000000000000000, 0xffffffffffffffff};
  constexpr std::array<unsigned, 9> counts{0, 1, 7, 31, 32, 63, 64, 65, 127};
  for (const auto width : {Width::w32, Width::x64}) {
    for (const auto operation : {Shift::left, Shift::right, Shift::arithmetic, Shift::rotate}) {
      Code code;
      code.shift(operation, Reg::x0, Reg::x0, Reg::x1, width);
      code.return_();
      const auto *entry = arena.append(code.bytes());
      for (const auto value : values) {
        for (const auto count : counts) {
          const auto expected = width == Width::x64
                                    ? reference_shift(operation, value, count)
                                    : std::uint64_t{reference_shift(
                                          operation, static_cast<std::uint32_t>(value), count)};
          require(dsp56300_a64_test_enter(entry, value, count) == expected,
                  "A64 variable shifts must preserve width, signedness and modulo counts");
        }
      }
    }
  }
}

void execute() {
  dsp56300::detail::ExecutableArena arena;
  Code code;
  Label again;
  code.constant(Reg::x2, 0);
  code.bind(again);
  code.alu(Alu::add, Reg::x2, Reg::x2, Reg::x1, Width::x64);
  code.constant(Reg::x3, 1);
  code.alu(Alu::subtract_flags, Reg::x0, Reg::x0, Reg::x3, Width::x64);
  code.branch(Condition::not_equal, again);
  code.move(Reg::x0, Reg::x2);
  code.return_();
  const auto *entry = arena.append(code.bytes());
  constexpr std::uint64_t count = 31;
  constexpr std::uint64_t increment = 0x123456789abcdef;
  require(dsp56300_a64_test_enter(entry, count, increment) == count * increment,
          "A64 native loop must preserve 64-bit operands and backward branches");
  Code negative;
  constexpr auto signed_value = std::int64_t{-123456789};
  constexpr unsigned shift = 17;
  negative.constant(Reg::x0, std::bit_cast<std::uint64_t>(signed_value));
  negative.shift(Shift::arithmetic, Reg::x0, Reg::x0, shift, Width::x64);
  negative.return_();
  require(dsp56300_a64_test_enter(arena.append(negative.bytes()), 0, 0) ==
              std::bit_cast<std::uint64_t>(signed_value >> shift),
          "A64 64-bit immediate construction and arithmetic shifts failed");
  require(dsp56300_a64_test_enter(entry, count, increment) == count * increment,
          "Publishing a second native function changed a retained translation");
}
#endif
}

int main() {
  try {
    encodings();
    fixups();
    barrel_encodings();
#if defined(__aarch64__)
    execute();
    execute_barrels();
#endif
    std::cout << "A64 encoding and branch checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
