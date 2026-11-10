// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>
#if defined(__aarch64__)
#include "jit/arena.hpp"

#include <algorithm>
#include <bit>
#include <limits>
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

void logical_encodings() {
  constexpr std::uint64_t word_mask = 0x00ffffff;
  constexpr std::uint64_t repeated_bytes = 0xff00ff00ff00ff00;
  constexpr std::uint64_t wrapped_word = 0x80000001;
  constexpr std::uint64_t high_run = 0xffffffffffffff00;
  constexpr std::uint64_t middle_run = 0x00ffffffffffff00;
  constexpr std::uint64_t upper_word = 0xffff0000;

  Code code;
  require(code.logical(Alu::bit_or, Reg::x0, Reg::zr, word_mask, Width::x64) &&
              code.logical(Alu::bit_and, Reg::x0, Reg::x1, word_mask) &&
              code.logical(Alu::bit_and, Reg::x0, Reg::x1, repeated_bytes, Width::x64) &&
              code.logical(Alu::bit_or, Reg::x0, Reg::x1, wrapped_word) &&
              code.logical(Alu::bit_xor, Reg::x0, Reg::x1, high_run, Width::x64) &&
              code.logical(Alu::bit_and, Reg::x0, Reg::x1, middle_run, Width::x64),
          "Architectural logical immediate masks must be encodable");
  constexpr std::array<std::uint32_t, 6> expected{
      0xb2405fe0, 0x12005c20, 0x92089c20, 0x32010420, 0xd278dc20, 0x9278bc20};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "Logical immediate encoding differs from independent LLVM assembler bytes");
  }
  const auto size = code.size();
  for (const auto value : {std::uint64_t{0},
                           std::uint64_t{0xffffffff},
                           std::uint64_t{0x12345678},
                           ~std::uint64_t{0}}) {
    require(!code.logical(Alu::bit_and, Reg::x0, Reg::x1, value),
            "Unsupported word masks must remain explicit");
  }
  require(!code.logical(Alu::bit_and, Reg::zr, Reg::x1, 1) && code.size() == size,
          "Rejected immediates must leave code untouched and never encode ZR as SP");
  Code constants;
  constants.constant(Reg::x0, ~std::uint64_t{0});
  constants.constant(Reg::x0, high_run);
  constants.constant(Reg::x0, 0x8000000000000000);
  constants.constant(Reg::x0, upper_word, Width::w32);
  constants.constant(Reg::x0, word_mask);
  constexpr std::array<std::uint32_t, 5> materialized{
      0x92800000, 0x92801fe0, 0xd2f00000, 0x52bfffe0, 0xb2405fe0};
  require(constants.size() == materialized.size() * sizeof(std::uint32_t),
          "Directed constants must use one instruction each");
  for (std::size_t index = 0; index < materialized.size(); ++index) {
    require(instruction(constants, index) == materialized[index],
            "Constant materialization differs from independent LLVM assembler bytes");
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

void bit_encodings() {
  Code code;
  code.count_leading_zeroes(Reg::x0, Reg::x1);
  code.count_leading_zeroes(Reg::x0, Reg::x1, Width::x64);
  code.reverse_bits(Reg::x0, Reg::x1);
  code.reverse_bits(Reg::x0, Reg::x1, Width::x64);
  constexpr std::array<std::uint32_t, 4> expected{0x5ac01020, 0xdac01020, 0x5ac00020, 0xdac00020};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index], "A64 CLZ/RBIT encoding differs from ISA");
  }
}

void packed_encodings() {
  Code code;
  Neon neon(code);
  neon.binary(VectorOp::add_words, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::subtract_words, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::multiply_words, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::multiply_upper, Vec{0}, Vec{1}, Vec{2});
  neon.extend(Vec{0}, Vec{1}, true);
  neon.narrow(Vec{0}, Vec{1}, true);
  neon.broadcast(Vec{0}, Reg::x1, true);
  neon.insert(Vec{0}, Reg::x1, 3, true);
  neon.extract(Reg::x0, Vec{1}, 3, true);
  constexpr unsigned rounding_shift = 23;
  neon.round_narrow(Vec{0}, Vec{1}, rounding_shift);
  neon.round_narrow(Vec{0}, Vec{1}, rounding_shift, true);
  constexpr std::array<std::uint32_t, 11> expected{0x4ea28420,
                                                   0x6ea28420,
                                                   0x4ea29c20,
                                                   0x4ea2c020,
                                                   0x4f20a420,
                                                   0x4ea12820,
                                                   0x4e040c20,
                                                   0x4e1c1c20,
                                                   0x0e1c3c20,
                                                   0x0f298c20,
                                                   0x4f298c20};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index], "Packed NEON encoding differs from ISA");
  }
}

void accumulate_encodings() {
  Code code;
  Neon neon(code);
  neon.binary(VectorOp::multiply_add, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::multiply_add_upper, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::multiply_subtract, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::multiply_subtract_upper, Vec{0}, Vec{1}, Vec{2});
  constexpr std::array<std::uint32_t, 4> expected{0x0ea28020, 0x4ea28020, 0x0ea2a020, 0x4ea2a020};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "NEON widening accumulation differs from ISA");
  }
}

void stack_encodings() {
  Code code;
  Neon neon(code);
  neon.push_doubles(Vec{8}, Vec{9});
  neon.pop_doubles(Vec{8}, Vec{9});
  constexpr std::array<std::uint32_t, 2> expected{0x6dbf27e8, 0x6cc127e8};
  require(instruction(code, 0) == expected[0] && instruction(code, 1) == expected[1],
          "NEON paired doubleword stack operations differ from ISA");
}

void subtract_encodings() {
  constexpr std::uint16_t maximum = 0xfff;
  constexpr std::array<std::uint32_t, 2> expected{0xd10203ff, 0x513ffc20};
  Code code;
  code.subtract(Reg::sp, Reg::sp, 128, Width::x64);
  code.subtract(Reg::x0, Reg::x1, maximum);
  require(instruction(code, 0) == expected[0] && instruction(code, 1) == expected[1],
          "Subtract immediate must encode SP and full 12-bit word immediates exactly");
  bool rejected = false;
  try {
    code.subtract(Reg::sp, Reg::sp, 0x1000, Width::x64);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && code.size() == 8, "Oversized subtraction must leave emitted code intact");
}

void selection_encodings() {
  Code code;
  Neon neon(code);
  constexpr Vec first{24};
  neon.binary(VectorOp::greater, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::bit_insert, Vec{0}, Vec{1}, Vec{2});
  neon.load_interleaved(first, Reg::x10);
  neon.store_interleaved(first, Reg::x10);
  neon.binary(VectorOp::unzip_even, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::unzip_odd, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::zip_low, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::zip_high, Vec{0}, Vec{1}, Vec{2});
  constexpr std::array<std::uint32_t, 8> expected{0x4ee23420,
                                                  0x6ea21c20,
                                                  0x4c400958,
                                                  0x4c000958,
                                                  0x4e821820,
                                                  0x4e825820,
                                                  0x4e823820,
                                                  0x4e827820};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "NEON comparison and insertion differ from assembler reference");
  }
}

void immediate_encodings() {
  constexpr std::array<std::uint32_t, 6> values{
      1, 0x007fffff, 0xff800000, 0x80000000, 0x12ff, 0xffffedff};
  constexpr std::array<std::uint32_t, 6> expected{
      0x4f000420, 0x4f03d7e0, 0x6f03d7e0, 0x4f046400, 0x4f00c640, 0x6f002640};
  Code code;
  Neon neon(code);
  for (const auto value : values) {
    require(neon.constant_words(Vec{0}, value), "Directed NEON immediate must be encodable");
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index], "NEON immediate differs from ISA");
  }
}

void neon_encodings() {
  Code code;
  Neon neon(code);
  neon.binary(VectorOp::add, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::subtract, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::bit_and, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::bit_or, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::bit_xor, Vec{0}, Vec{1}, Vec{2});
  neon.shift(Vec{0}, Vec{1}, Shift::left, 9);
  neon.shift(Vec{0}, Vec{1}, Shift::right, 9);
  neon.shift(Vec{0}, Vec{1}, Shift::arithmetic, 9);
  neon.shift(Vec{0}, Vec{1}, Shift::left, 8, true);
  neon.shift(Vec{0}, Vec{1}, Shift::arithmetic, 8, true);
  neon.narrow(Vec{0}, Vec{1});
  neon.extend(Vec{0}, Vec{1});
  neon.binary(VectorOp::multiply, Vec{0}, Vec{1}, Vec{2});
  neon.broadcast(Vec{0}, Reg::x1);
  neon.insert(Vec{0}, Reg::x1, 1);
  neon.extract(Reg::x1, Vec{0}, 1);
  neon.load(Vec{0}, Reg::x1, 32);
  neon.store(Vec{0}, Reg::x1, 32);
  neon.load(Vec{0}, Reg::x1, 8, true);
  neon.binary(VectorOp::maximum, Vec{0}, Vec{1}, Vec{2});
  neon.binary(VectorOp::minimum, Vec{0}, Vec{1}, Vec{2});
  constexpr std::array<std::uint32_t, 21> expected{
      0x4ee28420, 0x6ee28420, 0x4e221c20, 0x4ea21c20, 0x6e221c20, 0x4f495420, 0x6f770420,
      0x4f770420, 0x4f285420, 0x4f380420, 0x0ea12820, 0x0f20a420, 0x0ea2c020, 0x4e080c20,
      0x4e181c20, 0x4e183c01, 0x3dc00820, 0x3d800820, 0xfd400420, 0x4ea26420, 0x4ea26c20};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "NEON encoding differs from independent assembler reference");
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

void near_fixups() {
  Code code(BranchRange::near);
  Label begin;
  Label end;
  code.bind(begin);
  code.branch_far(Condition::equal, end);
  code.branch_zero_far(Reg::x0, begin, Width::x64, true);
  code.branch_far(Condition::always, end);
  code.constant(Reg::x0, 7);
  code.bind(end);
  code.return_();
  constexpr std::array<std::uint32_t, 5> expected{
      0x54000080, 0xb5ffffe0, 0x14000002, 0xd28000e0, 0xd65f03c0};
  require(code.size() == expected.size() * sizeof(std::uint32_t),
          "Near branches emit one instruction without padding or inverse trampolines");
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require(instruction(code, index) == expected[index],
            "Near forward/backward fixups match independent assembler encodings");
  }
}

void branch_ranges() {
  const std::vector<std::uint8_t> padding(Code::near_branch_bytes);
  for (const auto range : {BranchRange::full, BranchRange::near}) {
    Code code(range);
    Label end;
    code.branch_zero_far(Reg::x0, end, Width::x64, true);
    code.append(padding);
    bool rejected = false;
    try {
      code.bind(end);
    } catch (const std::length_error &) { rejected = true; }
    require(rejected == (range == BranchRange::near),
            "Only full-range trampolines accept targets outside the signed19 branch range");
  }
  Code backward(BranchRange::near);
  Label begin;
  backward.bind(begin);
  backward.append(padding);
  backward.branch_far(Condition::equal, begin);
  constexpr std::uint32_t minimum_delta_equal = 0x54800000;
  require(instruction(backward, padding.size() / sizeof(std::uint32_t)) == minimum_delta_equal,
          "The exact negative19 boundary remains encodable");
  bool rejected = false;
  try {
    backward.branch_far(Condition::equal, begin);
  } catch (const std::length_error &) { rejected = true; }
  require(rejected, "A backward branch one instruction past the negative19 limit must fail");
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

std::uint64_t
logical_result(const Alu operation, const std::uint64_t input, const std::uint64_t mask) {
  if (operation == Alu::bit_and) { return input & mask; }
  if (operation == Alu::bit_or) { return input | mask; }
  if (operation == Alu::bit_xor) { return input ^ mask; }
  throw std::logic_error("Invalid expected logical operation");
}

void execute_logical_mask(dsp56300::detail::ExecutableArena &arena,
                          const Width width,
                          const std::uint64_t mask,
                          const Alu operation) {
  Code code;
  code.load(Reg::x2, Reg::x0, 3 * sizeof(std::uint64_t), Access::doubleword);
  code.write_flags(Reg::x2);
  code.constant(Reg::x2, mask, width);
  code.store(Reg::x2, Reg::x0, 2 * sizeof(std::uint64_t), Access::doubleword);
  require(code.logical(operation, Reg::x1, Reg::x1, mask, width),
          "Every independently constructed rotated/repeated mask must encode");
  code.store(Reg::x1, Reg::x0, 0, Access::doubleword);
  code.read_flags(Reg::x2);
  code.store(Reg::x2, Reg::x0, sizeof(std::uint64_t), Access::doubleword);
  code.return_();
  const auto *entry = arena.append(code.bytes());
  constexpr std::array<std::uint64_t, 4> inputs{
      0, ~std::uint64_t{0}, 0xa55a3cc369960ff0, 0x5aa5c33c9669f00f};
  for (const auto input : inputs) {
    for (unsigned status = 0; status < 16; ++status) {
      constexpr unsigned flags_shift = 28;
      const auto flags = std::uint64_t{status} << flags_shift;
      std::array<std::uint64_t, 4> output{0, 0, 0, flags};
      static_cast<void>(
          dsp56300_a64_test_enter(entry, reinterpret_cast<std::uintptr_t>(output.data()), input));
      auto expected = logical_result(operation, input, mask);
      if (width == Width::w32) { expected &= std::numeric_limits<std::uint32_t>::max(); }
      require(output[0] == expected && output[1] == flags && output[2] == mask,
              "Immediate logic/constants must preserve every result bit, width and NZCV");
    }
  }
}

void execute_logical_width(dsp56300::detail::ExecutableArena &arena, const Width width) {
  const unsigned bits = width == Width::x64 ? 64U : 32U;
  for (unsigned period = 2; period <= bits; period *= 2) {
    for (unsigned ones = 1; ones < period; ++ones) {
      for (unsigned rotation = 0; rotation < period; ++rotation) {
        std::uint64_t mask = 0;
        for (unsigned bit = 0; bit < bits; ++bit) {
          if ((bit + rotation) % period < ones) { mask |= std::uint64_t{1} << bit; }
        }
        for (const auto operation : {Alu::bit_and, Alu::bit_or, Alu::bit_xor}) {
          execute_logical_mask(arena, width, mask, operation);
        }
      }
    }
  }
}

void execute_constants() {
  dsp56300::detail::ExecutableArena arena;
  constexpr std::array<std::uint64_t, 4> halves{0, 0xffff, 0x1234, 0x8000};
  constexpr unsigned patterns = 256;
  constexpr unsigned half_bits = 16;
  constexpr unsigned half_mask = 3;
  for (const auto width : {Width::w32, Width::x64}) {
    const unsigned lanes = width == Width::x64 ? 4U : 2U;
    for (unsigned pattern = 0; pattern < patterns; ++pattern) {
      std::uint64_t value = 0;
      for (unsigned lane = 0; lane < lanes; ++lane) {
        value |= halves[(pattern >> (lane * 2)) & half_mask] << (lane * half_bits);
      }
      Code code;
      code.write_flags(Reg::x1);
      code.constant(Reg::x1, value, width);
      code.store(Reg::x1, Reg::x0, 0, Access::doubleword);
      code.read_flags(Reg::x2);
      code.store(Reg::x2, Reg::x0, sizeof(std::uint64_t), Access::doubleword);
      code.return_();
      const auto *entry = arena.append(code.bytes());
      for (unsigned status = 0; status < 16; ++status) {
        constexpr unsigned flags_shift = 28;
        const auto flags = std::uint64_t{status} << flags_shift;
        std::array<std::uint64_t, 2> output{};
        static_cast<void>(
            dsp56300_a64_test_enter(entry, reinterpret_cast<std::uintptr_t>(output.data()), flags));
        require(output[0] == value && output[1] == flags,
                "MOVZ/MOVN/MOVK construction must preserve every lane and NZCV");
      }
    }
    execute_logical_width(arena, width);
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

void execute_bits() {
  dsp56300::detail::ExecutableArena arena;
  constexpr std::array<std::uint64_t, 7> values{
      0, 1, 0x80000000, 0xffffffff, 0x123456789abcdef, 0x8000000000000000, 0xffffffffffffffff};
  for (const auto width : {Width::w32, Width::x64}) {
    const unsigned bits = width == Width::x64 ? 64U : 32U;
    Code reversed;
    reversed.reverse_bits(Reg::x0, Reg::x0, width);
    reversed.return_();
    const auto *reverse_entry = arena.append(reversed.bytes());
    Code leading;
    leading.count_leading_zeroes(Reg::x0, Reg::x0, width);
    leading.return_();
    const auto *leading_entry = arena.append(leading.bytes());
    for (const auto value : values) {
      std::uint64_t expected = 0;
      for (unsigned bit = 0; bit < bits; ++bit) {
        expected = (expected << 1) | ((value >> bit) & 1U);
      }
      require(dsp56300_a64_test_enter(reverse_entry, value, 0) == expected,
              "A64 RBIT must reverse exactly the selected width");
      const auto zeros = width == Width::x64 ? std::countl_zero(value)
                                             : std::countl_zero(static_cast<std::uint32_t>(value));
      require(dsp56300_a64_test_enter(leading_entry, value, 0) == static_cast<std::uint64_t>(zeros),
              "A64 CLZ must retain its zero-input and width semantics");
    }
  }
}

void execute_stack() {
  constexpr std::uint64_t left = 0x0123456789abcdef;
  constexpr std::uint64_t right = 0xfedcba9876543210;
  std::array<std::uint64_t, 6> data{left, 0, right, 0, 0, 0};
  Code code;
  Neon neon(code);
  neon.load(Vec{0}, Reg::x0);
  neon.load(Vec{1}, Reg::x0, 16);
  neon.push_doubles(Vec{0}, Vec{1});
  neon.zero(Vec{0});
  neon.zero(Vec{1});
  neon.pop_doubles(Vec{0}, Vec{1});
  neon.store(Vec{0}, Reg::x0, 32, true);
  neon.store(Vec{1}, Reg::x0, 5 * sizeof(std::uint64_t), true);
  code.load(Reg::x0, Reg::x0, 32, Access::doubleword);
  code.return_();
  dsp56300::detail::ExecutableArena arena;
  require(dsp56300_a64_test_enter(arena.append(code.bytes()),
                                  reinterpret_cast<std::uintptr_t>(data.data()),
                                  0) == left &&
              data[4] == left && data[5] == right,
          "NEON paired stack operations preserve both doublewords and balance SP");
}

void execute_immediates() {
  constexpr std::array<std::uint32_t, 15> values{0,
                                                 1,
                                                 255,
                                                 0x80000000,
                                                 0x007fffff,
                                                 0xff800000,
                                                 0x0012ffff,
                                                 0x12ff,
                                                 0xffedffff,
                                                 0xffffedff,
                                                 0x1234ffff,
                                                 0x12345678,
                                                 0xabcdef01,
                                                 0xffffffff,
                                                 0xffff0000};
  dsp56300::detail::ExecutableArena arena;
  for (const auto value : values) {
    Code code;
    Neon neon(code);
    constexpr std::array unsupported{0x1234ffffU, 0x12345678U, 0xabcdef01U};
    const bool supported = std::ranges::find(unsupported, value) == unsupported.end();
    require(neon.constant_words(Vec{0}, value) == supported,
            "NEON constant selection must accept exactly the directed architectural immediates");
    if (!supported) {
      require(code.size() == 0, "Unencodable NEON immediates must leave code untouched");
      continue;
    }
    neon.store(Vec{0}, Reg::x0);
    code.return_();
    std::array<std::uint32_t, 4> result{};
    static_cast<void>(dsp56300_a64_test_enter(
        arena.append(code.bytes()), reinterpret_cast<std::uintptr_t>(result.data()), 0));
    require(std::ranges::all_of(result, [value](const auto lane) { return lane == value; }),
            "NEON immediates preserve every bit of each word lane");
  }
}

void execute_packed() {
  struct alignas(16) Data {
    std::array<std::int32_t, 4> left, right;
    std::array<std::int64_t, 4> product{};
    std::array<std::int32_t, 4> sum{}, difference{}, low_product{}, narrowed{}, inserted{},
        broadcast{}, rounded{};
  };

  constexpr Data initial{
      .left = {-2147483647 - 1, -1, 2147483647, 0x12345678},
      .right = {-2147483647 - 1, 2147483647, -2, std::bit_cast<std::int32_t>(0x87654321U)}};
  auto data = initial;
  constexpr Vec narrowed{17};
  constexpr Vec inserted{18};
  constexpr Vec broadcast{19};
  constexpr Vec rounded{20};
  constexpr unsigned rounding_shift = 23;
  Code code;
  Neon neon(code);
  neon.load(Vec{0}, Reg::x0, offsetof(Data, left));
  neon.load(Vec{1}, Reg::x0, offsetof(Data, right));
  neon.binary(VectorOp::multiply, Vec{2}, Vec{0}, Vec{1});
  neon.binary(VectorOp::multiply_upper, Vec{3}, Vec{0}, Vec{1});
  neon.store(Vec{2}, Reg::x0, offsetof(Data, product));
  neon.store(Vec{3}, Reg::x0, offsetof(Data, product) + 16);
  neon.binary(VectorOp::add_words, Vec{4}, Vec{0}, Vec{1});
  neon.store(Vec{4}, Reg::x0, offsetof(Data, sum));
  neon.binary(VectorOp::subtract_words, Vec{5}, Vec{0}, Vec{1});
  neon.store(Vec{5}, Reg::x0, offsetof(Data, difference));
  neon.binary(VectorOp::multiply_words, Vec{6}, Vec{0}, Vec{1});
  neon.store(Vec{6}, Reg::x0, offsetof(Data, low_product));
  neon.extend(Vec{7}, Vec{0});
  neon.extend(Vec{16}, Vec{0}, true);
  neon.narrow(narrowed, Vec{7});
  neon.narrow(narrowed, Vec{16}, true);
  neon.store(narrowed, Reg::x0, offsetof(Data, narrowed));
  for (unsigned lane = 0; lane < 4; ++lane) {
    code.constant(Reg::x9, static_cast<std::uint32_t>(data.right[lane]));
    neon.insert(inserted, Reg::x9, lane, true);
    neon.extract(Reg::x9, inserted, lane, true);
    code.store(Reg::x9, Reg::x0, offsetof(Data, inserted) + (lane * sizeof(std::int32_t)));
  }
  neon.broadcast(broadcast, Reg::x1, true);
  neon.store(broadcast, Reg::x0, offsetof(Data, broadcast));
  neon.round_narrow(rounded, Vec{2}, rounding_shift);
  neon.round_narrow(rounded, Vec{3}, rounding_shift, true);
  neon.store(rounded, Reg::x0, offsetof(Data, rounded));
  code.return_();
  dsp56300::detail::ExecutableArena arena;
  constexpr std::uint32_t repeated = 0xfedcba98;
  static_cast<void>(dsp56300_a64_test_enter(
      arena.append(code.bytes()), reinterpret_cast<std::uintptr_t>(&data), repeated));
  for (unsigned lane = 0; lane < 4; ++lane) {
    const auto left = std::int64_t{data.left[lane]};
    const auto right = std::int64_t{data.right[lane]};
    require(
        data.product[lane] == left * right &&
            static_cast<std::uint32_t>(data.low_product[lane]) ==
                static_cast<std::uint32_t>(left * right) &&
            static_cast<std::uint32_t>(data.sum[lane]) ==
                static_cast<std::uint32_t>(left + right) &&
            static_cast<std::uint32_t>(data.difference[lane]) ==
                static_cast<std::uint32_t>(left - right) &&
            data.narrowed[lane] == data.left[lane] && data.inserted[lane] == data.right[lane] &&
            static_cast<std::uint32_t>(data.broadcast[lane]) == repeated &&
            static_cast<std::uint32_t>(data.rounded[lane]) ==
                static_cast<std::uint32_t>(
                    (left * right + (std::int64_t{1} << (rounding_shift - 1))) >> rounding_shift),
        "Packed NEON preserves signed products, word arithmetic, all lanes and conversions");
  }
}

void execute_neon_memory() {
  dsp56300::detail::ExecutableArena arena;
  constexpr std::size_t large_offset = 65536;
  constexpr std::uint64_t guard = 0xabcdef0123456789;
  constexpr std::uint64_t value = 0x8123456789abcdef;
  std::vector<std::uint64_t> memory((large_offset / sizeof(value)) + 4, guard);
  for (const auto offset : {std::size_t{16}, large_offset}) {
    for (const bool words : {false, true}) {
      std::ranges::fill(memory, guard);
      Code code;
      Neon neon(code);
      neon.broadcast(Vec{0}, Reg::x1);
      neon.store(Vec{0}, Reg::x0, offset, words);
      neon.load(Vec{1}, Reg::x0, offset, words);
      neon.extract(Reg::x0, Vec{1}, words ? 0U : 1U);
      code.return_();
      const auto *entry = arena.append(code.bytes());
      require(dsp56300_a64_test_enter(
                  entry, reinterpret_cast<std::uintptr_t>(memory.data()), value) == value,
              "NEON memory preserves exact small and expanded displacements");
      const auto first = offset / sizeof(value);
      const auto last = first + (words ? 1U : 2U);
      for (std::size_t index = 0; index < memory.size(); ++index) {
        require(memory[index] == (index >= first && index < last ? value : guard),
                "NEON stores preserve every word outside the selected lane extent");
      }
    }
  }
}

void execute_scratch() {
  dsp56300::detail::ExecutableArena arena;
  constexpr std::array<std::uint16_t, 3> extents{16, 128, 4080};
  for (const auto bytes : extents) {
    Code code;
    code.write_flags(Reg::x1);
    code.add(Reg::x2, Reg::sp, 0, Width::x64);
    code.subtract(Reg::sp, Reg::sp, bytes, Width::x64);
    code.add(Reg::x3, Reg::sp, 0, Width::x64);
    code.alu(Alu::subtract, Reg::x2, Reg::x2, Reg::x3, Width::x64);
    code.store(Reg::x2, Reg::x0, 0, Access::doubleword);
    code.store(Reg::x0, Reg::sp, bytes - sizeof(std::uint64_t), Access::doubleword);
    code.load(Reg::x2, Reg::sp, bytes - sizeof(std::uint64_t), Access::doubleword);
    code.add(Reg::sp, Reg::sp, bytes, Width::x64);
    code.store(Reg::x2, Reg::x0, sizeof(std::uint64_t), Access::doubleword);
    code.read_flags(Reg::x2);
    code.store(Reg::x2, Reg::x0, 2 * sizeof(std::uint64_t), Access::doubleword);
    code.return_();
    const auto *entry = arena.append(code.bytes());
    for (unsigned status = 0; status < 16; ++status) {
      const auto flags = std::uint64_t{status} << 28;
      std::array<std::uint64_t, 3> output{};
      const auto pointer = reinterpret_cast<std::uintptr_t>(output.data());
      static_cast<void>(dsp56300_a64_test_enter(entry, pointer, flags));
      require(output == std::array<std::uint64_t, 3>{bytes, pointer, flags},
              "Scratch reservation must preserve exact SP extent, stack storage and every NZCV");
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
    logical_encodings();
    fixups();
    near_fixups();
    branch_ranges();
    barrel_encodings();
    bit_encodings();
    neon_encodings();
    packed_encodings();
    immediate_encodings();
    accumulate_encodings();
    stack_encodings();
    subtract_encodings();
    selection_encodings();
#if defined(__aarch64__)
    execute();
    execute_constants();
    execute_barrels();
    execute_bits();
    execute_neon_memory();
    execute_packed();
    execute_immediates();
    execute_stack();
    execute_scratch();
#endif
    std::cout << "A64 encoding and branch checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
