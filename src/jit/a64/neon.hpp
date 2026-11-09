// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/a64/code.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
enum class Vec : std::uint8_t {};

constexpr Vec operator+(const Vec reg, const unsigned offset) {
  return static_cast<Vec>(static_cast<unsigned>(reg) + offset);
}

enum class VectorOp : std::uint32_t {
  add = 0x4ee08400,
  subtract = 0x6ee08400,
  add_words = 0x4ea08400,
  subtract_words = 0x6ea08400,
  multiply_words = 0x4ea09c00,
  bit_and = 0x4e201c00,
  bit_or = 0x4ea01c00,
  bit_xor = 0x6e201c00,
  bit_insert = 0x6ea01c00,
  unzip_even = 0x4e801800,
  unzip_odd = 0x4e805800,
  zip_low = 0x4e803800,
  zip_high = 0x4e807800,
  greater = 0x4ee03400,
  maximum = 0x4ea06400,
  minimum = 0x4ea06c00,
  select = 0x6e601c00,
  multiply = 0x0ea0c000,
  multiply_upper = 0x4ea0c000,
  multiply_add = 0x0ea08000,
  multiply_add_upper = 0x4ea08000,
  multiply_subtract = 0x0ea0a000,
  multiply_subtract_upper = 0x4ea0a000
};

class Neon {
public:
  explicit Neon(Code &code) : code_(code) {}

  void binary(VectorOp operation, Vec destination, Vec left, Vec right);
  void move(Vec destination, Vec operand);
  void zero(Vec destination);
  [[nodiscard]] bool constant_words(Vec destination, std::uint32_t value);
  void shift(Vec destination, Vec operand, Shift operation, unsigned count, bool words = false);
  void narrow(Vec destination, Vec operand, bool upper = false);
  void extend(Vec destination, Vec operand, bool upper = false);
  void round_narrow(Vec destination, Vec operand, unsigned count, bool upper = false);
  void broadcast(Vec destination, Reg operand, bool words = false);
  void insert(Vec destination, Reg operand, unsigned lane, bool words = false);
  void extract(Reg destination, Vec operand, unsigned lane, bool words = false);
  void load(Vec destination, Reg base, std::size_t offset = 0, bool words = false);
  void store(Vec operand, Reg base, std::size_t offset = 0, bool words = false);
  void load_interleaved(Vec first, Reg base);
  void store_interleaved(Vec first, Reg base);
  void push_doubles(Vec left, Vec right);
  void pop_doubles(Vec left, Vec right);

private:
  enum class MemoryOp : std::uint8_t { load, store };
  void memory(Vec value, Reg base, std::size_t offset, bool words, MemoryOp operation);
  Code &code_;
};
}
