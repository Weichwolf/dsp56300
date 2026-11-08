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
  bit_and = 0x4e201c00,
  bit_or = 0x4ea01c00,
  bit_xor = 0x6e201c00,
  greater = 0x4ee03400,
  maximum = 0x4ea06400,
  minimum = 0x4ea06c00,
  select = 0x6e601c00,
  multiply = 0x0ea0c000
};

class Neon {
public:
  explicit Neon(Code &code) : code_(code) {}

  void binary(VectorOp operation, Vec destination, Vec left, Vec right);
  void move(Vec destination, Vec operand);
  void zero(Vec destination);
  void shift(Vec destination, Vec operand, Shift operation, unsigned count, bool words = false);
  void narrow(Vec destination, Vec operand);
  void extend(Vec destination, Vec operand);
  void broadcast(Vec destination, Reg operand);
  void insert(Vec destination, Reg operand, unsigned lane);
  void extract(Reg destination, Vec operand, unsigned lane);
  void load(Vec destination, Reg base, std::size_t offset = 0, bool words = false);
  void store(Vec operand, Reg base, std::size_t offset = 0, bool words = false);

private:
  enum class MemoryOp : std::uint8_t { load, store };
  void memory(Vec value, Reg base, std::size_t offset, bool words, MemoryOp operation);
  Code &code_;
};
}
