// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail::a64 {
enum class Reg : std::uint8_t {
  x0 = 0,
  x1 = 1,
  x2 = 2,
  x3 = 3,
  x4 = 4,
  x5 = 5,
  x6 = 6,
  x7 = 7,
  x8 = 8,
  x9 = 9,
  x10 = 10,
  x11 = 11,
  x12 = 12,
  x13 = 13,
  x14 = 14,
  x15 = 15,
  x16 = 16,
  x17 = 17,
  x19 = 19,
  x20 = 20,
  x21 = 21,
  x22 = 22,
  x23 = 23,
  x24 = 24,
  x25 = 25,
  x26 = 26,
  x27 = 27,
  x28 = 28,
  fp = 29,
  lr = 30,
  zr = 31,
  sp = 31
};
enum class Width : std::uint8_t { w32, x64 };
enum class Access : std::uint8_t { byte, half, word, doubleword };
enum class Condition : std::uint8_t {
  equal,
  not_equal,
  carry,
  no_carry,
  negative,
  positive,
  overflow,
  no_overflow,
  above,
  below_equal,
  greater_equal,
  less,
  greater,
  less_equal,
  always
};
enum class Alu : std::uint32_t {
  add = 0x0b000000,
  add_flags = 0x2b000000,
  subtract = 0x4b000000,
  subtract_flags = 0x6b000000,
  add_carry = 0x1a000000,
  subtract_carry = 0x5a000000,
  bit_and = 0x0a000000,
  bit_or = 0x2a000000,
  bit_xor = 0x4a000000,
  bit_clear = 0x0a200000,
  test = 0x6a000000
};
enum class Shift : std::uint8_t { left, right, arithmetic, rotate };
enum class BranchRange : std::uint8_t { full, near };

struct Reference {
  std::size_t offset;
  unsigned bits;
};

struct Label {
  std::vector<Reference> references;
  std::size_t offset = 0;
  bool bound = false;
};

class Code {
public:
  static constexpr std::size_t near_branch_bytes = std::size_t{1} << 20;

  explicit Code(BranchRange range = BranchRange::full) : range_(range) {}

  [[nodiscard]] const std::vector<std::uint8_t> &bytes() const noexcept { return bytes_; }

  [[nodiscard]] std::size_t size() const noexcept { return bytes_.size(); }

  void append(std::span<const std::uint8_t> bytes);
  void constant(Reg target, std::uint64_t value, Width width = Width::x64);
  void move(Reg target, Reg source, Width width = Width::x64);
  void alu(Alu operation, Reg target, Reg left, Reg right, Width width = Width::w32);
  [[nodiscard]] bool
  logical(Alu operation, Reg target, Reg source, std::uint64_t value, Width width = Width::w32);
  void add(Reg target, Reg source, std::uint16_t value, Width width = Width::w32);
  void subtract(Reg target, Reg source, std::uint16_t value, Width width = Width::w32);
  void extract(Reg target,
               Reg source,
               unsigned first,
               unsigned count,
               Width width = Width::w32,
               bool sign = false);
  void shift(Shift operation, Reg target, Reg source, unsigned count, Width width = Width::w32);
  void shift(Shift operation, Reg target, Reg source, Reg count, Width width = Width::w32);
  void reverse_half(Reg target, Reg source);
  void reverse_bits(Reg target, Reg source, Width width = Width::w32);
  void count_leading_zeroes(Reg target, Reg source, Width width = Width::w32);
  void load(Reg target, Reg base, std::size_t offset, Access access = Access::word);
  void store(Reg source, Reg base, std::size_t offset, Access access = Access::word);
  void multiply(Reg target, Reg left, Reg right, Width width = Width::w32);
  void divide(Reg target, Reg left, Reg right, bool sign, Width width = Width::w32);
  void multiply_subtract(Reg target, Reg left, Reg right, Reg minuend, Width width = Width::w32);
  void compare(Reg left, Reg right, Width width = Width::w32);
  void select(Reg target, Reg left, Reg right, Condition condition, Width width = Width::w32);
  void set(Reg target, Condition condition);
  void branch(Label &label);
  void branch(Condition condition, Label &label);
  void branch_zero(Reg value, Label &label, Width width = Width::w32, bool nonzero = false);
  void branch_far(Condition condition, Label &label);
  void branch_zero_far(Reg value, Label &label, Width width = Width::w32, bool nonzero = false);
  void bind(Label &label);
  void call(Reg target);
  void jump(Reg target);
  void return_();
  void push(Reg left, Reg right);
  void pop(Reg left, Reg right);
  void read_flags(Reg target);
  void write_flags(Reg source);

private:
  friend class Neon;
  void word(std::uint32_t value);
  void reference(Label &label, unsigned bits);
  void patch(Reference reference, std::size_t target);
  void memory(Reg value, Reg base, std::size_t offset, Access access, bool read);
  BranchRange range_;
  std::vector<std::uint8_t> bytes_;
};
}
