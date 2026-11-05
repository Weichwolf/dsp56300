// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "simd.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <vector>

namespace dsp56300::detail::x64 {
enum class Reg : std::uint8_t {
  ax,
  cx,
  dx,
  bx,
  sp,
  bp,
  si,
  di,
  r8,
  r9,
  r10,
  r11,
  r12,
  r13,
  r14,
  r15
};
enum class Width : std::uint8_t { byte = 1, word = 2, dword = 4, qword = 8 };
enum class Xmm : std::uint8_t { x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11 };
enum class Packed : std::uint8_t {
  copy = 0x6f,
  add_qword = 0xd4,
  equal_dword = 0x76,
  bit_xor = 0xef,
  unpack_low_qword = 0x6c
};

template <unsigned LaneBits>
concept ScalarTransferLane = SimdShape<LaneBits, 128> && (LaneBits == 32 || LaneBits == 64);
enum class Branch : std::uint8_t {
  above_equal = 0x83,
  equal = 0x84,
  not_equal = 0x85,
  below_equal = 0x86
};

struct RegisterPair {
  Reg destination, source;
};

struct IndexedAddress {
  Reg base, index;
  std::size_t offset = 0;
};

struct Label {
  std::vector<std::size_t> references;
  std::size_t offset = 0;
  bool bound = false;
};

class Code {
public:
  void byte(std::uint32_t value);
  void u32(std::uint32_t value);
  void u64(std::uint64_t value);
  void raw(std::initializer_list<std::uint8_t> bytes);
  void raw(std::span<const std::uint8_t> bytes);
  void mov(RegisterPair registers, Width width = Width::qword);
  void constant(Reg destination, std::uint64_t value);
  void load(Reg destination, Reg base, std::size_t offset, Width width = Width::qword);
  void store(Reg base, std::size_t offset, Reg source, Width width = Width::qword);
  void load_indexed(Reg destination, IndexedAddress address, Width width = Width::dword);
  void store_indexed(IndexedAddress address, Reg source, Width width = Width::dword);
  void store_constant(Reg base, std::size_t offset, Width width, std::uint32_t value);
  void
  memory_immediate(unsigned group, Reg base, std::size_t offset, Width width, std::uint32_t value);
  void binary(std::uint32_t opcode, RegisterPair registers, Width width = Width::qword);
  void immediate(unsigned group, Reg destination, std::uint32_t value, Width width = Width::qword);
  void shift(unsigned group, Reg destination, unsigned count, Width width = Width::qword);
  void multiply(RegisterPair registers);
  void divide(Reg divisor);
  void unary(unsigned group, Reg destination);
  void set(unsigned condition, Reg destination);
  void packed(Packed operation, Xmm destination, Xmm source);

  template <unsigned LaneBits = 32>
    requires ScalarTransferLane<LaneBits>
  void packed_from(const Xmm destination, const Reg source) {
    scalar_from(destination, source, LaneBits == 64 ? Width::qword : Width::dword);
  }

  template <unsigned LaneBits = 64>
    requires ScalarTransferLane<LaneBits>
  void packed_to(const Reg destination, const Xmm source) {
    scalar_to(destination, source, LaneBits == 64 ? Width::qword : Width::dword);
  }

  template <unsigned LaneBits = 64>
    requires ScalarTransferLane<LaneBits>
  void packed_load_lane(const Xmm destination, const Reg base, const std::size_t offset) {
    scalar_load(destination, base, offset, LaneBits == 64 ? Width::qword : Width::dword);
  }

  void packed_shift_right(Xmm destination, unsigned count);
  void packed_load(Xmm destination, Reg base, std::size_t offset);
  void packed_store(Reg base, std::size_t offset, Xmm source);
  void move_if(unsigned condition, RegisterPair registers);
  void jump_if(Branch condition, Label &label);
  void jump(Label &label);
  void call(Label &label);
  void bind(Label &label);

  [[nodiscard]] const std::vector<std::uint8_t> &bytes() const noexcept { return bytes_; }

private:
  void scalar_from(Xmm destination, Reg source, Width width);
  void scalar_to(Reg destination, Xmm source, Width width);
  void scalar_load(Xmm destination, Reg base, std::size_t offset, Width width);
  void rex(Width width, unsigned reg, unsigned base, unsigned index = 0);
  void memory(unsigned reg, Reg base, std::size_t offset);
  void indexed(unsigned reg, IndexedAddress address);
  void reference(Label &label);
  void patch(std::size_t offset, std::size_t target);
  std::vector<std::uint8_t> bytes_;
};
} // namespace dsp56300::detail::x64
