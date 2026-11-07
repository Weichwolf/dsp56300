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
enum class Xmm : std::uint8_t {
  x0,
  x1,
  x2,
  x3,
  x4,
  x5,
  x6,
  x7,
  x8,
  x9,
  x10,
  x11,
  x12,
  x13,
  x14,
  x15
};
enum class Packed : std::uint8_t {
  copy = 0x6f,
  add_qword = 0xd4,
  equal_dword = 0x76,
  bit_or = 0xeb,
  bit_xor = 0xef,
  unpack_low_qword = 0x6c
};

template <unsigned LaneBits>
concept ScalarTransferLane = SimdShape<LaneBits, 128> && (LaneBits == 32 || LaneBits == 64);
enum class Branch : std::uint8_t {
  below = 0x82,
  above_equal = 0x83,
  equal = 0x84,
  not_equal = 0x85,
  below_equal = 0x86,
  above = 0x87
};

struct RegisterPair {
  Reg destination, source;
};

struct IndexedAddress {
  Reg base, index;
  std::size_t offset = 0;
};

// Low byte is the opcode; the next byte is the VEX opcode-map selector.
enum class VectorOp : std::uint16_t {
  copy = 0x016f,
  add = 0x01d4,
  subtract = 0x01fb,
  multiply = 0x0228,
  bit_and = 0x01db,
  bit_or = 0x01eb,
  bit_xor = 0x01ef,
  greater_qword = 0x0237,
  minimum_dword = 0x0239,
  maximum_dword = 0x023d,
  blend_dword = 0x0302,
  shuffle_dword = 0x0170,
  shuffle_bytes = 0x0200,
  shift_right_qword = 0x01d3,
  extend_dword = 0x0225
};

struct VectorOperands {
  Xmm destination, left, right;
};

struct VectorAddress {
  Reg base;
  std::size_t offset = 0;
};

struct VectorEncoding {
  unsigned map = 1, prefix = 1;
  bool wide = true, qword = false;
};

struct VectorFields {
  unsigned destination = 0, left = 0, base = 0, index = 0;
};

enum class VectorShift : std::uint8_t { right = 2, arithmetic = 4, left = 6 };

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
  void align_entry();
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
  void vector(VectorOp operation, VectorOperands registers, bool wide = true);
  void vector_unary(VectorOp operation, Xmm destination, Xmm source, bool wide = true);
  void vector_immediate(VectorOp operation, VectorOperands registers, unsigned immediate);
  void vector_shift(
      Xmm destination, Xmm source, VectorShift operation, unsigned bits, bool dwords = false);
  void vector_load(Xmm destination, VectorAddress address, bool wide = true);
  void vector_store(VectorAddress address, Xmm source, bool wide = true);
  void vector_broadcast(Xmm destination, VectorAddress address, bool dwords = false);
  void vector_pack(Xmm destination, Xmm low, Xmm high);
  void vector_extract(Xmm destination, Xmm source, unsigned lane);
  void vector_scalar(Reg destination, Xmm source, unsigned lane);
  void vector_load_dword(Xmm destination, VectorAddress address);
  void vector_insert_dword(Xmm destination, IndexedAddress address, unsigned lane);
  void vector_finish();

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
  void vex(VectorEncoding encoding, VectorFields fields);
  std::vector<std::uint8_t> bytes_;
};
} // namespace dsp56300::detail::x64
