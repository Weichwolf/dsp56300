// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/code.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>

namespace dsp56300::detail::a64 {
namespace {
constexpr unsigned register_shift = 5, operand_shift = 16, immediate_shift = 10;
constexpr unsigned half_bits = 16, byte_bits = 8, byte_mask = 0xff;
constexpr unsigned instruction_bytes = 4, condition_shift = 12;
constexpr std::uint32_t wide_bit = 0x80000000;

constexpr auto reg(const Reg value) {
  return static_cast<std::uint32_t>(value);
}

constexpr auto wide(const Width width) {
  return width == Width::x64 ? wide_bit : 0U;
}

constexpr unsigned bit_count(const Width width) {
  return width == Width::x64 ? 64U : 32U;
}
}

void Code::word(const std::uint32_t value) {
  for (unsigned shift = 0; shift < 32; shift += byte_bits) {
    bytes_.push_back(static_cast<std::uint8_t>((value >> shift) & byte_mask));
  }
}

void Code::append(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() % instruction_bytes != 0) {
    throw std::invalid_argument("A64 code must contain complete instructions");
  }
  bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
}

void Code::constant(const Reg target, const std::uint64_t value, const Width width) {
  if (width == Width::w32 && value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("A64 constant exceeds its register width");
  }
  constexpr std::uint32_t movz = 0x52800000;
  constexpr std::uint32_t movk = 0x72800000;
  constexpr unsigned lane_shift = 21;
  constexpr unsigned half_mask = 0xffff;
  for (unsigned lane = 0; lane < bit_count(width) / half_bits; ++lane) {
    const auto part = static_cast<std::uint32_t>((value >> (lane * half_bits)) & half_mask);
    if (lane != 0 && part == 0) { continue; }
    word(wide(width) | (lane == 0 ? movz : movk) | (lane << lane_shift) | (part << register_shift) |
         reg(target));
  }
}

void Code::move(const Reg target, const Reg source, const Width width) {
  alu(Alu::bit_or, target, Reg::zr, source, width);
}

void Code::alu(
    const Alu operation, const Reg target, const Reg left, const Reg right, const Width width) {
  word(wide(width) | static_cast<std::uint32_t>(operation) | (reg(right) << operand_shift) |
       (reg(left) << register_shift) | reg(target));
}

void Code::add(const Reg target, const Reg source, const std::uint16_t value, const Width width) {
  constexpr std::uint32_t opcode = 0x11000000;
  constexpr unsigned maximum = 0xfff;
  if (value > maximum) { throw std::invalid_argument("A64 add immediate exceeds 12 bits"); }
  word(wide(width) | opcode | (std::uint32_t{value} << immediate_shift) |
       (reg(source) << register_shift) | reg(target));
}

void Code::extract(const Reg target,
                   const Reg source,
                   const unsigned first,
                   const unsigned count,
                   const Width width,
                   const bool sign) {
  const auto bits = bit_count(width);
  if (count == 0 || first >= bits || count > bits - first) {
    throw std::invalid_argument("Invalid A64 bit-field extent");
  }
  constexpr std::uint32_t ubfm = 0x53000000;
  constexpr std::uint32_t sbfm = 0x13000000;
  constexpr std::uint32_t wide_n = 0x00400000;
  word(wide(width) | (sign ? sbfm : ubfm) | (width == Width::x64 ? wide_n : 0U) |
       (first << operand_shift) | ((first + count - 1) << immediate_shift) |
       (reg(source) << register_shift) | reg(target));
}

void Code::shift(const Shift operation,
                 const Reg target,
                 const Reg source,
                 const unsigned count,
                 const Width width) {
  const auto bits = bit_count(width);
  if (count >= bits) { throw std::invalid_argument("A64 shift exceeds register width"); }
  if (operation == Shift::right || operation == Shift::arithmetic) {
    extract(target, source, count, bits - count, width, operation == Shift::arithmetic);
    return;
  }
  constexpr std::uint32_t ubfm = 0x53000000;
  constexpr std::uint32_t extr = 0x13800000;
  constexpr std::uint32_t wide_n = 0x00400000;
  const auto encoding = wide(width) | (width == Width::x64 ? wide_n : 0U);
  if (operation == Shift::left) {
    word(encoding | ubfm | (((bits - count) % bits) << operand_shift) |
         ((bits - count - 1) << immediate_shift) | (reg(source) << register_shift) | reg(target));
  } else {
    word(encoding | extr | (reg(source) << operand_shift) | (count << immediate_shift) |
         (reg(source) << register_shift) | reg(target));
  }
}

void Code::shift(
    const Shift operation, const Reg target, const Reg source, const Reg count, const Width width) {
  constexpr std::uint32_t opcode = 0x1ac02000;
  word(wide(width) | opcode | (static_cast<unsigned>(operation) << immediate_shift) |
       (reg(count) << operand_shift) | (reg(source) << register_shift) | reg(target));
}

void Code::reverse_half(const Reg target, const Reg source) {
  constexpr std::uint32_t opcode = 0x5ac00400;
  word(opcode | (reg(source) << register_shift) | reg(target));
}

void Code::memory(const Reg value,
                  const Reg base,
                  const std::size_t offset,
                  const Access access,
                  const bool read) {
  constexpr std::uint32_t store = 0x39000000;
  constexpr std::uint32_t load = 0x00400000;
  constexpr unsigned size_shift = 30;
  constexpr unsigned maximum_offset = 0xfff;
  const auto size = static_cast<unsigned>(access);
  const auto alignment = std::size_t{1} << size;
  if (offset % alignment != 0 || offset / alignment > maximum_offset) {
    throw std::invalid_argument("Unaligned or oversized A64 load/store offset");
  }
  word(store | (read ? load : 0U) | (size << size_shift) |
       (static_cast<std::uint32_t>(offset / alignment) << immediate_shift) |
       (reg(base) << register_shift) | reg(value));
}

void Code::load(const Reg target, const Reg base, const std::size_t offset, const Access access) {
  memory(target, base, offset, access, true);
}

void Code::store(const Reg source, const Reg base, const std::size_t offset, const Access access) {
  memory(source, base, offset, access, false);
}

void Code::multiply(const Reg target, const Reg left, const Reg right, const Width width) {
  constexpr std::uint32_t opcode = 0x1b007c00;
  word(wide(width) | opcode | (reg(right) << operand_shift) | (reg(left) << register_shift) |
       reg(target));
}

void Code::divide(
    const Reg target, const Reg left, const Reg right, const bool sign, const Width width) {
  constexpr std::uint32_t unsigned_divide = 0x1ac00800;
  constexpr std::uint32_t signed_divide = 0x1ac00c00;
  word(wide(width) | (sign ? signed_divide : unsigned_divide) | (reg(right) << operand_shift) |
       (reg(left) << register_shift) | reg(target));
}

void Code::multiply_subtract(
    const Reg target, const Reg left, const Reg right, const Reg minuend, const Width width) {
  constexpr std::uint32_t opcode = 0x1b008000;
  word(wide(width) | opcode | (reg(right) << operand_shift) | (reg(minuend) << immediate_shift) |
       (reg(left) << register_shift) | reg(target));
}

void Code::compare(const Reg left, const Reg right, const Width width) {
  alu(Alu::subtract_flags, Reg::zr, left, right, width);
}

void Code::select(const Reg target,
                  const Reg left,
                  const Reg right,
                  const Condition condition,
                  const Width width) {
  constexpr std::uint32_t opcode = 0x1a800000;
  word(wide(width) | opcode | (reg(right) << operand_shift) |
       (static_cast<unsigned>(condition) << condition_shift) | (reg(left) << register_shift) |
       reg(target));
}

void Code::set(const Reg target, const Condition condition) {
  constexpr std::uint32_t opcode = 0x1a9f07e0;
  word(opcode | ((static_cast<unsigned>(condition) ^ 1U) << condition_shift) | reg(target));
}

void Code::patch(const Reference reference, const std::size_t target) {
  const auto delta =
      static_cast<std::int64_t>(target) - static_cast<std::int64_t>(reference.offset);
  const auto limit = std::int64_t{1} << (reference.bits - 1);
  if (delta % instruction_bytes != 0 || delta / instruction_bytes < -limit ||
      delta / instruction_bytes >= limit) {
    throw std::length_error("A64 branch target exceeds immediate range");
  }
  const auto mask = (std::uint32_t{1} << reference.bits) - 1;
  const auto field = (std::bit_cast<std::uint64_t>(delta / instruction_bytes) & mask)
                     << (reference.bits == 26 ? 0U : register_shift);
  for (unsigned index = 0; index < instruction_bytes; ++index) {
    bytes_[reference.offset + index] |=
        static_cast<std::uint8_t>((field >> (index * byte_bits)) & byte_mask);
  }
}

void Code::reference(Label &label, const unsigned bits) {
  const Reference pending{.offset = size() - instruction_bytes, .bits = bits};
  if (label.bound) {
    patch(pending, label.offset);
  } else {
    label.references.push_back(pending);
  }
}

void Code::branch(Label &label) {
  constexpr std::uint32_t opcode = 0x14000000;
  constexpr unsigned bits = 26;
  word(opcode);
  reference(label, bits);
}

void Code::branch(const Condition condition, Label &label) {
  constexpr std::uint32_t opcode = 0x54000000;
  constexpr unsigned bits = 19;
  word(opcode | static_cast<unsigned>(condition));
  reference(label, bits);
}

void Code::branch_zero(const Reg value, Label &label, const Width width, const bool nonzero) {
  constexpr std::uint32_t opcode = 0x34000000;
  constexpr std::uint32_t nonzero_bit = 0x01000000;
  constexpr unsigned bits = 19;
  word(wide(width) | opcode | (nonzero ? nonzero_bit : 0U) | reg(value));
  reference(label, bits);
}

void Code::bind(Label &label) {
  if (label.bound) { throw std::logic_error("A64 label already bound"); }
  label.offset = size();
  label.bound = true;
  for (const auto pending : label.references) { patch(pending, label.offset); }
  label.references.clear();
}

void Code::branch_far(const Condition condition, Label &label) {
  if (condition == Condition::always) {
    branch(label);
    return;
  }
  Label skip;
  branch(static_cast<Condition>(static_cast<unsigned>(condition) ^ 1U), skip);
  branch(label);
  bind(skip);
}

void Code::branch_zero_far(const Reg value, Label &label, const Width width, const bool nonzero) {
  Label skip;
  branch_zero(value, skip, width, !nonzero);
  branch(label);
  bind(skip);
}

void Code::call(const Reg target) {
  constexpr std::uint32_t opcode = 0xd63f0000;
  word(opcode | (reg(target) << register_shift));
}

void Code::jump(const Reg target) {
  constexpr std::uint32_t opcode = 0xd61f0000;
  word(opcode | (reg(target) << register_shift));
}

void Code::return_() {
  constexpr std::uint32_t opcode = 0xd65f03c0;
  word(opcode);
}

void Code::push(const Reg left, const Reg right) {
  constexpr std::uint32_t opcode = 0xa9bf03e0;
  word(opcode | (reg(right) << immediate_shift) | reg(left));
}

void Code::pop(const Reg left, const Reg right) {
  constexpr std::uint32_t opcode = 0xa8c103e0;
  word(opcode | (reg(right) << immediate_shift) | reg(left));
}

void Code::read_flags(const Reg target) {
  constexpr std::uint32_t opcode = 0xd53b4200;
  word(opcode | reg(target));
}

void Code::write_flags(const Reg source) {
  constexpr std::uint32_t opcode = 0xd51b4200;
  word(opcode | reg(source));
}
}
