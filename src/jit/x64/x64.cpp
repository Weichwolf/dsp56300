// SPDX-License-Identifier: GPL-3.0-only
#include "x64.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>

namespace dsp56300::detail::x64 {
namespace {
enum EncodingByte : std::uint8_t {
  low_byte_mask = 0xff,
  operand_prefix = 0x66,
  stack_index = 0x24,
  store_byte = 0xc6,
  store_other = 0xc7,
  multiply_opcode = 0xaf,
  unary_opcode = 0xf7,
  extend_byte = 0xb6
};

constexpr unsigned byte_bits = 8, rex_prefix = 0x40, wide_bit = 8, register_bit = 4;
constexpr unsigned index_bit = 2, word_scale = 2;
constexpr unsigned mov_register = 0x89, mov_load = 0x8b, mov_immediate = 0xb8;
constexpr unsigned mod_register = 0xc0, mod_small = 0x40, mod_large = 0x80;
constexpr unsigned alu_immediate = 0x81, shift_immediate = 0xc1;

std::uint32_t immediate_opcode(const Width width, const std::uint32_t value) {
  constexpr std::uint32_t immediate_byte = 0x80;
  constexpr std::uint32_t immediate_short = 0x83;
  constexpr std::uint32_t maximum_short = 127;
  constexpr std::uint32_t word_mask = 0xffff;
  if (width == Width::byte) { return immediate_byte; }
  const auto mask = width == Width::word ? word_mask : std::numeric_limits<std::uint32_t>::max();
  const auto truncated = value & mask;
  return truncated <= maximum_short || truncated >= mask - maximum_short ? immediate_short
                                                                         : alu_immediate;
}

void immediate_value(Code &code, const Width width, const std::uint32_t value, const bool compact) {
  code.byte(value & low_byte_mask);
  if (compact) { return; }
  code.byte((value >> byte_bits) & low_byte_mask);
  if (width == Width::word) { return; }
  code.byte((value >> (2 * byte_bits)) & low_byte_mask);
  code.byte((value >> (3 * byte_bits)) & low_byte_mask);
}
}

void Code::byte(const std::uint32_t value) {
  if (value > std::numeric_limits<std::uint8_t>::max()) {
    throw std::logic_error("Invalid x64 byte");
  }
  bytes_.push_back(static_cast<std::uint8_t>(value));
}

void Code::u32(const std::uint32_t value) {
  for (unsigned shift = 0; shift < sizeof(value) * byte_bits; shift += byte_bits) {
    byte((value >> shift) & low_byte_mask);
  }
}

void Code::u64(const std::uint64_t value) {
  for (unsigned shift = 0; shift < sizeof(value) * byte_bits; shift += byte_bits) {
    byte(static_cast<std::uint32_t>((value >> shift) & low_byte_mask));
  }
}

void Code::raw(const std::span<const std::uint8_t> bytes) {
  bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
}

void Code::raw(const std::initializer_list<std::uint8_t> bytes) {
  bytes_.insert(bytes_.end(), bytes);
}

void Code::rex(const Width width, const unsigned reg, const unsigned base, const unsigned index) {
  if (width == Width::word) { byte(operand_prefix); }
  const auto prefix = rex_prefix | (width == Width::qword ? wide_bit : 0U) |
                      ((reg >> 3) * register_bit) | (base >> 3) | ((index >> 3) * index_bit);
  if (prefix != rex_prefix || (width == Width::byte && base >= 4)) { byte(prefix); }
}

void Code::indexed(const unsigned reg, const IndexedAddress address) {
  if (address.index == Reg::sp) { throw std::logic_error("RSP cannot index a guest word"); }
  if (!std::in_range<std::int32_t>(address.offset)) {
    throw std::logic_error("x64 indexed displacement overflow");
  }
  constexpr std::size_t maximum_small = 127;
  const bool small = address.offset <= maximum_small;
  byte((small ? mod_small : mod_large) | ((reg & 7U) << 3) | 4U);
  byte((word_scale << 6) | ((static_cast<unsigned>(address.index) & 7U) << 3) |
       (static_cast<unsigned>(address.base) & 7U));
  if (small) {
    byte(static_cast<std::uint32_t>(address.offset));
  } else {
    u32(static_cast<std::uint32_t>(address.offset));
  }
}

void Code::load_indexed(const Reg destination, const IndexedAddress address, const Width width) {
  rex(width,
      static_cast<unsigned>(destination),
      static_cast<unsigned>(address.base),
      static_cast<unsigned>(address.index));
  byte(mov_load);
  indexed(static_cast<unsigned>(destination), address);
}

void Code::store_indexed(const IndexedAddress address, const Reg source, const Width width) {
  rex(width,
      static_cast<unsigned>(source),
      static_cast<unsigned>(address.base),
      static_cast<unsigned>(address.index));
  byte(mov_register);
  indexed(static_cast<unsigned>(source), address);
}

void Code::memory(const unsigned reg, const Reg base, const std::size_t offset) {
  if (!std::in_range<std::int32_t>(offset)) { throw std::logic_error("x64 displacement overflow"); }
  constexpr std::size_t maximum_small = 127;
  const bool small = offset <= maximum_small;
  const auto address = static_cast<unsigned>(base) & 7;
  byte((small ? mod_small : mod_large) | ((reg & 7) << 3) | address);
  if (address == 4) { byte(stack_index); }
  if (small) {
    byte(static_cast<std::uint32_t>(offset));
  } else {
    u32(static_cast<std::uint32_t>(offset));
  }
}

void Code::mov(const RegisterPair registers, const Width width) {
  const auto src = static_cast<unsigned>(registers.source);
  const auto dst = static_cast<unsigned>(registers.destination);
  rex(width, src, dst);
  byte(mov_register);
  byte(mod_register | ((src & 7) << 3) | (dst & 7));
}

void Code::constant(const Reg destination, const std::uint64_t value) {
  constexpr std::uint64_t minimum_signed32 = 0xffffffff80000000;
  const auto dst = static_cast<unsigned>(destination);
  if (value <= std::numeric_limits<std::uint32_t>::max()) {
    // A 32-bit MOV clears the complete upper half without changing flags.
    rex(Width::dword, 0, dst);
    byte(mov_immediate + (dst & 7));
    u32(static_cast<std::uint32_t>(value));
  } else if (value >= minimum_signed32) {
    rex(Width::qword, 0, dst);
    byte(store_other);
    byte(mod_register | (dst & 7));
    u32(static_cast<std::uint32_t>(value));
  } else {
    rex(Width::qword, 0, dst);
    byte(mov_immediate + (dst & 7));
    u64(value);
  }
}

void Code::load(const Reg destination,
                const Reg base,
                const std::size_t offset,
                const Width width) {
  rex(width, static_cast<unsigned>(destination), static_cast<unsigned>(base));
  byte(mov_load);
  memory(static_cast<unsigned>(destination), base, offset);
}

void Code::store(const Reg base, const std::size_t offset, const Reg source, const Width width) {
  rex(width, static_cast<unsigned>(source), static_cast<unsigned>(base));
  byte(mov_register);
  memory(static_cast<unsigned>(source), base, offset);
}

void Code::packed(const Packed operation, const Xmm destination, const Xmm source) {
  byte(operand_prefix);
  rex(Width::dword, static_cast<unsigned>(destination), static_cast<unsigned>(source));
  raw({0x0f, static_cast<std::uint8_t>(operation)});
  byte(mod_register | ((static_cast<unsigned>(destination) & 7) << 3) |
       (static_cast<unsigned>(source) & 7));
}

void Code::packed_shift_right(const Xmm destination, const unsigned count) {
  constexpr unsigned packed_shift = 0x73;
  constexpr unsigned logical_right_group = 2;
  byte(operand_prefix);
  rex(Width::dword, logical_right_group, static_cast<unsigned>(destination));
  raw({0x0f, packed_shift});
  byte(mod_register | (logical_right_group << 3) | (static_cast<unsigned>(destination) & 7));
  byte(count);
}

void Code::scalar_from(const Xmm destination, const Reg source, const Width width) {
  constexpr unsigned move_dword = 0x6e;
  byte(operand_prefix);
  rex(width, static_cast<unsigned>(destination), static_cast<unsigned>(source));
  raw({0x0f, move_dword});
  byte(mod_register | ((static_cast<unsigned>(destination) & 7) << 3) |
       (static_cast<unsigned>(source) & 7));
}

void Code::scalar_to(const Reg destination, const Xmm source, const Width width) {
  constexpr unsigned move_qword = 0x7e;
  byte(operand_prefix);
  rex(width, static_cast<unsigned>(source), static_cast<unsigned>(destination));
  raw({0x0f, move_qword});
  byte(mod_register | ((static_cast<unsigned>(source) & 7) << 3) |
       (static_cast<unsigned>(destination) & 7));
}

void Code::scalar_load(const Xmm destination,
                       const Reg base,
                       const std::size_t offset,
                       const Width width) {
  constexpr unsigned move_qword_prefix = 0xf3;
  constexpr unsigned move_qword = 0x7e;
  constexpr unsigned move_dword = 0x6e;
  byte(width == Width::qword ? move_qword_prefix : operand_prefix);
  rex(Width::dword, static_cast<unsigned>(destination), static_cast<unsigned>(base));
  raw({0x0f, static_cast<std::uint8_t>(width == Width::qword ? move_qword : move_dword)});
  memory(static_cast<unsigned>(destination), base, offset);
}

void Code::packed_load(const Xmm destination, const Reg base, const std::size_t offset) {
  constexpr unsigned unaligned_prefix = 0xf3;
  constexpr unsigned packed_load_opcode = 0x6f;
  byte(unaligned_prefix);
  rex(Width::dword, static_cast<unsigned>(destination), static_cast<unsigned>(base));
  raw({0x0f, packed_load_opcode});
  memory(static_cast<unsigned>(destination), base, offset);
}

void Code::packed_store(const Reg base, const std::size_t offset, const Xmm source) {
  constexpr unsigned unaligned_prefix = 0xf3;
  constexpr unsigned packed_store_opcode = 0x7f;
  byte(unaligned_prefix);
  rex(Width::dword, static_cast<unsigned>(source), static_cast<unsigned>(base));
  raw({0x0f, packed_store_opcode});
  memory(static_cast<unsigned>(source), base, offset);
}

void Code::store_constant(const Reg base,
                          const std::size_t offset,
                          const Width width,
                          const std::uint32_t value) {
  rex(width, 0, static_cast<unsigned>(base));
  byte(width == Width::byte ? store_byte : store_other);
  memory(0, base, offset);
  if (width == Width::byte) {
    byte(value);
  } else if (width == Width::word) {
    byte(value & low_byte_mask);
    byte((value >> byte_bits) & low_byte_mask);
  } else {
    u32(value);
  }
}

void Code::memory_immediate(const unsigned group,
                            const Reg base,
                            const std::size_t offset,
                            const Width width,
                            const std::uint32_t value) {
  const auto opcode = immediate_opcode(width, value);
  rex(width, group, static_cast<unsigned>(base));
  byte(opcode);
  memory(group, base, offset);
  immediate_value(*this, width, value, opcode != alu_immediate);
}

void Code::binary(const std::uint32_t opcode, const RegisterPair registers, const Width width) {
  const auto dst = static_cast<unsigned>(registers.destination);
  const auto src = static_cast<unsigned>(registers.source);
  rex(width, src, dst);
  byte(opcode);
  byte(mod_register | ((src & 7) << 3) | (dst & 7));
}

void Code::immediate(const unsigned group,
                     const Reg destination,
                     const std::uint32_t value,
                     const Width width) {
  const auto dst = static_cast<unsigned>(destination);
  const auto opcode = immediate_opcode(width, value);
  rex(width, group, dst);
  if (destination == Reg::ax && (width == Width::byte || opcode == alu_immediate)) {
    constexpr unsigned accumulator_byte = 4;
    constexpr unsigned accumulator_other = 5;
    byte((width == Width::byte ? accumulator_byte : accumulator_other) + (group * byte_bits));
  } else {
    byte(opcode);
    byte(mod_register | (group << 3) | (dst & 7));
  }
  immediate_value(*this, width, value, opcode != alu_immediate);
}

void Code::shift(const unsigned group,
                 const Reg destination,
                 const unsigned count,
                 const Width width) {
  const auto dst = static_cast<unsigned>(destination);
  rex(width, group, dst);
  byte(shift_immediate);
  byte(mod_register | (group << 3) | (dst & 7));
  byte(count);
}

void Code::multiply(const RegisterPair registers) {
  const auto dst = static_cast<unsigned>(registers.destination);
  const auto src = static_cast<unsigned>(registers.source);
  rex(Width::qword, dst, src);
  raw({0x0f, multiply_opcode});
  byte(mod_register | ((dst & 7) << 3) | (src & 7));
}

void Code::unary(const unsigned group, const Reg destination) {
  const auto dst = static_cast<unsigned>(destination);
  rex(Width::qword, group, dst);
  byte(unary_opcode);
  byte(mod_register | (group << 3) | (dst & 7));
}

void Code::set(const unsigned condition, const Reg destination) {
  const auto dst = static_cast<unsigned>(destination);
  rex(Width::byte, 0, dst);
  byte(0x0f);
  byte(condition);
  byte(mod_register | (dst & 7));
  // MOVZX r32,r8 leaves the condition flags intact.
  rex(Width::byte, dst, dst);
  raw({0x0f, extend_byte});
  byte(mod_register | ((dst & 7) << 3) | (dst & 7));
}

void Code::move_if(const unsigned condition, const RegisterPair registers) {
  const auto dst = static_cast<unsigned>(registers.destination);
  const auto src = static_cast<unsigned>(registers.source);
  rex(Width::qword, dst, src);
  raw({0x0f});
  byte(condition);
  byte(mod_register | ((dst & 7U) << 3) | (src & 7U));
}

void Code::patch(const std::size_t offset, const std::size_t target) {
  const auto delta =
      static_cast<std::int64_t>(target) - static_cast<std::int64_t>(offset + sizeof(std::uint32_t));
  if (!std::in_range<std::int32_t>(delta)) {
    throw std::logic_error("x64 branch displacement overflow");
  }
  const auto displacement = static_cast<std::uint32_t>(delta);
  for (unsigned index = 0; index < sizeof(displacement); ++index) {
    bytes_.at(offset + index) =
        static_cast<std::uint8_t>((displacement >> (index * byte_bits)) & low_byte_mask);
  }
}

void Code::reference(Label &label) {
  const auto offset = bytes_.size();
  u32(0);
  if (label.bound) {
    patch(offset, label.offset);
  } else {
    label.references.push_back(offset);
  }
}

void Code::jump_if(const Branch condition, Label &label) {
  byte(0x0f);
  byte(static_cast<unsigned>(condition));
  reference(label);
}

void Code::jump(Label &label) {
  constexpr unsigned near_jump = 0xe9;
  byte(near_jump);
  reference(label);
}

void Code::call(Label &label) {
  constexpr unsigned near_call = 0xe8;
  byte(near_call);
  reference(label);
}

void Code::bind(Label &label) {
  if (label.bound) { throw std::logic_error("x64 label already bound"); }
  label.offset = bytes_.size();
  label.bound = true;
  for (const auto offset : label.references) { patch(offset, label.offset); }
  label.references.clear();
}

} // namespace dsp56300::detail::x64
