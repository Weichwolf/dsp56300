// SPDX-License-Identifier: GPL-3.0-only
#include "jit/code.hpp"

#include <cstdint>

namespace dsp56300::detail::x64 {
namespace {
constexpr unsigned map_mask = 31, register_mask = 7, vector_register_mask = 15;
constexpr unsigned vector_prefix = 0xc4, register_mode = 0xc0, byte_mask = 255;
constexpr unsigned prefix_repeat = 2;

enum EncodingByte : std::uint8_t {
  shift_dword = 0x72,
  shift_qword = 0x73,
  load_opcode = 0x6f,
  store_opcode = 0x7f,
  broadcast = 0x59,
  broadcast_dword = 0x58,
  pack_words = 0xc6,
  extract = 0x39,
  extract_scalar = 0x16,
  load_dword = 0x6e,
  insert_dword = 0x22,
  short_vex = 0xc5,
  unused_vex = 0xf8,
  clear_upper = 0x77
};

unsigned number(const Xmm reg) {
  return static_cast<unsigned>(reg);
}

unsigned number(const Reg reg) {
  return static_cast<unsigned>(reg);
}
}

void Code::vex(const VectorEncoding encoding, const VectorFields fields) {
  byte(vector_prefix);
  byte((((~fields.destination >> 3) & 1) << 7) | (((~fields.index >> 3) & 1) << 6) |
       (((~fields.base >> 3) & 1) << 5) | (encoding.map & map_mask));
  byte((static_cast<unsigned>(encoding.qword) << 7) | ((~fields.left & vector_register_mask) << 3) |
       (static_cast<unsigned>(encoding.wide) << 2) | encoding.prefix);
}

void Code::vector(const VectorOp operation, const VectorOperands registers, const bool wide) {
  const auto encoded = static_cast<unsigned>(operation);
  const auto destination = number(registers.destination);
  const auto source = number(registers.right);
  vex({.map = encoded >> 8, .wide = wide},
      {.destination = destination, .left = number(registers.left), .base = source});
  byte(encoded & byte_mask);
  byte(register_mode | ((destination & register_mask) << 3) | (source & register_mask));
}

void Code::vector_unary(const VectorOp operation,
                        const Xmm destination,
                        const Xmm source,
                        const bool wide) {
  vector(operation, {.destination = destination, .left = Xmm::x0, .right = source}, wide);
}

void Code::vector_immediate(const VectorOp operation,
                            const VectorOperands registers,
                            const unsigned immediate) {
  vector(operation, registers);
  byte(immediate);
}

void Code::vector_shift(const Xmm destination,
                        const Xmm source,
                        const VectorShift operation,
                        const unsigned bits,
                        const bool dwords) {
  const auto group = static_cast<unsigned>(operation);
  vex({}, {.destination = group, .left = number(destination), .base = number(source)});
  byte(dwords ? shift_dword : shift_qword);
  byte(register_mode | (group << 3) | (number(source) & register_mask));
  byte(bits);
}

void Code::vector_load(const Xmm destination, const VectorAddress address, const bool wide) {
  vex({.prefix = prefix_repeat, .wide = wide},
      {.destination = number(destination), .base = number(address.base)});
  byte(load_opcode);
  memory(number(destination), address.base, address.offset);
}

void Code::vector_store(const VectorAddress address, const Xmm source, const bool wide) {
  vex({.prefix = prefix_repeat, .wide = wide},
      {.destination = number(source), .base = number(address.base)});
  byte(store_opcode);
  memory(number(source), address.base, address.offset);
}

void Code::vector_broadcast(const Xmm destination, const VectorAddress address, const bool dwords) {
  vex({.map = 2}, {.destination = number(destination), .base = number(address.base)});
  byte(dwords ? broadcast_dword : broadcast);
  memory(number(destination), address.base, address.offset);
}

void Code::vector_pack(const Xmm destination, const Xmm low, const Xmm high) {
  constexpr unsigned select_low_dwords = 0x88;
  vex({.prefix = 0, .wide = false},
      {.destination = number(destination), .left = number(low), .base = number(high)});
  byte(pack_words);
  byte(register_mode | ((number(destination) & register_mask) << 3) |
       (number(high) & register_mask));
  byte(select_low_dwords);
}

void Code::vector_extract(const Xmm destination, const Xmm source, const unsigned lane) {
  vex({.map = 3}, {.destination = number(source), .base = number(destination)});
  byte(extract);
  byte(register_mode | ((number(source) & register_mask) << 3) |
       (number(destination) & register_mask));
  byte(lane);
}

void Code::vector_scalar(const Reg destination, const Xmm source, const unsigned lane) {
  vex({.map = 3, .wide = false, .qword = true},
      {.destination = number(source), .base = number(destination)});
  byte(extract_scalar);
  byte(register_mode | ((number(source) & register_mask) << 3) |
       (number(destination) & register_mask));
  byte(lane);
}

void Code::vector_load_dword(const Xmm destination, const VectorAddress address) {
  vex({.wide = false}, {.destination = number(destination), .base = number(address.base)});
  byte(load_dword);
  memory(number(destination), address.base, address.offset);
}

void Code::vector_insert_dword(const Xmm destination,
                               const IndexedAddress address,
                               const unsigned lane) {
  vex({.map = 3, .wide = false},
      {.destination = number(destination),
       .left = number(destination),
       .base = number(address.base),
       .index = number(address.index)});
  byte(insert_dword);
  indexed(number(destination), address);
  byte(lane);
}

void Code::vector_finish() {
  raw({short_vex, unused_vex, clear_upper});
}
} // namespace dsp56300::detail::x64
