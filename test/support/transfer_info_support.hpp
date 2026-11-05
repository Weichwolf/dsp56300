// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "instruction_decoder.hpp"
#include "support.hpp"
#include "transfers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>

namespace dsp56300::test::transfer_info_test {
// Independent FM12-14 register-selector expectations, including reserved holes.
inline constexpr std::array<RegisterId, 64> registers{
    RegisterId::none, RegisterId::none, RegisterId::none, RegisterId::none, RegisterId::x0,
    RegisterId::x1,   RegisterId::y0,   RegisterId::y1,   RegisterId::a0,   RegisterId::b0,
    RegisterId::a2,   RegisterId::b2,   RegisterId::a1,   RegisterId::b1,   RegisterId::a,
    RegisterId::b,    RegisterId::r0,   RegisterId::r1,   RegisterId::r2,   RegisterId::r3,
    RegisterId::r4,   RegisterId::r5,   RegisterId::r6,   RegisterId::r7,   RegisterId::n0,
    RegisterId::n1,   RegisterId::n2,   RegisterId::n3,   RegisterId::n4,   RegisterId::n5,
    RegisterId::n6,   RegisterId::n7,   RegisterId::m0,   RegisterId::m1,   RegisterId::m2,
    RegisterId::m3,   RegisterId::m4,   RegisterId::m5,   RegisterId::m6,   RegisterId::m7,
    RegisterId::none, RegisterId::none, RegisterId::ep,   RegisterId::none, RegisterId::none,
    RegisterId::none, RegisterId::none, RegisterId::none, RegisterId::vba,  RegisterId::sc,
    RegisterId::none, RegisterId::none, RegisterId::none, RegisterId::none, RegisterId::none,
    RegisterId::none, RegisterId::sz,   RegisterId::sr,   RegisterId::omr,  RegisterId::sp,
    RegisterId::ssh,  RegisterId::ssl,  RegisterId::la,   RegisterId::lc};
inline constexpr std::array modes{AddressKind::post_decrement_offset,
                                  AddressKind::post_increment_offset,
                                  AddressKind::post_decrement,
                                  AddressKind::post_increment,
                                  AddressKind::indirect,
                                  AddressKind::indexed,
                                  AddressKind::absolute_extension,
                                  AddressKind::pre_decrement};
inline std::size_t cases = 0;

inline TransferOperand reg(const RegisterId id) {
  return {.reg = id};
}

inline TransferOperand memory(const TransferSpace space, const EncodedAddress address) {
  return {.kind = TransferOperandKind::memory, .space = space, .address = address};
}

struct Effective {
  unsigned mode, index;
};

inline TransferOperand ea(const TransferSpace space, const Effective encoded) {
  return memory(space,
                {.kind = modes.at(encoded.mode),
                 .index = static_cast<std::uint8_t>(encoded.mode == 6 ? 0U : encoded.index)});
}

inline TransferOperand absolute(const TransferSpace space, const Word address) {
  return memory(space, {.literal = address});
}

inline TransferOperand immediate(const Word value, const std::uint8_t shift = 0) {
  return {.kind = TransferOperandKind::immediate, .immediate = value, .immediate_shift = shift};
}

inline TransferOperand extension() {
  return {.kind = TransferOperandKind::extension};
}

inline EncodedTransfer directed(const EncodedTransfer load, const bool read) {
  return read ? load : EncodedTransfer{.source = load.destination, .destination = load.source};
}

inline InstructionTransfers expected(const InstructionKind kind,
                                     const unsigned words,
                                     const std::initializer_list<EncodedTransfer> moves) {
  InstructionTransfers result{.instruction = {.kind = kind,
                                              .words = static_cast<std::uint8_t>(words),
                                              .repeatable = words == 1}};
  for (const auto move : moves) { result.moves.at(result.count++) = move; }
  return result;
}

inline void verify(const Word opcode, const InstructionTransfers &wanted) {
  ++cases;
  const auto actual = inspect_transfers(opcode);
  if (!actual) {
    throw std::runtime_error("Missing encoded MOVE operands, opcode=" + std::to_string(opcode));
  }
  require(actual.has_value(), "Independent encoding has its described data MOVE component");
  if (*actual != wanted) {
    throw std::runtime_error("Encoded MOVE mismatch, opcode=" + std::to_string(opcode));
  }
  require(*actual == wanted,
          "Full independent family/width/operand/EA/update/ALU-byte description matches");
}

inline void unsupported(const Word opcode) {
  ++cases;
  require(!inspect_transfers(opcode),
          "Unsupported description never becomes an empty successful set");
}

void scalar();
void register_moves();
void address_updates();
void paired();
void mixed();
void long_moves();
void control();
void program();
void peripheral();
void displacement();
void literals();
} // namespace dsp56300::test::transfer_info_test
