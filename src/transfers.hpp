// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace dsp56300 {
// Scalar identifiers follow the architectural six-bit selector table. Composite
// L: views are identifiers only, not six-bit selectors or another memory bank.
enum class RegisterId : std::uint8_t {
  none = 0,
  x0 = 4,
  x1 = 5,
  y0 = 6,
  y1 = 7,
  a0 = 8,
  b0 = 9,
  a2 = 10,
  b2 = 11,
  a1 = 12,
  b1 = 13,
  a = 14,
  b = 15,
  r0 = 16,
  r1 = 17,
  r2 = 18,
  r3 = 19,
  r4 = 20,
  r5 = 21,
  r6 = 22,
  r7 = 23,
  n0 = 24,
  n1 = 25,
  n2 = 26,
  n3 = 27,
  n4 = 28,
  n5 = 29,
  n6 = 30,
  n7 = 31,
  m0 = 32,
  m1 = 33,
  m2 = 34,
  m3 = 35,
  m4 = 36,
  m5 = 37,
  m6 = 38,
  m7 = 39,
  ep = 42,
  vba = 48,
  sc = 49,
  sz = 56,
  sr = 57,
  omr = 58,
  sp = 59,
  ssh = 60,
  ssl = 61,
  la = 62,
  lc = 63,
  x = 64,
  y = 65,
  a10 = 66,
  b10 = 67,
  ab = 68,
  ba = 69
};

enum class TransferSpace : std::uint8_t { program, x, y, long_pair };

enum class AddressKind : std::uint8_t {
  post_decrement_offset,
  post_increment_offset,
  post_decrement,
  post_increment,
  indirect,
  indexed,
  absolute_extension,
  pre_decrement,
  absolute,
  displacement,
  displacement_extension
};

struct EncodedAddress {
  AddressKind kind = AddressKind::absolute;
  std::uint8_t index = 0;
  // Literal absolute address or sign-extended short displacement. Extension
  // kinds deliberately contain no sampled extension value.
  Word literal = 0;
  bool operator==(const EncodedAddress &) const = default;
};

struct AddressInputs {
  std::uint8_t r = 0, n = 0, m = 0, updated_r = 0;
  bool operator==(const AddressInputs &) const = default;
};

// Architectural EA expression inputs/writeback, not hardware interlock/port
// masks. Register-relative kinds reject indices outside 0..7 before a shift.
// Unknown kinds return no description.
[[nodiscard]] std::optional<AddressInputs> address_inputs(EncodedAddress address) noexcept;

enum class TransferOperandKind : std::uint8_t { register_value, memory, immediate, extension };

struct TransferOperand {
  TransferOperandKind kind = TransferOperandKind::register_value;
  RegisterId reg = RegisterId::none;
  TransferSpace space = TransferSpace::x;
  EncodedAddress address{};
  Word immediate = 0;
  std::uint8_t immediate_shift = 0;
  bool operator==(const TransferOperand &) const = default;
};

struct EncodedTransfer {
  TransferOperand source{}, destination{};
  bool operator==(const EncodedTransfer &) const = default;
};

struct InstructionTransfers {
  InstructionShape instruction{};
  std::array<EncodedTransfer, 2> moves{};
  std::uint8_t count = 0;
  std::optional<EncodedAddress> address_update = std::nullopt;
  // A retained encoded byte, not a decoded ALU dependency/result or validity
  // claim. Concurrent ALU destinations and implicit effects need qualification.
  Word parallel_alu = 0;
  bool operator==(const InstructionTransfers &) const = default;
};

// Describes encoded data MOVE operands only. nullopt is an unsupported
// description, never evidence that an instruction has no operands/effects.
[[nodiscard]] std::optional<InstructionTransfers> inspect_transfers(Word opcode) noexcept;
} // namespace dsp56300
