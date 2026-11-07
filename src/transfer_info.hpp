// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"
#include "transfers.hpp"

namespace dsp56300::detail::transfer_info {
struct EffectiveEncoding {
  unsigned mode = 0, index = 0;
};

struct ImmediateEncoding {
  Word value = 0;
  unsigned shift = 0;
};

class Builder {
public:
  InstructionTransfers result;
  bool valid = true;

  [[nodiscard]] TransferOperand reg(unsigned selector) noexcept;
  [[nodiscard]] static TransferOperand view(RegisterId id) noexcept;
  [[nodiscard]] static TransferOperand literal(ImmediateEncoding encoded) noexcept;
  [[nodiscard]] static TransferOperand extension() noexcept;
  [[nodiscard]] static TransferOperand memory(TransferSpace space, EncodedAddress address) noexcept;
  [[nodiscard]] TransferOperand
  effective(TransferSpace space, EffectiveEncoding encoded, bool allow_immediate = false) noexcept;
  void move(EncodedTransfer transfer) noexcept;
};

[[nodiscard]] EffectiveEncoding effective_encoding(Word opcode) noexcept;
void parallel(Builder &builder, Word opcode) noexcept;
void control(Builder &builder, Word opcode, InstructionKind kind) noexcept;
void program(Builder &builder, Word opcode, InstructionKind kind) noexcept;
void peripheral(Builder &builder, Word opcode, InstructionKind kind) noexcept;
void displacement(Builder &builder, Word opcode, bool long_form) noexcept;
} // namespace dsp56300::detail::transfer_info
