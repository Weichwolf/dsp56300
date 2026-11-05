// SPDX-License-Identifier: GPL-3.0-only
#include "transfer_info.hpp"

#include "dsp56300/core.hpp"
#include "transfers.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace dsp56300 {
std::optional<AddressInputs> address_inputs(const EncodedAddress address) noexcept {
  if (address.kind == AddressKind::absolute || address.kind == AddressKind::absolute_extension) {
    return AddressInputs{};
  }
  if (address.index >= 8) { return std::nullopt; }
  const auto mask = static_cast<std::uint8_t>(1U << address.index);
  AddressInputs result{.r = mask, .m = mask};
  switch (address.kind) {
    case AddressKind::post_decrement_offset:
    case AddressKind::post_increment_offset:
      result.n = mask;
      result.updated_r = mask;
      break;
    case AddressKind::post_decrement:
    case AddressKind::post_increment:
    case AddressKind::pre_decrement: result.updated_r = mask; break;
    case AddressKind::indirect: result.m = 0; break;
    case AddressKind::indexed: result.n = mask; break;
    case AddressKind::displacement:
    case AddressKind::displacement_extension: break;
    case AddressKind::absolute:
    case AddressKind::absolute_extension: return AddressInputs{};
    default: return std::nullopt;
  }
  return result;
}
} // namespace dsp56300

namespace dsp56300::detail::transfer_info {
TransferOperand Builder::reg(const unsigned selector) noexcept {
  constexpr unsigned first = 4;
  constexpr unsigned contiguous_end = 40;
  constexpr unsigned extended_pointer = 42;
  constexpr unsigned vector_base = 48;
  constexpr unsigned stack_counter = 49;
  constexpr unsigned stack_size = 56;
  constexpr unsigned last = 63;
  const bool known = (selector >= first && selector < contiguous_end) ||
                     selector == extended_pointer || selector == vector_base ||
                     selector == stack_counter || (selector >= stack_size && selector <= last);
  if (!known) {
    valid = false;
    return {};
  }
  return view(static_cast<RegisterId>(selector));
}

TransferOperand Builder::view(const RegisterId id) noexcept {
  return {.kind = TransferOperandKind::register_value, .reg = id};
}

TransferOperand Builder::literal(const ImmediateEncoding encoded) noexcept {
  return {.kind = TransferOperandKind::immediate,
          .immediate = encoded.value,
          .immediate_shift = static_cast<std::uint8_t>(encoded.shift)};
}

TransferOperand Builder::extension() noexcept {
  return {.kind = TransferOperandKind::extension};
}

TransferOperand Builder::memory(const TransferSpace space, const EncodedAddress address) noexcept {
  return {.kind = TransferOperandKind::memory, .space = space, .address = address};
}

EffectiveEncoding effective_encoding(const Word opcode) noexcept {
  return {.mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U};
}

TransferOperand Builder::effective(const TransferSpace space,
                                   const EffectiveEncoding encoded,
                                   const bool allow_immediate) noexcept {
  if (encoded.mode == 6) {
    if (encoded.index == 4 && allow_immediate) { return extension(); }
    if (encoded.index != 0) {
      valid = false;
      return {};
    }
  }
  return memory(space,
                {.kind = static_cast<AddressKind>(encoded.mode),
                 .index = static_cast<std::uint8_t>(encoded.mode == 6 ? 0U : encoded.index)});
}

void Builder::move(const EncodedTransfer transfer) noexcept {
  if (result.count == result.moves.size()) {
    valid = false;
    return;
  }
  result.moves.at(static_cast<std::size_t>(result.count++)) = transfer;
}
} // namespace dsp56300::detail::transfer_info
