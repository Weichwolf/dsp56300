// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
struct ReverseOperands {
  Word original = 0, amount = 0;
  bool subtract = false;
};

Word reverse_add(const ReverseOperands input) noexcept {
  const auto operand = input.amount & word_mask;
  if (std::has_single_bit(operand)) {
    const auto lower = (operand << 1) - 1;
    const auto stops = (input.subtract ? input.original : ~input.original) & lower;
    // The forced LSB also handles a carry/borrow that leaves the 24-bit word.
    const auto last = Word{1} << (std::bit_width(stops | 1U) - 1);
    return input.original ^ ((operand << 1) - last);
  }
  const auto difference = input.original ^ operand;
  auto generated = (input.subtract ? ~input.original : input.original) & operand;
  auto propagated = input.subtract ? ~difference : difference;
  // Carry/borrow travels towards lower DSP bits. Combine 1,2,4,8,16-bit prefixes.
  for (unsigned distance = 1; distance < word_bits; distance *= 2) {
    generated |= propagated & (generated >> distance);
    propagated &= propagated >> distance;
  }
  return (difference ^ (generated >> 1)) & word_mask;
}

std::int32_t calculation_offset(const Registers &state, const Word opcode) {
  const auto index = (opcode >> 8) & 7U;
  const bool reverse = (state.m[index] & modifier_mask) == 0;
  if ((opcode & (1U << 14)) == 0) {
    constexpr Word upper_displacement = 0x70;
    const auto raw = ((opcode >> 7) & upper_displacement) | ((opcode >> 4) & 15U);
    const auto expanded = (raw & 64U) != 0 ? raw | (word_mask & ~Word{127}) : raw;
    return reverse ? static_cast<std::int32_t>(expanded) : signed_word(expanded);
  }
  const auto mode = (opcode >> 11) & 3U;
  if (mode >= 2) { return mode == 2 ? -1 : 1; }
  const auto offset =
      reverse ? static_cast<std::int32_t>(state.n[index]) : signed_word(state.n[index]);
  return mode == 0 ? -offset : offset;
}
}

Word Core::Impl::update_address(const AddressUpdate update) {
  const auto index = update.index;
  const auto offset = update.offset;
  const auto original = cpu.reg.r[index];
  const auto modifier = cpu.reg.m[index] & modifier_mask;
  if (modifier == modifier_mask) { return address_add(original, offset); }
  if (modifier == 0) {
    const auto amount = static_cast<Word>(offset < 0 ? -static_cast<std::int64_t>(offset) : offset);
    return reverse_add({.original = original, .amount = amount, .subtract = offset < 0});
  }
  if ((modifier & multiple_wrap_bit) != 0) {
    const auto mask = modifier & multiple_wrap_mask;
    if ((modifier & multiple_reserved_bit) != 0 || mask == 0 || !std::has_single_bit(mask + 1)) {
      fail(FaultKind::unsupported_mode, Space::program, current_pc, modifier);
      return original;
    }
    return (original & ~mask) | (address_add(original, offset) & mask);
  }
  const Word length = modifier + 1;
  const auto span_mask = std::bit_ceil(length) - 1;
  const auto magnitude = offset < 0 ? -static_cast<std::int64_t>(offset) : offset;
  // The documented P*2^k block stride has a positive original operand.
  // A negative Nn of exactly the modulus still takes the ordinary ring path.
  const bool block_stride = magnitude > length || (magnitude == length && update.positive_operand);
  if (block_stride && (static_cast<Word>(offset) & span_mask) == 0) {
    return address_add(original, offset);
  }
  const auto lower = original & ~span_mask;
  auto result = static_cast<std::int64_t>(original) + offset;
  if (result >= static_cast<std::int64_t>(lower) + length) {
    result -= length;
  } else if (result < lower) {
    result += length;
  }
  return static_cast<Word>(result) & word_mask;
}

Word Core::Impl::effective(const AddressForm form) {
  const auto mode = form.mode;
  const auto index = form.index;
  const auto extra = form.extra;
  const auto original = cpu.reg.r[index];
  switch (mode) {
    case 4: return original;
    case 6:
      if (index == 0 || index == 4) { return extra; }
      fail(FaultKind::invalid_instruction, Space::program, current_pc, index);
      return 0;
    case 0:
    case 1:
    case 2:
    case 3:
    case 5:
    case 7: break;
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, mode); return 0;
  }
  const auto modifier = cpu.reg.m[index] & modifier_mask;
  std::int32_t offset = 1;
  const bool register_offset = mode == 0 || mode == 1 || mode == 5;
  if (register_offset) {
    // Reverse carry reverses the raw operand before applying the addressing sign.
    // In particular N=$800000 is a positive reverse-carry operand, not -2^23.
    offset =
        modifier == 0 ? static_cast<std::int32_t>(cpu.reg.n[index]) : signed_word(cpu.reg.n[index]);
  }
  if (mode == 0 || mode == 2 || mode == 7) { offset = -offset; }
  const auto result = modifier == modifier_mask
                          ? address_add(original, offset)
                          : update_address({.index = index,
                                            .offset = offset,
                                            .positive_operand = !register_offset ||
                                                                signed_word(cpu.reg.n[index]) > 0});
  if (mode != 5) { cpu.reg.r[index] = result; }
  return mode == 5 || mode == 7 ? result : original;
}

void Core::Impl::execute_address_calculation(const Instruction &decoded) {
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  const auto opcode = decoded.opcode;
  const auto index = (opcode >> 8) & 7U;
  const bool relative = decoded.operation == Operation::load_relative_address;
  const bool displacement = !relative && (opcode & (1U << 14)) == 0;
  const auto destination = displacement ? r_register_base + (opcode & 15U) : opcode & five_bit_mask;
  if (destination < x0_register) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  Word result = 0;
  if (relative) {
    const auto offset = decoded.words == 2 ? decoded.extension : cpu.reg.r[index];
    result = address_add(current_pc, signed_word(offset));
  } else {
    const auto offset = calculation_offset(cpu.reg, opcode);
    const auto mode = (opcode >> 11) & 3U;
    const bool positive =
        displacement ? offset > 0 : mode >= 2 || signed_word(cpu.reg.n[index]) > 0;
    result = update_address({.index = index, .offset = offset, .positive_operand = positive});
  }
  if (fault.kind == FaultKind::none) { write_register(destination, result); }
}

} // namespace dsp56300
