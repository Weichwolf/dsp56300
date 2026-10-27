// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"

#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
Word reverse_word(Word value) noexcept {
  Word result = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    result = (result << 1) | (value & 1);
    value >>= 1;
  }
  return result;
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
    const auto reversed = reverse_word(original);
    return reverse_word(address_add(reversed,
                                    offset < 0 ? -static_cast<std::int32_t>(reverse_word(amount))
                                               : static_cast<std::int32_t>(reverse_word(amount))));
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
  if (magnitude > length && (static_cast<Word>(offset) & span_mask) == 0) {
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
  if (mode == 0 || mode == 1 || mode == 5) {
    // Reverse carry reverses the raw operand before applying the addressing sign.
    // In particular N=$800000 is a positive reverse-carry operand, not -2^23.
    offset =
        modifier == 0 ? static_cast<std::int32_t>(cpu.reg.n[index]) : signed_word(cpu.reg.n[index]);
  }
  if (mode == 0 || mode == 2 || mode == 7) { offset = -offset; }
  const auto result = modifier == modifier_mask
                          ? address_add(original, offset)
                          : update_address({.index = index, .offset = offset});
  if (mode != 5) { cpu.reg.r[index] = result; }
  return mode == 5 || mode == 7 ? result : original;
}

} // namespace dsp56300
