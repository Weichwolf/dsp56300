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
  const auto aligned_size = std::bit_ceil(length);
  const auto magnitude = offset < 0 ? -static_cast<std::int64_t>(offset) : offset;
  if (magnitude > length && (static_cast<Word>(offset) & (aligned_size - 1)) == 0) {
    return address_add(original, offset);
  }
  const auto lower = original & ~(aligned_size - 1);
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
  // Reverse carry reverses the raw operand before applying the addressing sign.
  // In particular N=$800000 is a positive reverse-carry operand, not -2^23.
  const auto offset = (cpu.reg.m[index] & modifier_mask) == 0
                          ? static_cast<std::int32_t>(cpu.reg.n[index])
                          : signed_word(cpu.reg.n[index]);
  switch (mode) {
    case 0: cpu.reg.r[index] = update_address({.index = index, .offset = -offset}); return original;
    case 1: cpu.reg.r[index] = update_address({.index = index, .offset = offset}); return original;
    case 2: cpu.reg.r[index] = update_address({.index = index, .offset = -1}); return original;
    case 3: cpu.reg.r[index] = update_address({.index = index, .offset = 1}); return original;
    case 4: return original;
    case 5: return update_address({.index = index, .offset = offset});
    case 6:
      if (index == 0 || index == 4) { return extra; }
      fail(FaultKind::invalid_instruction, Space::program, current_pc, index);
      return 0;
    case 7:
      cpu.reg.r[index] = update_address({.index = index, .offset = -1});
      return cpu.reg.r[index];
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, mode); return 0;
  }
}

} // namespace dsp56300
