// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr unsigned width_shift = 12, half_word_bits = 12;
constexpr Word half_word_mask = 0xfff, unsigned_extract = 0x80;
constexpr std::array controls{
    a1_register, b1_register, x0_register, y0_register, x1_register, y1_register};
constexpr std::array sources{
    a0_register, b0_register, x0_register, y0_register, x1_register, y1_register};
}

Word Core::Impl::bit_field_control(const Instruction &decoded) {
  if (decoded.words == 2) { return decoded.extension; }
  const auto selector = (decoded.opcode >> 1) & 7U;
  if (selector < 2) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return 0;
  }
  return read_register(controls.at(selector - 2));
}

void Core::Impl::extract_field(const Instruction &decoded, const Word control) {
  const unsigned offset = control & six_bit_mask;
  const unsigned width = (control >> width_shift) & six_bit_mask;
  if (offset + width > accumulator_bits) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, control);
    return;
  }
  const bool unsigned_result = (decoded.opcode & unsigned_extract) != 0;
  if (width == 0 && !unsigned_result) {
    // Rev.5 does not define a sign bit for an empty field. Keep this valid-but-
    // unqualified corner explicit until independently resolved, not guessed.
    fail(FaultKind::unsupported_mode, Space::program, current_pc, control);
    return;
  }
  const auto source = ((decoded.opcode & 16U) != 0 ? cpu.reg.b : cpu.reg.a) >> padding_bits;
  const auto mask = (std::uint64_t{1} << width) - 1;
  auto result = (source >> offset) & mask;
  if (!unsigned_result && (result & (std::uint64_t{1} << (width - 1))) != 0) { result |= ~mask; }
  result = (result & accumulator_mask) << padding_bits;

  ((decoded.opcode & 1U) != 0 ? cpu.reg.b : cpu.reg.a) = result;
  arithmetic_flags(result, false);

  cpu.reg.sr &= ~carry;
}

void Core::Impl::insert_field(const Instruction &decoded, const Word control) {
  const auto selector = (decoded.opcode >> 4) & 7U;
  const unsigned offset = control & six_bit_mask;
  const unsigned width = (control >> width_shift) & six_bit_mask;
  if (selector < 2 || width > word_bits || offset + width > accumulator_bits) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return;
  }
  const auto source = read_register(sources.at(selector - 2));
  const auto field_mask = (std::uint64_t{1} << width) - 1;
  const auto mask = field_mask << offset;
  auto &destination = (decoded.opcode & 1U) != 0 ? cpu.reg.b : cpu.reg.a;
  const auto raw =
      ((destination >> padding_bits) & ~mask) | ((std::uint64_t{source} & field_mask) << offset);

  destination = raw << padding_bits;
  arithmetic_flags(destination, false);

  cpu.reg.sr &= ~carry;
}

void Core::Impl::merge_field(const Instruction &decoded, const Word source) {
  auto &destination = (decoded.opcode & 1U) != 0 ? cpu.reg.b : cpu.reg.a;
  const Word result = ((source & half_word_mask) << half_word_bits) |
                      (static_cast<Word>(destination >> main_shift) & half_word_mask);
  constexpr auto mask = std::uint64_t{word_mask} << main_shift;

  destination = (destination & ~mask) | (std::uint64_t{result} << main_shift);
  logical_flags(result);
}

void Core::Impl::count_leading_bits(const Word opcode) {
  const auto source = (opcode & 2U) != 0 ? cpu.reg.b : cpu.reg.a;
  constexpr int exponent_bias = 9;
  const auto leading = std::countl_zero((source >> host_sign_bit) == 0 ? source : ~source);
  const auto count = source == 0 ? 0 : exponent_bias - leading;

  ((opcode & 1U) != 0 ? cpu.reg.b : cpu.reg.a) = static_cast<std::uint64_t>(count) << main_shift;
  logical_flags(static_cast<Word>(count) & word_mask);
}

void Core::Impl::execute_bit_field(const Instruction &decoded) {
  // All BFU operations explicitly bypass arithmetic saturation (section 3.2.3).
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  if (decoded.operation == Operation::count_leading_bits) {
    count_leading_bits(decoded.opcode);
    return;
  }
  const auto control = bit_field_control(decoded);
  if (fault.kind != FaultKind::none) { return; }
  if (decoded.operation == Operation::bit_field_extract) {
    extract_field(decoded, control);
  } else if (decoded.operation == Operation::bit_field_insert) {
    insert_field(decoded, control);
  } else if (decoded.operation == Operation::bit_field_merge) {
    merge_field(decoded, control);
  } else {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
  }
}
} // namespace dsp56300
