// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word lsr_operation = 0x23, lsl_operation = 0x33;
constexpr Word ror_operation = 0x27, rol_operation = 0x37;
constexpr unsigned arithmetic_count_mask = 63, logical_count_mask = 31;
constexpr unsigned arithmetic_source_bit = 7, register_source_bit = 4;
constexpr std::uint64_t main_word_mask = std::uint64_t{word_mask} << main_shift;
constexpr std::array controls{
    a1_register, b1_register, x0_register, y0_register, x1_register, y1_register};
}

void Core::Impl::arithmetic_shift(const std::uint64_t value, const ShiftControl control) {
  if (control.count >= accumulator_bits) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, control.count);
    return;
  }
  const auto count = control.count;
  auto result = value;
  bool carried = false;
  bool overflowed = false;
  if (count != 0) {
    const auto signed_value = std::bit_cast<std::int64_t>(value);
    if (control.left) {
      result = value << count;
      carried = ((value >> (host_sign_bit + 1 - count)) & 1) != 0;
      const auto lost = signed_value >> (host_sign_bit - count);
      overflowed = lost != 0 && lost != -1;
    } else {
      result = static_cast<std::uint64_t>(signed_value >> count) & padding_mask;
      carried = ((value >> (padding_bits + count - 1)) & 1) != 0;
    }
  }
  arithmetic_flags(result, overflowed);
  if ((cpu.reg.sr & arithmetic_saturation) != 0) { saturate_arithmetic(result); }
  (control.destination_b ? cpu.reg.b : cpu.reg.a) = result;
  cpu.reg.sr = (cpu.reg.sr & ~carry) | (carried ? carry : 0);
}

void Core::Impl::logical_shift(const ShiftControl control, const bool rotate) {
  if (control.count > word_bits) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, control.count);
    return;
  }
  auto &accumulator = control.destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = static_cast<Word>(accumulator >> main_shift) & word_mask;
  auto result = before;
  bool carried = false;
  if (control.count != 0) {
    carried = control.left ? ((before >> (word_bits - control.count)) & 1) != 0
                           : ((before >> (control.count - 1)) & 1) != 0;
    result = control.left ? (before << control.count) & word_mask : before >> control.count;
    if (rotate) {
      const auto previous = cpu.reg.sr & carry;
      result |= control.left ? previous : previous << (word_bits - 1);
    }
  }
  accumulator = (accumulator & ~main_word_mask) | (std::uint64_t{result} << main_shift);
  logical_flags(result);
  cpu.reg.sr = (cpu.reg.sr & ~carry) | (carried ? carry : 0);
}

bool Core::Impl::word_shift(const Word opcode) {
  const auto operation = opcode & 0xf7U;
  if (operation != lsr_operation && operation != lsl_operation && operation != ror_operation &&
      operation != rol_operation) {
    return false;
  }
  // Parallel moves may already have written this accumulator: shift the latched input.
  const bool destination_b = (opcode & 8U) != 0;
  (destination_b ? cpu.reg.b : cpu.reg.a) = latched[destination_b ? 1 : 0];
  logical_shift({.destination_b = destination_b, .left = (opcode & 16U) != 0}, (opcode & 4U) != 0);
  return true;
}

void Core::Impl::execute_shift(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto operation = decoded.operation;
  const bool arithmetic = operation == Operation::arithmetic_shift_immediate ||
                          operation == Operation::arithmetic_shift_register;
  if (!numeric_mode_supported(arithmetic ? arithmetic_saturation : 0)) { return; }
  const bool immediate = operation == Operation::arithmetic_shift_immediate ||
                         operation == Operation::logical_shift_immediate;
  unsigned count = opcode >> 1;
  if (!immediate) {
    const auto selector = (opcode >> 1) & 7;
    if (selector < 2) {
      fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
      return;
    }
    count = read_register(controls[selector - 2]);
  }
  bool left = (opcode & (1U << 5)) == 0;
  if (immediate) { left = arithmetic ? (opcode & (1U << 8)) != 0 : (opcode & (1U << 6)) == 0; }
  const ShiftControl control{.count =
                                 count & (arithmetic ? arithmetic_count_mask : logical_count_mask),
                             .destination_b = (opcode & 1U) != 0,
                             .left = left};
  if (arithmetic) {
    const bool source_b =
        (opcode & (1U << (immediate ? arithmetic_source_bit : register_source_bit))) != 0;
    arithmetic_shift(source_b ? cpu.reg.b : cpu.reg.a, control);
  } else {
    logical_shift(control);
  }
}
} // namespace dsp56300
