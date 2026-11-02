// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr std::array divisors{x0_register, y0_register, x1_register, y1_register};
constexpr std::array shift_sources{
    a1_register, b1_register, x0_register, y0_register, x1_register, y1_register};
}

void Core::Impl::normalize_fast(const Word opcode) {
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  const auto selector = (opcode >> 1) & 7U;
  if (selector < 2) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto shift = signed_word(read_register(shift_sources[selector - 2]));
  if (shift < -static_cast<int>(accumulator_bits - 1) ||
      shift > static_cast<int>(accumulator_bits)) {
    fail(FaultKind::unsupported_mode, Space::program, current_pc, opcode);
    return;
  }
  const bool destination_b = (opcode & 1U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto saved_carry = cpu.reg.sr & carry;
  mark_alu_accumulator(destination_b);
  if (shift == static_cast<int>(accumulator_bits)) {
    destination = (destination >> host_sign_bit) != 0 ? padding_mask : 0;
    arithmetic_flags(destination, false);
  } else {
    arithmetic_shift(destination,
                     {.count = static_cast<unsigned>(shift < 0 ? -shift : shift),
                      .destination_b = destination_b,
                      .left = shift < 0,
                      .saturate = false});
  }
  cpu.reg.sr = (cpu.reg.sr & ~carry) | saved_carry;
  omit_alu_status(carry);
}

void Core::Impl::execute_iteration(const Instruction &decoded) {
  const auto qualified_modes = decoded.operation == Operation::normalize_iteration
                                   ? arithmetic_saturation | compatibility_mode
                                   : 0U;
  if (!numeric_mode_supported(qualified_modes)) { return; }
  const auto opcode = decoded.opcode;
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = destination;
  if (decoded.operation == Operation::divide_iteration) {
    const auto source = read_register(divisors[(opcode >> 4) & 3U]);
    const auto shifted = (before << 1) | (std::uint64_t{cpu.reg.sr & carry} << padding_bits);
    const bool opposite_signs = (before >> host_sign_bit) != (source >> (word_bits - 1));
    mark_alu_accumulator(destination_b);
    destination = opposite_signs ? wrapped_add(shifted, aligned_word(source))
                                 : wrapped_sub(shifted, aligned_word(source));
    mark_alu(RegisterStorage::sr, carry | overflow);
    cpu.reg.sr &= ~(carry | overflow);
    if ((destination >> host_sign_bit) == 0) { cpu.reg.sr |= carry; }
    if (((before ^ shifted) >> host_sign_bit) != 0) {
      mark_alu(RegisterStorage::sr, overflow | limit);
      cpu.reg.sr |= overflow | limit;
    }
    return;
  }
  const bool left = (cpu.reg.sr & extension) == 0;
  if (left && ((cpu.reg.sr & unnormalized) == 0 || (cpu.reg.sr & zero) != 0)) {
    // FM13-146/12.4.2: no shift retains data/Rn, while result flags still refresh.
    arithmetic_flags(before, false);
    return;
  }
  const auto saved_carry = cpu.reg.sr & carry;
  arithmetic_shift(before, {.count = 1, .destination_b = destination_b, .left = left});
  cpu.reg.sr = (cpu.reg.sr & ~carry) | saved_carry;
  omit_alu_status(carry);
  const auto index = (opcode >> 8) & 7U;
  mark_alu_address(index);
  constexpr Word compatibility_address_mask = 0xffff; // FM4.2: clear the upper eight AGU bits.
  const auto mask = (cpu.reg.sr & compatibility_mode) != 0 ? compatibility_address_mask : word_mask;
  cpu.reg.r[index] = address_add(cpu.reg.r[index], left ? -1 : 1) & mask;
}
} // namespace dsp56300
