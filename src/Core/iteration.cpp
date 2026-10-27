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
}

void Core::Impl::execute_iteration(const Instruction &decoded) {
  if (!numeric_mode_supported()) { return; }
  const auto opcode = decoded.opcode;
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  const auto before = destination;
  if (decoded.operation == Operation::divide_iteration) {
    const auto source = read_register(divisors[(opcode >> 4) & 3U]);
    const auto shifted = (before << 1) | (std::uint64_t{cpu.reg.sr & carry} << padding_bits);
    const bool opposite_signs = (before >> host_sign_bit) != (source >> (word_bits - 1));
    destination = opposite_signs ? wrapped_add(shifted, aligned_word(source))
                                 : wrapped_sub(shifted, aligned_word(source));
    cpu.reg.sr &= ~(carry | overflow);
    if ((destination >> host_sign_bit) == 0) { cpu.reg.sr |= carry; }
    if (((before ^ shifted) >> host_sign_bit) != 0) { cpu.reg.sr |= overflow | limit; }
    return;
  }
  const bool left = (cpu.reg.sr & extension) == 0;
  if (left && ((cpu.reg.sr & unnormalized) == 0 || (cpu.reg.sr & zero) != 0)) { return; }
  const auto saved_carry = cpu.reg.sr & carry;
  arithmetic_shift(before, {.count = 1, .destination_b = destination_b, .left = left});
  cpu.reg.sr = (cpu.reg.sr & ~carry) | saved_carry;
  const auto index = (opcode >> 8) & 7U;
  cpu.reg.r[index] = address_add(cpu.reg.r[index], left ? -1 : 1);
}
} // namespace dsp56300
