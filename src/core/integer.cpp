// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"

#include <cstdint>

namespace dsp56300 {
using namespace detail;

void Core::Impl::execute_unit(const Instruction &decoded) {
  if (!numeric_mode_supported()) { return; }
  const bool destination_b = (decoded.opcode & 1U) != 0;
  add_operands({.left = destination_b ? cpu.reg.b : cpu.reg.a,
                .right = std::uint64_t{1} << padding_bits,
                .destination_b = destination_b,
                .subtract = decoded.operation == Operation::decrement});
}

void Core::Impl::compare_unsigned(const Word opcode) {
  // CMPU explicitly bypasses arithmetic saturation. Other unimplemented data
  // modes remain rejected rather than silently applying 24-bit semantics.
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  const auto selector = (opcode >> 1) & 7U;
  if (selector >= 1 && selector <= 3) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const bool destination_b = (opcode & 1U) != 0;
  const auto left = ((destination_b ? cpu.reg.b : cpu.reg.a) >> padding_bits) & long_mask;
  auto right = ((destination_b ? cpu.reg.a : cpu.reg.b) >> padding_bits) & long_mask;
  if (selector >= 4) {
    const auto bank = (selector & 1U) == 0 ? cpu.reg.x : cpu.reg.y;
    const auto word = (selector < 6 ? bank : bank >> word_bits) & word_mask;
    right = word << word_bits;
  }
  const auto result = wrapped_sub(left, right);
  Word flags = (result & long_mask) == 0 ? zero : 0;
  // Both unsigned operands are zero-extended to the arithmetic unit. Its bit
  // 55 (N), unlike Z's explicit low-48 test, is set precisely on a borrow.
  if (left < right) { flags |= carry | negative; }
  constexpr Word changed = carry | negative | zero | overflow;
  cpu.reg.sr = (cpu.reg.sr & ~changed) | flags;
}
} // namespace dsp56300
