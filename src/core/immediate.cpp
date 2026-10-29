// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <cstdint>

namespace dsp56300 {
using namespace detail;

void Core::Impl::execute_immediate(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const Word input = decoded.words == 2 ? decoded.extension : (opcode >> 8) & six_bit_mask;
  const auto kind = opcode & 7;
  const bool qualified = kind == 0 || kind == 4 || kind == 2 || kind == 3 || kind == 6;
  if (!numeric_mode_supported(qualified ? arithmetic_saturation : 0)) { return; }
  const bool destination_b = (opcode & 8U) != 0;
  auto &destination = destination_b ? cpu.reg.b : cpu.reg.a;
  if (kind == 0 || kind == 4 || kind == 5) {
    add_operands({.left = destination,
                  .right = aligned_word(input),
                  .destination_b = destination_b,
                  .subtract = kind != 0,
                  .compare = kind == 5});
    return;
  }
  constexpr std::uint64_t main_mask = std::uint64_t{word_mask} << main_shift;
  auto result = static_cast<Word>(destination >> main_shift) & word_mask;
  switch (kind) {
    case 2: result |= input; break;
    case 3: result ^= input; break;
    case 6: result &= input; break;
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode); return;
  }
  destination = (destination & ~main_mask) | (std::uint64_t{result} << main_shift);
  logical_flags(result);
}
} // namespace dsp56300
