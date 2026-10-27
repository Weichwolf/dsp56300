// SPDX-License-Identifier: GPL-3.0-only
#include "native_moves.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>

namespace dsp56300::detail {
namespace {
constexpr unsigned pair_bit = 23, y_mode_shift = 20, x_register_shift = 18, y_load_bit = 22;
constexpr unsigned high_register_shift = 17;
}

NativeMoves plan_moves(const Instruction &instruction) {
  NativeMoves result;
  if (instruction.operation != Operation::parallel) { return result; }
  const auto opcode = instruction.opcode;
  if ((opcode & (Word{1} << pair_bit)) != 0) {
    constexpr std::array xregs{x0_register, x1_register, a_register, b_register};
    constexpr std::array yregs{y0_register, y1_register, a_register, b_register};
    const auto xmode = (opcode >> 11) & 3;
    const auto ymode = (opcode >> y_mode_shift) & 3;
    const auto xindex = (opcode >> 8) & 7;
    result.transfers[0] = {.address = {.mode = xmode == 0 ? 4 : xmode, .index = xindex},
                           .reg = xregs.at((opcode >> x_register_shift) & 3),
                           .space = Space::x,
                           .load = (opcode & (Word{1} << 15)) != 0};
    result.transfers[1] = {.address = {.mode = ymode == 0 ? 4 : ymode,
                                       .index = ((opcode >> 13) & 3) + (xindex < 4 ? 4U : 0U)},
                           .reg = yregs.at((opcode >> 16) & 3),
                           .space = Space::y,
                           .load = (opcode & (Word{1} << y_load_bit)) != 0};
    result.count = 2;
    return result;
  }
  if ((opcode & memory_group_mask) != memory_group || (opcode & long_selector_mask) == 0) {
    return result;
  }
  const unsigned reg =
      ((opcode >> 16) & 7) | ((opcode >> high_register_shift) & high_register_mask);
  if ((reg < x0_register || reg > y1_register) && reg != a_register && reg != b_register) {
    return result;
  }
  const bool effective = (opcode & (Word{1} << 14)) != 0;
  const NativeMove move{
      .address = effective ? AddressForm{.mode = (opcode >> 11) & 7,
                                         .index = (opcode >> 8) & 7,
                                         .extra = instruction.extension}
                           : AddressForm{.mode = 6, .extra = (opcode >> 8) & six_bit_mask},
      .reg = reg,
      .space = (opcode & (Word{1} << 19)) != 0 ? Space::y : Space::x,
      .load = (opcode & (Word{1} << 15)) != 0};
  if (move.address.mode == 6 && move.address.index != 0 && !(move.immediate() && move.load)) {
    return result;
  }
  result.transfers[0] = move;
  result.count = 1;
  return result;
}
} // namespace dsp56300::detail
