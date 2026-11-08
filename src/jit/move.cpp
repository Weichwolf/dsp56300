// SPDX-License-Identifier: GPL-3.0-only
#include "move.hpp"

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <array>

namespace dsp56300::detail {
namespace {
constexpr unsigned pair_bit = 23, y_mode_shift = 20, x_register_shift = 18, y_load_bit = 22;
constexpr unsigned high_register_shift = 17;

NativeMoves long_moves(const Instruction &instruction) {
  const auto opcode = instruction.opcode;
  const bool load = (opcode & (Word{1} << 15)) != 0;
  const bool effective = (opcode & (Word{1} << 14)) != 0;
  const AddressForm address = effective
                                  ? AddressForm{.mode = (opcode >> 11) & 7,
                                                .index = (opcode >> 8) & 7,
                                                .extra = instruction.extension}
                                  : AddressForm{.mode = 6, .extra = (opcode >> 8) & six_bit_mask};
  if (address.mode == 6 && address.index != 0) { return {}; }
  const auto selector = ((opcode >> 16) & 3) | ((opcode >> 17) & 4);
  constexpr std::array high{a1_register,
                            b1_register,
                            x1_register,
                            y1_register,
                            a_register,
                            b_register,
                            a_register,
                            b_register};
  constexpr std::array low{a0_register,
                           b0_register,
                           x0_register,
                           y0_register,
                           a_register,
                           b_register,
                           b_register,
                           a_register};
  return {.transfers = {NativeMove{.address = address, .reg = high.at(selector), .load = load},
                        NativeMove{.address = address,
                                   .reg = low.at(selector),
                                   .space = Space::y,
                                   .load = load}},
          .count = 2,
          .long_pair = true};
}
}

bool pure_address_update(const Instruction &instruction) {
  return instruction.operation == Operation::parallel &&
         (instruction.opcode & update_group_mask) == update_group &&
         (instruction.opcode & byte_mask) == 0;
}

NativeAddressImmediate plan_address_immediate(const Instruction &instruction) {
  // Fixed memory-group, load, immediate-EA and zero ALU fields. Only the
  // five register-selector bits and the irrelevant X/Y bus bit can vary.
  constexpr Word mask = 0xc0ffff;
  constexpr Word immediate = 0x40f400;
  if (instruction.operation != Operation::parallel || instruction.words != 2 ||
      (instruction.opcode & mask) != immediate) {
    return {};
  }
  const auto reg =
      ((instruction.opcode >> 16) & 7) | ((instruction.opcode >> 17) & high_register_mask);
  if (reg < r_register_base || reg >= m_register_base) { return {}; }
  return {.destination = reg, .value = instruction.extension & word_mask};
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
  if ((opcode & memory_group_mask) != memory_group) { return result; }
  if ((opcode & long_selector_mask) == 0) { return long_moves(instruction); }
  const unsigned reg =
      ((opcode >> 16) & 7) | ((opcode >> high_register_shift) & high_register_mask);
  const bool load = (opcode & (Word{1} << 15)) != 0;
  const bool partial_store = !load && reg >= a0_register && reg <= b1_register;
  const bool address_register = reg >= r_register_base && reg < m_register_base;
  if ((reg < x0_register || reg > y1_register) && reg != a_register && reg != b_register) {
    if (!partial_store && !address_register) { return result; }
  }
  const bool effective = (opcode & (Word{1} << 14)) != 0;
  const NativeMove move{
      .address = effective ? AddressForm{.mode = (opcode >> 11) & 7,
                                         .index = (opcode >> 8) & 7,
                                         .extra = instruction.extension}
                           : AddressForm{.mode = 6, .extra = (opcode >> 8) & six_bit_mask},
      .reg = reg,
      .space = (opcode & (Word{1} << 19)) != 0 ? Space::y : Space::x,
      .load = load};
  if (move.address.mode == 6 && move.address.index != 0 && !(move.immediate() && move.load)) {
    return result;
  }
  result.transfers[0] = move;
  result.count = 1;
  return result;
}
} // namespace dsp56300::detail
