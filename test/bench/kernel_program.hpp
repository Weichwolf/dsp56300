// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "kernel_reference.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace dsp56300::test::kernels {
inline constexpr Word entry = 0x1000, input = 0x1000, output = 0x1000;
inline constexpr Word coefficients = 16, states = 32, irq_vector = 8;
inline constexpr Word program_words = 0x4000, data_words = 0x50000;
template <isa::Opcode operation> inline constexpr Word op = static_cast<Word>(operation);
inline constexpr Word read = Word{1} << 15, effective = Word{1} << 14, y_space = Word{1} << 19;
inline constexpr Word postincrement = 3U << 11;

inline constexpr Word byte_mask = 0xff, register_high_mask = 0x18;
inline constexpr unsigned register_high_shift = 17;

constexpr Word single(const Word reg) {
  return op<isa::Opcode::move_long> | ((reg & 7U) << 16) |
         ((reg & register_high_mask) << register_high_shift);
}

constexpr Word transfer(const Word source, const Word destination) {
  return op<isa::Opcode::move_none> | (source << 13) | (destination << 8);
}

constexpr Word loop(const std::size_t count) {
  return op<isa::Opcode::do_immediate> | (static_cast<Word>(count & byte_mask) << 8) |
         static_cast<Word>(count >> 8);
}

inline constexpr Word mac = op<isa::Opcode::mac_register_a> | (5U << 4);
inline constexpr Word subtract_product = op<isa::Opcode::mac_register_a> | (6U << 4) | 4U;
// X:(R0)+ -> X0 and Y:(R4)+ -> Y0, simultaneous with old-operand ALU execution.
inline constexpr Word dual_load = op<isa::Opcode::move_xy> | (7U << 20) | read | postincrement;
inline constexpr std::array irq{op<isa::Opcode::add_x1_b>,
                                single(15)}; // ADD X1,B; MOVE B,X:0. X1 holds one.

inline std::vector<Word> fir() {
  std::vector<Word> result{loop(core_kernels::blocks),
                           0,
                           loop(core_kernels::frames),
                           0,
                           dual_load | (op<isa::Opcode::clr_a> & byte_mask),
                           loop(core_kernels::taps - 1),
                           0};
  result[6] = entry + static_cast<Word>(result.size());
  const std::array body{dual_load | (mac & byte_mask),
                        mac,
                        op<isa::Opcode::rnd_a>,
                        single(14) | y_space | effective | postincrement | (1U << 8),
                        op<isa::Opcode::move_address_update>,
                        op<isa::Opcode::move_address_update> | (4U << 8)};
  result.insert(result.end(), body.begin(), body.end());
  result[3] = entry + static_cast<Word>(result.size()) - 1;
  result.push_back(op<isa::Opcode::nop>);
  result[1] = entry + static_cast<Word>(result.size()) - 1;
  result.push_back(op<isa::Opcode::stop>);
  return result;
}

inline std::vector<Word> biquads() {
  std::vector<Word> result{loop(core_kernels::blocks),
                           0,
                           loop(core_kernels::frames),
                           0,
                           single(4) | read | effective | postincrement};
  for (Word section = 0; section < core_kernels::sections; ++section) {
    const auto state = states + (2 * section);
    const auto coefficient = coefficients + (5 * section);
    const auto load_state = [state](const Word slot, const Word reg) {
      return single(reg) | read | ((state + slot) << 8);
    };
    const auto store_state = [state](const Word slot) {
      return single(14) | ((state + slot) << 8);
    };
    const auto weight = [coefficient](const Word slot) {
      return single(6) | y_space | read | ((coefficient + slot) << 8);
    };
    const auto low = [](const Word value) { return value & byte_mask; };
    const std::array body{load_state(0, 5),
                          load_state(1, 7),
                          weight(0) | low(op<isa::Opcode::tfr_x1_a>),
                          weight(1) | low(mac),
                          op<isa::Opcode::rnd_a>,
                          transfer(14, 5) | low(op<isa::Opcode::tfr_y1_a>),
                          weight(3) | low(mac),
                          weight(2) | low(subtract_product),
                          op<isa::Opcode::rnd_a>,
                          store_state(0) | low(op<isa::Opcode::clr_a>),
                          weight(4) | low(mac),
                          subtract_product,
                          op<isa::Opcode::rnd_a>,
                          store_state(1),
                          transfer(5, 4)};
    result.insert(result.end(), body.begin(), body.end());
  }
  result.push_back(single(4) | y_space | effective | postincrement | (1U << 8));
  result[3] = entry + static_cast<Word>(result.size()) - 1;
  result.push_back(op<isa::Opcode::nop>);
  result[1] = entry + static_cast<Word>(result.size()) - 1;
  result.push_back(op<isa::Opcode::stop>);
  return result;
}
} // namespace dsp56300::test::kernels
