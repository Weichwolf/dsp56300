// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <cstdint>

namespace dsp56300::detail {
// DSP56300 Family Manual, table 12-14: six-bit register selectors.
enum RegisterNumber : std::uint8_t {
  x0_register = 4,
  x1_register = 5,
  y0_register = 6,
  y1_register = 7,
  a0_register = 8,
  b0_register = 9,
  a2_register = 10,
  b2_register = 11,
  a1_register = 12,
  b1_register = 13,
  a_register = 14,
  b_register = 15,
  r_register_base = 16,
  n_register_base = 24,
  m_register_base = 32,
  ep_register = 42,
  vba_register = 48,
  sc_register = 49,
  sz_register = 56,
  sr_register = 57,
  omr_register = 58,
  sp_register = 59,
  ssh_register = 60,
  ssl_register = 61,
  la_register = 62,
  lc_register = 63,
};

inline constexpr unsigned main_shift = 32;
inline constexpr Word byte_mask = 0xff, five_bit_mask = 31, six_bit_mask = 63;
inline constexpr Word modifier_mask = 0xffff, multiple_wrap_bit = 0x8000;
inline constexpr Word multiple_wrap_mask = 0x3fff;
inline constexpr Word multiple_reserved_bit = 0x4000;
inline constexpr unsigned high_register_mask = 0x18;
inline constexpr Word parallel_group_mask = 0xe00000, immediate_group = 0x200000;
inline constexpr Word register_group_mask = 0xfc0000, update_group_mask = 0xffe000;
inline constexpr Word update_group = 0x204000, no_move_group = 0x200000;
inline constexpr Word upper_move_mask = 0xffff00, pair_group_mask = 0xf00000;
inline constexpr Word pair_group = 0x100000, memory_group_mask = 0xc00000, memory_group = 0x400000;
inline constexpr Word class_two_mask = 0xfe4000, class_two = 0x080000;
inline constexpr Word long_selector_mask = 0x340000;
} // namespace dsp56300::detail
