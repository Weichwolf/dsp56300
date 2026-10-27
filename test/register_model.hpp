// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-register and quotient/remainder bus model for fixture generators.
#pragma once
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::register_model {
enum RegisterId : std::uint8_t {
  reg_r_last = 23,
  reg_n_base = 24,
  end_modifiers = 40,
  reg_ep = 42,
  reg_vba = 48,
  reg_sc = 49,
  reg_sz = 56,
  reg_sr = 57,
  reg_omr = 58,
  reg_sp = 59,
  reg_ssh = 60,
  reg_ssl = 61,
  reg_la = 62,
  reg_lc = 63
};

inline constexpr unsigned extension_shift = 48;
inline constexpr Word pointer_mask = 63, extension_fill = 0xffff00;
inline constexpr std::uint64_t seed_accumulator_a = 0x7e345678abcdef;
inline constexpr std::uint64_t seed_accumulator_b = 0x813456abcdef12;
inline constexpr std::uint64_t seed_x = 0x345678fedcba, seed_y = 0x876543123456;
inline constexpr Word frame_high = 0x123400, frame_low = 0x567800, loop_address = 0xabc;
inline constexpr Word cursor = 0x180, datum = 0xfedcba;

inline bool valid(const unsigned number) {
  return (number >= 4 && number < end_modifiers) || number == reg_ep || number == reg_vba ||
         number == reg_sc || (number >= reg_sz && number < 64);
}

inline Registers seed(const Word mode) {
  auto result = initial_state();
  result.a = seed_accumulator_a;
  result.b = seed_accumulator_b;
  result.x = seed_x;
  result.y = seed_y;
  result.sr = mode | (status_byte & ~(flag_s | flag_l));
  result.omr = 5;
  result.ep = cursor;
  result.vba = frame_high;
  result.sz = loop_address;
  result.la = loop_address;
  result.lc = 7;
  result.sp = 2;
  result.sc = 2;
  for (std::size_t index = 0; index < agu_registers; ++index) {
    result.r[index] = cursor + static_cast<Word>(index);
    result.n[index] = 3;
  }
  for (std::size_t index = 0; index < stack_entries; ++index) {
    result.stack[index] = (std::uint64_t{frame_high + index} << word_bits) | (frame_low + index);
  }
  return result;
}

inline Word bus_word(Registers &state, const std::uint64_t raw) {
  const unsigned bits =
      word_bits + ((state.sr & mode_down) != 0 ? 1U : 0U) - ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  const auto value = signed56(raw);
  auto whole = value / unit;
  if (value % unit < 0) { --whole; }
  const unsigned growth_bit = bits + word_bits - 3;
  if (((raw >> growth_bit) & 1) != ((raw >> (growth_bit + 1)) & 1)) { state.sr |= flag_s; }
  const auto maximum = static_cast<std::int64_t>(sign24 - 1);
  const auto minimum = -static_cast<std::int64_t>(sign24);
  if (whole < minimum || whole > maximum) { state.sr |= flag_l; }
  return static_cast<Word>(std::clamp(whole, minimum, maximum)) & word_mask;
}

inline Word read(Registers &state, const unsigned number) {
  if (number >= 16 && number < reg_n_base) { return state.r.at(number - 16); }
  if (number >= reg_n_base && number < 32) { return state.n.at(number - reg_n_base); }
  if (number >= 32 && number < end_modifiers) { return state.m.at(number - 32); }
  switch (number) {
    case 4: return static_cast<Word>(state.x) & word_mask;
    case 5: return static_cast<Word>(state.x >> word_bits);
    case 6: return static_cast<Word>(state.y) & word_mask;
    case 7: return static_cast<Word>(state.y >> word_bits);
    case 8: return static_cast<Word>(state.a) & word_mask;
    case 9: return static_cast<Word>(state.b) & word_mask;
    case 10:
    case 11: {
      const auto top = static_cast<Word>((number == 10 ? state.a : state.b) >> extension_shift);
      return top < 128 ? top : top | extension_fill;
    }
    case 12: return static_cast<Word>(state.a >> word_bits) & word_mask;
    case 13: return static_cast<Word>(state.b >> word_bits) & word_mask;
    case 14: return bus_word(state, state.a);
    case 15: return bus_word(state, state.b);
    case reg_ep: return state.ep;
    case reg_vba: return state.vba;
    case reg_sc: return state.sc;
    case reg_sz: return state.sz;
    case reg_sr: return state.sr;
    case reg_omr: return state.omr;
    case reg_sp: return state.sp;
    case reg_ssh: {
      const auto value = static_cast<Word>(state.stack.at(state.sp % stack_entries) >> word_bits);
      state.sp = (state.sp + pointer_mask) % 64;
      --state.sc;
      return value;
    }
    case reg_ssl: return static_cast<Word>(state.stack.at(state.sp % stack_entries)) & word_mask;
    case reg_la: return state.la;
    case reg_lc: return state.lc;
    default: throw std::logic_error("Invalid model source register");
  }
}

inline void write(Registers &state, const unsigned number, const Word value) {
  if (number >= 16 && number < reg_n_base) {
    state.r.at(number - 16) = value;
    return;
  }
  if (number >= reg_n_base && number < 32) {
    state.n.at(number - reg_n_base) = value;
    return;
  }
  if (number >= 32 && number < end_modifiers) {
    state.m.at(number - 32) = value;
    return;
  }
  if (number >= 4 && number <= 7) {
    auto &bank = number < 6 ? state.x : state.y;
    const unsigned shift = number % 2 == 0 ? 0U : word_bits;
    bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
    return;
  }
  if (number >= 8 && number <= 13) {
    auto &bank = number % 2 == 0 ? state.a : state.b;
    unsigned shift = word_bits;
    if (number < 10) {
      shift = 0;
    } else if (number < 12) {
      shift = extension_shift;
    }
    const auto mask = number < 10 || number >= 12 ? std::uint64_t{word_mask} : std::uint64_t{0xff};
    bank = (bank & ~(mask << shift)) | ((std::uint64_t{value} & mask) << shift);
    return;
  }
  switch (number) {
    case 14:
    case 15:
      (number == 14 ? state.a : state.b) =
          static_cast<std::uint64_t>(signed24(value) * (std::int64_t{1} << word_bits)) &
          accumulator_mask;
      break;
    case reg_ep: state.ep = value; break;
    case reg_vba: state.vba = value; break;
    case reg_sc: state.sc = value % 32; break;
    case reg_sz: state.sz = value; break;
    case reg_sr: state.sr = value; break;
    case reg_omr: state.omr = value; break;
    case reg_sp: state.sp = value % 64; break;
    case reg_ssh:
      state.sp = (state.sp + 1) % 64;
      ++state.sc;
      state.stack.at(state.sp % stack_entries) =
          (std::uint64_t{value} << word_bits) |
          (state.stack.at(state.sp % stack_entries) & word_mask);
      break;
    case reg_ssl:
      state.stack.at(state.sp % stack_entries) =
          ((state.stack.at(state.sp % stack_entries) >> word_bits) << word_bits) | value;
      break;
    case reg_la: state.la = value; break;
    case reg_lc: state.lc = value; break;
    default: throw std::logic_error("Invalid model destination register");
  }
}

} // namespace dsp56300::test::register_model
