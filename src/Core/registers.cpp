// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"

#include <bit>
#include <cstdint>

namespace dsp56300 {
using namespace detail;

Word Core::Impl::read_register(const unsigned number) {
  const auto &reg = cpu.reg;
  if (number >= r_register_base && number < n_register_base) {
    return reg.r[number - r_register_base];
  }
  if (number >= n_register_base && number < m_register_base) {
    return reg.n[number - n_register_base];
  }
  if (number >= m_register_base && number < m_register_base + agu_registers) {
    return reg.m[number - m_register_base];
  }
  switch (number) {
    case x0_register: return static_cast<Word>(reg.x) & word_mask;
    case x1_register: return static_cast<Word>(reg.x >> word_bits);
    case y0_register: return static_cast<Word>(reg.y) & word_mask;
    case y1_register: return static_cast<Word>(reg.y >> word_bits);
    case a0_register: return static_cast<Word>(reg.a >> padding_bits) & word_mask;
    case b0_register: return static_cast<Word>(reg.b >> padding_bits) & word_mask;
    case a1_register: return static_cast<Word>(reg.a >> main_shift) & word_mask;
    case b1_register: return static_cast<Word>(reg.b >> main_shift) & word_mask;
    case a2_register:
      return static_cast<Word>(std::bit_cast<std::int64_t>(reg.a) >> accumulator_bits) & word_mask;
    case b2_register:
      return static_cast<Word>(std::bit_cast<std::int64_t>(reg.b) >> accumulator_bits) & word_mask;
    case a_register: return transfer_word(reg.a);
    case b_register: return transfer_word(reg.b);
    case ep_register: return reg.ep;
    case vba_register: return reg.vba;
    case sc_register: return reg.sc;
    case sz_register: return reg.sz;
    case sr_register: return reg.sr;
    case omr_register: return reg.omr;
    case sp_register: return reg.sp;
    case la_register: return reg.la;
    case lc_register: return reg.lc;
    case ssh_register: return static_cast<Word>(pop() >> word_bits);
    case ssl_register:
      return static_cast<Word>(reg.stack[reg.sp & (stack_entries - 1)]) & word_mask;
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, number); return 0;
  }
}

void Core::Impl::write_register(const unsigned number, const Word value) {
  auto &reg = cpu.reg;
  if (number >= r_register_base && number < n_register_base) {
    reg.r[number - r_register_base] = value;
    return;
  }
  if (number >= n_register_base && number < m_register_base) {
    reg.n[number - n_register_base] = value;
    return;
  }
  if (number >= m_register_base && number < m_register_base + agu_registers) {
    reg.m[number - m_register_base] = value;
    return;
  }
  constexpr std::uint64_t main_mask = std::uint64_t{word_mask} << main_shift;
  constexpr std::uint64_t low_mask = std::uint64_t{word_mask} << padding_bits;
  constexpr std::uint64_t high_mask = std::uint64_t{0xff} << accumulator_bits;
  switch (number) {
    case x0_register: reg.x = (reg.x & ~std::uint64_t{word_mask}) | value; break;
    case x1_register: reg.x = (reg.x & word_mask) | (std::uint64_t{value} << word_bits); break;
    case y0_register: reg.y = (reg.y & ~std::uint64_t{word_mask}) | value; break;
    case y1_register: reg.y = (reg.y & word_mask) | (std::uint64_t{value} << word_bits); break;
    case a0_register: reg.a = (reg.a & ~low_mask) | (std::uint64_t{value} << padding_bits); break;
    case b0_register: reg.b = (reg.b & ~low_mask) | (std::uint64_t{value} << padding_bits); break;
    case a1_register: reg.a = (reg.a & ~main_mask) | (std::uint64_t{value} << main_shift); break;
    case b1_register: reg.b = (reg.b & ~main_mask) | (std::uint64_t{value} << main_shift); break;
    case a2_register:
      reg.a = (reg.a & ~high_mask) | (std::uint64_t{value} << accumulator_bits);
      break;
    case b2_register:
      reg.b = (reg.b & ~high_mask) | (std::uint64_t{value} << accumulator_bits);
      break;
    case a_register: reg.a = aligned_word(value); break;
    case b_register: reg.b = aligned_word(value); break;
    case ep_register: reg.ep = value; break;
    case vba_register: reg.vba = value; break;
    case sc_register: reg.sc = value & (stack_entries * 2 - 1); break;
    case sz_register: reg.sz = value; break;
    case sr_register: reg.sr = value; break;
    case omr_register: reg.omr = value; break;
    case sp_register: reg.sp = value & (stack_entries * 4 - 1); break;
    case la_register: reg.la = value; break;
    case lc_register: reg.lc = value; break;
    case ssh_register: {
      const auto next = (reg.sp + 1) & (stack_entries - 1);
      push(value, static_cast<Word>(reg.stack[next]) & word_mask);
      break;
    }
    case ssl_register:
      reg.stack[reg.sp & (stack_entries - 1)] =
          (reg.stack[reg.sp & (stack_entries - 1)] & ~std::uint64_t{word_mask}) | value;
      break;
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, number); break;
  }
}

} // namespace dsp56300
