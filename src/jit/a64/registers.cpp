// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
namespace {
std::size_t register_offset(const unsigned number) {
  if (number >= r_register_base && number < n_register_base) {
    return offsetof(Registers, r) + ((number - r_register_base) * sizeof(Word));
  }
  if (number >= n_register_base && number < m_register_base) {
    return offsetof(Registers, n) + ((number - n_register_base) * sizeof(Word));
  }
  if (number >= m_register_base && number < m_register_base + agu_registers) {
    return offsetof(Registers, m) + ((number - m_register_base) * sizeof(Word));
  }
  switch (number) {
    case ep_register: return offsetof(Registers, ep);
    case vba_register: return offsetof(Registers, vba);
    case sc_register: return offsetof(Registers, sc);
    case sz_register: return offsetof(Registers, sz);
    case omr_register: return offsetof(Registers, omr);
    case sp_register: return offsetof(Registers, sp);
    case la_register: return offsetof(Registers, la);
    case lc_register: return offsetof(Registers, lc);
    default: throw std::logic_error("Invalid A64 register offset");
  }
}
}

bool Emitter::valid_register(const unsigned number) {
  return (number >= x0_register && number < m_register_base + agu_registers) ||
         number == ep_register || number == vba_register || number == sc_register ||
         (number >= sz_register && number <= lc_register);
}

void Emitter::stack_slot(const bool next) {
  code_.load(Reg::x10, cpu, offsetof(Registers, sp));
  if (next) { code_.add(Reg::x10, Reg::x10, 1); }
  mask(Reg::x11, Reg::x10, stack_entries - 1);
  code_.shift(Shift::left, Reg::x11, Reg::x11, 3, Width::x64);
  code_.alu(Alu::add, Reg::x11, cpu, Reg::x11, Width::x64);
}

void Emitter::read_register(const BlockInstruction &operation, const unsigned number) {
  if (!valid_register(number)) {
    code_.constant(Reg::x0, number);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
  } else if (number <= b_register) {
    read_data(number, operation.mode);
  } else if (number == sr_register) {
    code_.move(Reg::x0, status, Width::w32);
  } else if (number == ssh_register) {
    pop_frame(operation);
    code_.shift(Shift::right, Reg::x0, Reg::x0, word_bits, Width::x64);
  } else if (number == ssl_register) {
    stack_slot(false);
    code_.load(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  } else {
    code_.load(Reg::x0, cpu, register_offset(number));
  }
}

void Emitter::write_register(const BlockInstruction &operation, const unsigned number) {
  if (!valid_register(number)) {
    code_.constant(Reg::x0, number);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
  } else if (number <= b_register) {
    write_data(number);
  } else if (number == sr_register) {
    code_.move(status, Reg::x0, Width::w32);
    boundary(NativeBoundary::owner);
  } else if (number == ssh_register) {
    stack_slot(true);
    code_.load(Reg::x9, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    mask(Reg::x9, Reg::x9, word_mask, Width::w32);
    code_.shift(Shift::left, Reg::x0, Reg::x0, word_bits, Width::x64);
    code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x9, Width::x64);
    push_frame(operation);
  } else if (number == ssl_register) {
    stack_slot(false);
    code_.load(Reg::x9, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    mask(Reg::x9, Reg::x9, ~std::uint64_t{word_mask});
    code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x9, Width::x64);
    code_.store(Reg::x0, Reg::x11, offsetof(Registers, stack), Access::doubleword);
  } else {
    if (number == vba_register) { mask(Reg::x0, Reg::x0, word_mask ^ byte_mask, Width::w32); }
    if (number == sc_register) { mask(Reg::x0, Reg::x0, (stack_entries * 2) - 1, Width::w32); }
    if (number == sp_register) { mask(Reg::x0, Reg::x0, (stack_entries * 4) - 1, Width::w32); }
    code_.store(Reg::x0, cpu, register_offset(number));
    if (number == omr_register || number == la_register) { boundary(NativeBoundary::owner); }
  }
}
}
