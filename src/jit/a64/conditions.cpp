// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <stdexcept>

namespace dsp56300::detail::a64 {
void Emitter::condition(const Reg result, const unsigned selector) {
  const auto kind = selector & 7U;
  if (kind == 1 || kind == 7) {
    code_.shift(Shift::right, Reg::x9, status, 2);
    code_.alu(Alu::bit_xor, Reg::x9, Reg::x9, status);
    mask(Reg::x9, Reg::x9, overflow, Width::w32);
    code_.compare(Reg::x9, Reg::zr);
    code_.set(result, Condition::equal);
  } else {
    unsigned bits = 0;
    switch (kind) {
      case 0: bits = carry; break;
      case 2:
      case 4: bits = zero; break;
      case 3: bits = negative; break;
      case 5: bits = extension; break;
      case 6: bits = limit; break;
      default: throw std::logic_error("Unknown DSP condition");
    }
    mask(Reg::x9, status, bits, Width::w32);
    code_.compare(Reg::x9, Reg::zr);
    code_.set(result, Condition::equal);
  }
  if (kind == 4 || kind == 7) {
    mask(Reg::x9, status, kind == 4 ? unnormalized | extension : zero, Width::w32);
    code_.compare(Reg::x9, Reg::zr);
    code_.set(Reg::x9, kind == 4 ? Condition::not_equal : Condition::equal);
    code_.alu(Alu::bit_and, result, result, Reg::x9);
  }
  if ((selector & 8U) != 0) {
    code_.constant(Reg::x17, 1);
    code_.alu(Alu::bit_xor, result, result, Reg::x17);
  }
}
}
