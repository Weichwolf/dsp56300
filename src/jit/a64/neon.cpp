// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/neon.hpp"

#include "jit/a64/code.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
namespace {
constexpr unsigned source_shift = 5, right_shift = 16, lane_shift = 20;
constexpr unsigned memory_shift = 10;

constexpr auto reg(const Vec value) {
  return static_cast<std::uint32_t>(value);
}
}

void Neon::binary(const VectorOp operation,
                  const Vec destination,
                  const Vec left,
                  const Vec right) {
  code_.word(static_cast<std::uint32_t>(operation) | (reg(right) << right_shift) |
             (reg(left) << source_shift) | reg(destination));
}

void Neon::move(const Vec destination, const Vec operand) {
  binary(VectorOp::bit_or, destination, operand, operand);
}

void Neon::zero(const Vec destination) {
  binary(VectorOp::bit_xor, destination, destination, destination);
}

void Neon::shift(const Vec destination,
                 const Vec operand,
                 const Shift operation,
                 const unsigned count,
                 const bool words) {
  const unsigned width = words ? 32U : 64U;
  if (count >= width) { throw std::invalid_argument("NEON shift exceeds lane width"); }
  if (count == 0) {
    move(destination, operand);
    return;
  }
  const bool left = operation == Shift::left;
  if (!left && operation != Shift::right && operation != Shift::arithmetic) {
    throw std::invalid_argument("Unsupported NEON shift");
  }
  constexpr std::uint32_t shl = 0x4f005400;
  constexpr std::uint32_t sshr = 0x4f000400;
  constexpr std::uint32_t ushr = 0x6f000400;
  const auto right_opcode = operation == Shift::right ? ushr : sshr;
  const auto opcode = left ? shl : right_opcode;
  const auto amount = left ? width + count : (2 * width) - count;
  code_.word(opcode | (amount << right_shift) | (reg(operand) << source_shift) | reg(destination));
}

void Neon::narrow(const Vec destination, const Vec operand) {
  constexpr std::uint32_t xtn = 0x0ea12800;
  code_.word(xtn | (reg(operand) << source_shift) | reg(destination));
}

void Neon::extend(const Vec destination, const Vec operand) {
  constexpr std::uint32_t sshll = 0x0f20a400;
  code_.word(sshll | (reg(operand) << source_shift) | reg(destination));
}

void Neon::broadcast(const Vec destination, const Reg operand) {
  constexpr std::uint32_t dup = 0x4e080c00;
  code_.word(dup | (static_cast<unsigned>(operand) << source_shift) | reg(destination));
}

void Neon::insert(const Vec destination, const Reg operand, const unsigned lane) {
  if (lane > 1) { throw std::invalid_argument("Invalid NEON doubleword lane"); }
  constexpr std::uint32_t ins = 0x4e081c00;
  code_.word(ins | (lane << lane_shift) | (static_cast<unsigned>(operand) << source_shift) |
             reg(destination));
}

void Neon::extract(const Reg destination, const Vec operand, const unsigned lane) {
  if (lane > 1) { throw std::invalid_argument("Invalid NEON doubleword lane"); }
  constexpr std::uint32_t umov = 0x4e083c00;
  code_.word(umov | (lane << lane_shift) | (reg(operand) << source_shift) |
             static_cast<unsigned>(destination));
}

void Neon::load(const Vec destination, const Reg base, const std::size_t offset, const bool words) {
  memory(destination, base, offset, words, MemoryOp::load);
}

void Neon::store(const Vec operand, const Reg base, const std::size_t offset, const bool words) {
  memory(operand, base, offset, words, MemoryOp::store);
}

void Neon::memory(
    const Vec value, Reg base, std::size_t offset, const bool words, const MemoryOp operation) {
  const unsigned scale = words ? 8U : 16U;
  constexpr std::size_t maximum = 4095;
  if (offset % scale != 0) { throw std::invalid_argument("Unaligned NEON displacement"); }
  if (offset / scale > maximum) {
    if (base == Reg::sp) {
      code_.add(Reg::x16, Reg::sp, 0, Width::x64);
    } else {
      code_.move(Reg::x16, base);
    }
    code_.constant(Reg::x17, offset);
    code_.alu(Alu::add, Reg::x16, Reg::x16, Reg::x17, Width::x64);
    base = Reg::x16;
    offset = 0;
  }
  constexpr std::uint32_t str_q = 0x3d800000;
  constexpr std::uint32_t str_d = 0xfd000000;
  constexpr std::uint32_t load_bit = 0x00400000;
  const auto opcode = (words ? str_d : str_q) | (operation == MemoryOp::load ? load_bit : 0U);
  code_.word(opcode | (static_cast<std::uint32_t>(offset / scale) << memory_shift) |
             (static_cast<unsigned>(base) << source_shift) | reg(value));
}
}
