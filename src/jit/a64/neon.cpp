// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/neon.hpp"

#include "jit/a64/code.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
namespace {
constexpr unsigned source_shift = 5, right_shift = 16;
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

bool Neon::constant_words(const Vec destination, const std::uint32_t value) {
  constexpr unsigned byte_bits = 8;
  constexpr std::uint32_t byte_mask = 255;
  constexpr std::uint32_t movi = 0x4f000400;
  constexpr std::uint32_t inverse_bit = 0x20000000;
  constexpr unsigned low_bits = 5;
  constexpr unsigned mode_shift = 12;
  for (const bool inverse : {false, true}) {
    const auto bits = inverse ? ~value : value;
    const auto emit = [&](const std::uint32_t immediate, const unsigned mode) {
      code_.word(movi | (inverse ? inverse_bit : 0U) | (mode << mode_shift) |
                 ((immediate >> low_bits) << right_shift) |
                 ((immediate & ((1U << low_bits) - 1)) << source_shift) | reg(destination));
    };
    for (unsigned lane = 0; lane < 4; ++lane) {
      const auto shift = lane * byte_bits;
      const auto byte = (bits >> shift) & byte_mask;
      if (bits == (byte << shift)) {
        emit(byte, lane * 2);
        return true;
      }
    }
    for (const unsigned shift : {8U, 16U}) {
      const auto low = (std::uint32_t{1} << shift) - 1;
      const auto byte = bits >> shift;
      if (byte <= byte_mask && (bits & low) == low) {
        emit(byte, shift == byte_bits ? 12U : 13U);
        return true;
      }
    }
  }
  return false;
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

void Neon::narrow(const Vec destination, const Vec operand, const bool upper) {
  constexpr std::uint32_t xtn = 0x0ea12800;
  constexpr std::uint32_t high = 0x40000000;
  code_.word(xtn | (upper ? high : 0U) | (reg(operand) << source_shift) | reg(destination));
}

void Neon::extend(const Vec destination, const Vec operand, const bool upper) {
  constexpr std::uint32_t sshll = 0x0f20a400;
  constexpr std::uint32_t high = 0x40000000;
  code_.word(sshll | (upper ? high : 0U) | (reg(operand) << source_shift) | reg(destination));
}

void Neon::round_narrow(const Vec destination,
                        const Vec operand,
                        const unsigned count,
                        const bool upper) {
  constexpr unsigned width = 32;
  if (count == 0 || count > width) { throw std::invalid_argument("Invalid NEON rounding shift"); }
  constexpr std::uint32_t rshrn = 0x0f008c00;
  constexpr std::uint32_t high = 0x40000000;
  code_.word(rshrn | (upper ? high : 0U) | ((2 * width - count) << right_shift) |
             (reg(operand) << source_shift) | reg(destination));
}

void Neon::broadcast(const Vec destination, const Reg operand, const bool words) {
  constexpr std::uint32_t dup = 0x4e000c00;
  const auto size = words ? 4U : 8U;
  code_.word(dup | (size << right_shift) | (static_cast<unsigned>(operand) << source_shift) |
             reg(destination));
}

void Neon::insert(const Vec destination, const Reg operand, const unsigned lane, const bool words) {
  if (lane >= (words ? 4U : 2U)) { throw std::invalid_argument("Invalid NEON insertion lane"); }
  constexpr std::uint32_t ins = 0x4e001c00;
  const auto size = words ? 4U : 8U;
  const auto element = size | (lane * size * 2);
  code_.word(ins | (element << right_shift) | (static_cast<unsigned>(operand) << source_shift) |
             reg(destination));
}

void Neon::extract(const Reg destination,
                   const Vec operand,
                   const unsigned lane,
                   const bool words) {
  if (lane >= (words ? 4U : 2U)) { throw std::invalid_argument("Invalid NEON extraction lane"); }
  constexpr std::uint32_t umov = 0x0e003c00;
  constexpr std::uint32_t wide = 0x40000000;
  const auto size = words ? 4U : 8U;
  const auto element = size | (lane * size * 2);
  code_.word(umov | (words ? 0U : wide) | (element << right_shift) |
             (reg(operand) << source_shift) | static_cast<unsigned>(destination));
}

void Neon::load(const Vec destination, const Reg base, const std::size_t offset, const bool words) {
  memory(destination, base, offset, words, MemoryOp::load);
}

void Neon::store(const Vec operand, const Reg base, const std::size_t offset, const bool words) {
  memory(operand, base, offset, words, MemoryOp::store);
}

void Neon::push_doubles(const Vec left, const Vec right) {
  constexpr std::uint32_t stp = 0x6dbf03e0;
  constexpr unsigned pair_shift = 10;
  code_.word(stp | (reg(right) << pair_shift) | reg(left));
}

void Neon::pop_doubles(const Vec left, const Vec right) {
  constexpr std::uint32_t ldp = 0x6cc103e0;
  constexpr unsigned pair_shift = 10;
  code_.word(ldp | (reg(right) << pair_shift) | reg(left));
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
