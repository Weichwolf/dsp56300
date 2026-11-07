// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstdint>

namespace dsp56300::detail::a64 {
void Emitter::transfer_long(const Reg value, const Word mode) {
  transfer_word(value, mode);
  const unsigned shift =
      padding_bits + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  code_.shift(Shift::arithmetic, Reg::x0, value, shift, Width::x64);
  constexpr auto minimum = -(std::int64_t{1} << (double_word_bits - 1));
  constexpr auto maximum = (std::uint64_t{1} << (double_word_bits - 1)) - 1;
  code_.constant(Reg::x1, maximum);
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  code_.select(Reg::x0, Reg::x1, Reg::x0, Condition::greater, Width::x64);
  code_.constant(Reg::x1, static_cast<std::uint64_t>(minimum));
  code_.compare(Reg::x0, Reg::x1, Width::x64);
  code_.select(Reg::x0, Reg::x1, Reg::x0, Condition::less, Width::x64);
  mask(Reg::x0, Reg::x0, long_mask);
}

void Emitter::long_capture(const BlockInstruction &operation, const unsigned selector) {
  if (selector >= 6) {
    read_register(operation, selector == 6 ? a_register : b_register);
    code_.store(Reg::x0, Reg::sp, Frame::value);
    read_register(operation, selector == 6 ? b_register : a_register);
    code_.store(Reg::x0, Reg::sp, Frame::other_value);
    return;
  }
  if (selector >= 4) {
    transfer_long((selector & 1U) == 0 ? accumulator_a : accumulator_b, operation.mode);
  } else if (selector >= 2) {
    code_.move(Reg::x0, selector == 2 ? bank_x : bank_y);
  } else {
    code_.shift(Shift::right,
                Reg::x0,
                selector == 0 ? accumulator_a : accumulator_b,
                padding_bits,
                Width::x64);
  }
  mask(Reg::x1, Reg::x0, word_mask, Width::w32);
  code_.store(Reg::x1, Reg::sp, Frame::other_value);
  code_.shift(Shift::right, Reg::x0, Reg::x0, word_bits, Width::x64);
  mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  code_.store(Reg::x0, Reg::sp, Frame::value);
}

void Emitter::long_commit(const unsigned selector) {
  if (selector >= 6) {
    code_.load(Reg::x0, Reg::sp, Frame::value);
    write_data(selector == 6 ? a_register : b_register);
    code_.load(Reg::x0, Reg::sp, Frame::other_value);
    write_data(selector == 6 ? b_register : a_register);
    return;
  }
  code_.load(Reg::x0, Reg::sp, Frame::other_value);
  code_.load(Reg::x1, Reg::sp, Frame::value);
  code_.shift(Shift::left, Reg::x1, Reg::x1, word_bits, Width::x64);
  code_.alu(Alu::bit_or, Reg::x0, Reg::x0, Reg::x1, Width::x64);
  if (selector >= 4) {
    code_.extract(Reg::x0, Reg::x0, 0, double_word_bits, Width::x64, true);
    code_.shift(Shift::left,
                (selector & 1U) == 0 ? accumulator_a : accumulator_b,
                Reg::x0,
                padding_bits,
                Width::x64);
  } else if (selector >= 2) {
    code_.move(selector == 2 ? bank_x : bank_y, Reg::x0);
  } else {
    const auto target = selector == 0 ? accumulator_a : accumulator_b;
    mask(target, target, ~(long_mask << padding_bits));
    code_.shift(Shift::left, Reg::x0, Reg::x0, padding_bits, Width::x64);
    code_.alu(Alu::bit_or, target, target, Reg::x0, Width::x64);
  }
}

void Emitter::long_move(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto selector = ((opcode >> 16) & 3U) | ((opcode >> 17) & 4U);
  const bool extended = (opcode & (1U << 14)) != 0;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = instruction.extension};
  if (extended && form.mode == 6 && form.index != 0) {
    code_.constant(Reg::x0, form.index);
    fault(operation, FaultKind::invalid_instruction, Reg::x0);
    return;
  }
  if (extended) {
    effective(operation, form);
  } else {
    code_.constant(Reg::x0, (opcode >> 8) & six_bit_mask);
  }
  code_.store(Reg::x0, Reg::sp, Frame::address);
  const bool load = (opcode & (1U << 15)) != 0;
  if (load) {
    memory(operation, {.space = Space::x});
    code_.store(Reg::x0, Reg::sp, Frame::value);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    memory(operation, {.space = Space::y});
    code_.store(Reg::x0, Reg::sp, Frame::other_value);
    long_commit(selector);
  } else {
    long_capture(operation, selector);
    code_.load(Reg::x0, Reg::sp, Frame::address);
    code_.load(Reg::x1, Reg::sp, Frame::value);
    memory(operation, {.space = Space::x, .write = true});
    code_.load(Reg::x0, Reg::sp, Frame::address);
    code_.load(Reg::x1, Reg::sp, Frame::other_value);
    memory(operation, {.space = Space::y, .write = true});
  }
}
}
