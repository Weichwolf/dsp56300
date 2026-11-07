// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail::a64 {
inline constexpr Reg cpu = Reg::x19, context = Reg::x20, accumulator_a = Reg::x21,
                     accumulator_b = Reg::x22, bank_x = Reg::x23, bank_y = Reg::x24,
                     status = Reg::x25, pc = Reg::x26, retired = Reg::x27, remaining = Reg::x28;

enum AluByte : std::uint8_t {
  alu_transfer = 1,
  alu_test = 3,
  alu_round = 0x11,
  alu_clear = 0x13,
  alu_complement = 0x17,
  alu_absolute = 0x26,
  alu_negate = 0x36,
  alu_x0 = 0x41,
  alu_y0 = 0x51,
  alu_x1 = 0x61,
  alu_y1 = 0x71,
  alu_asr = 0x22,
  alu_asl = 0x32,
  alu_lsr = 0x23,
  alu_lsl = 0x33,
  alu_ror = 0x27,
  alu_rol = 0x37,
  alu_add_right = 0x02,
  alu_subtract_right = 0x06,
  alu_add_left = 0x12,
  alu_subtract_left = 0x16,
  alu_maximum = 0x1d,
  alu_maximum_magnitude = 0x15,
  alu_mask = 0xf7
};

struct ShiftOperation {
  Reg target;
  bool left = false, saturate = false;
};

enum class Flow : std::uint8_t { sequential, control, context, owner };

class Emitter {
public:
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const BlockInstruction> operations);

private:
  void begin();
  void snapshot();
  void publish();
  void retire(const BlockInstruction &operation);
  void operation(const BlockInstruction &operation);
  [[nodiscard]] bool numeric_mode(const BlockInstruction &operation, bool saturating);
  [[nodiscard]] static bool supported_alu(Word opcode);
  void alu(const BlockInstruction &operation);
  void conditional_alu(const BlockInstruction &operation);
  void conditional_transfer(const BlockInstruction &operation);
  [[nodiscard]] bool transfer(const BlockInstruction &operation);
  void moves(const BlockInstruction &operation);
  void register_move(const BlockInstruction &operation);
  void control_move(const BlockInstruction &operation);
  void address_calculation(const BlockInstruction &operation);
  void calculation_offset(Word opcode);
  [[nodiscard]] static bool valid_register(unsigned number);
  void read_register(const BlockInstruction &operation, unsigned number);
  void write_register(const BlockInstruction &operation, unsigned number);
  void read_data(unsigned number, Word mode);
  void write_data(unsigned number);
  void transfer_word(Reg value, Word mode);
  void stack_slot(bool next);
  [[nodiscard]] bool control(const BlockInstruction &operation);
  void condition(Reg result, unsigned selector);
  void boundary(NativeBoundary state);
  void control_byte(const BlockInstruction &operation);
  void jump(const BlockInstruction &operation);
  void branch(const BlockInstruction &operation);
  void return_(const BlockInstruction &operation);
  void push_frame(const BlockInstruction &operation);
  void pop_frame(const BlockInstruction &operation);
  void effective(const BlockInstruction &operation, AddressForm form);
  void update_address(const BlockInstruction &operation);
  void reverse_address();
  void ring_address(Label &linear);
  void multiple_address(const BlockInstruction &operation);
  void source(Reg value, unsigned selector, bool destination_b);
  void product_word(Reg value, unsigned selector);
  void arithmetic(const BlockInstruction &operation);
  void shifted_arithmetic(const BlockInstruction &operation);
  void compare_magnitude(const BlockInstruction &operation);
  void maximum(Word opcode);
  void magnitude(Reg value, Reg temporary);
  void multiply(const BlockInstruction &operation);
  void unary(const BlockInstruction &operation);
  void logic(const BlockInstruction &operation);
  void shift(const BlockInstruction &operation);
  void shift_count(const BlockInstruction &operation, bool arithmetic);
  void arithmetic_shift(Word mode, ShiftOperation operation);
  void logical_shift(ShiftOperation operation, bool rotate);
  void fault(const BlockInstruction &operation, FaultKind kind, Reg value);
  void round(Word mode);
  void saturate(Word mode, bool rounded);
  void arithmetic_flags(Word mode, bool carry_changed);
  void logical_flags(Reg value);
  void
  complete(const BlockInstruction &operation, bool compare, bool carried, bool rounded = false);
  void mask(Reg target, Reg value, std::uint64_t bits, Width width = Width::x64);
  [[nodiscard]] static Reg destination(Word opcode);
  Code code_;
  Label done_;
  Flow flow_ = Flow::sequential;
};
}
