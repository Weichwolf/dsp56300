// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"

#include <cstddef>
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

struct Frame {
  static constexpr std::size_t address = 40, other_address = 44, value = 48, other_value = 56,
                               secondary = 64, access = 80;
  static constexpr std::size_t pairs = (access + sizeof(NativeAccess) + 15) / 16;
};

struct BusOperand {
  Space space = Space::x;
  bool write = false, bit = false;
};

struct MemoryLabels {
  Label &mapped;
  Label &unmapped;
};

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
  [[nodiscard]] bool numeric_mode(const BlockInstruction &operation, Word ignored_modes);
  [[nodiscard]] static bool supported_alu(Word opcode);
  void alu(const BlockInstruction &operation);
  void conditional_alu(const BlockInstruction &operation);
  void conditional_transfer(const BlockInstruction &operation);
  [[nodiscard]] bool transfer(const BlockInstruction &operation);
  void moves(const BlockInstruction &operation);
  void register_move(const BlockInstruction &operation);
  void memory_move(const BlockInstruction &operation);
  void class_two_move(const BlockInstruction &operation);
  void long_move(const BlockInstruction &operation);
  void long_capture(const BlockInstruction &operation, unsigned selector);
  void long_commit(unsigned selector);
  void transfer_long(Reg value, Word mode);
  void mixed_move(const BlockInstruction &operation);
  void paired_move(const BlockInstruction &operation);
  void paired_address(const BlockInstruction &operation, AddressForm form, std::size_t offset);
  void paired_capture(const BlockInstruction &operation, unsigned reg, BusOperand operand);
  void paired_commit(const BlockInstruction &operation, unsigned reg, BusOperand operand);
  void memory(const BlockInstruction &operation, BusOperand operand);
  void memory_windows(BusOperand operand, MemoryLabels labels);
  void memory_flat(BusOperand operand, MemoryLabels labels);
  void memory_owned(const BlockInstruction &operation, BusOperand operand);
  void owned_access(const BlockInstruction &operation, BusOperand operand, BusAction action);
  void memory_fault(const BlockInstruction &operation, BusOperand operand);
  void bus_failure();
  void mark_program_written();
  void memory_transfer(const BlockInstruction &operation);
  void displacement_move(const BlockInstruction &operation);
  void peripheral_move(const BlockInstruction &operation);
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
  [[nodiscard]] bool product(const BlockInstruction &operation);
  void ordinary_product(const BlockInstruction &operation);
  void mixed_product(const BlockInstruction &operation);
  void product_complete(const BlockInstruction &operation);
  [[nodiscard]] bool bit_field(const BlockInstruction &operation);
  [[nodiscard]] bool field_control(const BlockInstruction &operation);
  void field_bounds(const BlockInstruction &operation, bool insert);
  void extract_field(const BlockInstruction &operation);
  void insert_field(const BlockInstruction &operation);
  void merge_field(const BlockInstruction &operation);
  void count_leading_bits(const BlockInstruction &operation);
  void viterbi_shift(const BlockInstruction &operation);
  [[nodiscard]] bool bits(const BlockInstruction &operation);
  void register_bit(const BlockInstruction &operation, BitAccess access);
  void memory_bit(const BlockInstruction &operation, BitAccess access);
  [[nodiscard]] bool bit_address(const BlockInstruction &operation, BitOperand operand);
  void modify_bit(BitAction action, unsigned bit);
  void bit_branch(const BlockInstruction &operation);
  [[nodiscard]] bool integer(const BlockInstruction &operation);
  void unit(const BlockInstruction &operation);
  void compare_unsigned(const BlockInstruction &operation);
  void divide_iteration(const BlockInstruction &operation);
  void normalize_iteration(const BlockInstruction &operation);
  void normalize_fast(const BlockInstruction &operation);
  void normalize_step(const BlockInstruction &operation, int count);
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
  Label *failure_ = nullptr;
};
}
