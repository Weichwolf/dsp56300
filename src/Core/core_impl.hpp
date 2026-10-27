// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace dsp56300 {
struct Core::Impl {
  explicit Impl(Buffers supplied, Io *io, Options settings);
  [[nodiscard]] std::span<Word> area(Space space) const;
  void fail(FaultKind kind, Space space, Word address, Word value = 0);
  [[nodiscard]] Word read_memory(Space space, Word address);
  void write_memory(Space space, Word address, Word value);
  [[nodiscard]] detail::Instruction instruction(Word pc);
  void step();
  void step_decoded(const detail::Instruction &decoded);
  [[nodiscard]] std::uint32_t run_general(std::uint32_t budget);
  [[nodiscard]] static std::uint32_t jit_execute(void *owner,
                                                 const detail::NativeOp *operation) noexcept;
  [[nodiscard]] static std::uint32_t jit_moves(void *owner,
                                               const detail::NativeOp *operation) noexcept;
  void execute(const detail::Instruction &decoded);
  void execute_immediate(const detail::Instruction &decoded);
  void execute_jump(const detail::Instruction &decoded);
  void execute_branch(const detail::Instruction &decoded);
  void execute_bit(const detail::Instruction &decoded);
  [[nodiscard]] bool access_bit(const detail::Instruction &decoded, BitAction action);
  [[nodiscard]] bool register_bit(const detail::BitAccess &access);
  [[nodiscard]] Word bit_address(const detail::BitOperand &operand,
                                 const detail::Instruction &decoded);
  void conditional_transfer(Word opcode);
  void conditional_alu(Word opcode);
  [[nodiscard]] Word control_address(const detail::Instruction &decoded);
  void parallel(const detail::Instruction &decoded);
  void moves(const detail::Instruction &decoded);
  void register_move(Word opcode);
  void paired_move(Word opcode);
  void memory_move(const detail::Instruction &decoded);
  void long_move(const detail::Instruction &decoded);
  void mixed_move(const detail::Instruction &decoded);
  void class_two_move(const detail::Instruction &decoded);
  void control_move(const detail::Instruction &decoded);
  void control_register_move(Word opcode);
  void program_move(const detail::Instruction &decoded);
  void peripheral_move(const detail::Instruction &decoded);
  void peripheral_register_move(const detail::Instruction &decoded);
  void alu(Word opcode);
  void multiply(Word opcode);
  [[nodiscard]] bool unary(Word opcode);
  [[nodiscard]] bool logic(Word opcode);
  [[nodiscard]] bool addition(Word opcode);
  [[nodiscard]] bool shifted_addition(Word opcode);
  void add_operands(detail::AddOperands input);
  [[nodiscard]] bool word_shift(Word opcode);
  [[nodiscard]] bool numeric_mode_supported();
  void execute_shift(const detail::Instruction &decoded);
  void arithmetic_shift(std::uint64_t value, detail::ShiftControl control);
  void logical_shift(detail::ShiftControl control, bool rotate = false);
  [[nodiscard]] Word read_register(unsigned number);
  void write_register(unsigned number, Word value);
  [[nodiscard]] Word effective(detail::AddressForm form);
  [[nodiscard]] Word update_address(detail::AddressUpdate update);
  [[nodiscard]] Word transfer_word(std::uint64_t value);
  [[nodiscard]] std::uint64_t transfer_long(std::uint64_t value);
  [[nodiscard]] std::uint64_t source(unsigned selector, bool destination_b) const;
  void arithmetic_flags(std::uint64_t value, bool overflowed);
  void logical_flags(Word value);
  void round(std::uint64_t &value);
  void push(Word high, Word low);
  [[nodiscard]] std::uint64_t pop();
  void finish_loop();
  detail::Cpu cpu;
  Io *io;
  std::array<std::uint64_t, 4> latched{};
  detail::Instruction repeated;
  std::unique_ptr<detail::Jit> jit;
  Buffers buffers;
  static constexpr std::size_t page_words = 1024;
  static constexpr std::size_t page_count = address_words / page_words;
  using Page = std::array<detail::Instruction, page_words>;
  std::array<std::unique_ptr<Page>, page_count> pages{};
  Word current_pc = 0;
  unsigned current_words = 1;
  Options options;
  Fault fault;
  bool executing = false;
};
} // namespace dsp56300
