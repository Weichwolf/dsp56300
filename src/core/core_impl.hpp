// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "program_cache.hpp"
#include "staged.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300 {
struct Core::Impl {
  using VectorPhase = detail::VectorPhase;
  using Vector = detail::VectorState;

  [[nodiscard]] bool exception_ready() const noexcept {
    return vector.phase != VectorPhase::none && cpu.repeat_remaining == 0;
  }

  [[nodiscard]] bool debug_ready() const noexcept {
    return (cpu.debug_flags & detail::debug_interrupt_pending) != 0 && cpu.repeat_remaining == 0 &&
           vector.phase == VectorPhase::none;
  }

  void request_debug(bool software);
  void execute_debug(const detail::Instruction &decoded);
  void admit_debug();

  [[nodiscard]] bool cacheable(const Word address) const noexcept {
    return (cpu.reg.sr & detail::cache_enable) != 0 && address >= options.cacheable_begin &&
           address < options.cacheable_end;
  }

  [[nodiscard]] detail::ProgramCache &program_cache();

  void synchronize_cache_mode() {
    if ((cpu.reg.sr & detail::cache_enable) == 0) { icache.reset(); }
  }

  [[nodiscard]] Word fetch_program(Word address);
  [[nodiscard]] Word read_external(Space space, Word address);
  void write_external(Space space, Word address, Word value);
  void execute_processing(const detail::Instruction &decoded);
  void execute_cache(const detail::Instruction &decoded);
  void execute_exception(const detail::Instruction &decoded);
  void step_vector();
  [[nodiscard]] detail::Instruction vector_instruction();
  void call_vector(const detail::Instruction &decoded);

  explicit Impl(Buffers supplied, Io *io, Options settings, bool external_memory = false);
  [[nodiscard]] detail::ExecutionState execution_state() const;
  void restore_execution(const detail::ExecutionState &state);

  [[nodiscard]] bool staged_active() const noexcept { return staged && staged->active; }

  [[nodiscard]] bool staged_computing() const noexcept { return staged && staged->computing; }

  void compute_instruction();
  [[nodiscard]] std::exception_ptr compute_attempt();
  void resume_instruction(const InstructionEffect &effect);
  [[nodiscard]] bool plan_instruction() const;
  void finish_computation(const std::exception_ptr &error);
  void apply_publication(const InstructionEffect &effect);
  void capture_program_words();
  [[nodiscard]] bool pcu_publication() const noexcept;
  void mark_pcu_field(RegisterUpdate update) const;
  void mark_control_transfer() const;
  void publish_pcu() const;
  void apply_pcu(const PcuWriteback &update);
  [[nodiscard]] Word read_pcu_register(unsigned number);
  void mark_hidden(detail::RetirementField field) const;
  void mark_debug(std::uint8_t mask) const;
  void mark_vector() const;
  void retire_instruction(unsigned cycles);
  void retire_owned();
  [[nodiscard]] bool unit_publication() const noexcept;
  void begin_unit(RegisterUnit unit) const;
  void mark_unit_field(RegisterUpdate update) const;
  [[nodiscard]] Word read_unit_register(unsigned number);
  void write_unit_register(unsigned number, Word value);
  void publish_unit() const;
  void apply_unit(const UnitWriteback &update);
  [[nodiscard]] bool alu_publication() const noexcept;
  void mark_alu(RegisterStorage storage, std::uint64_t mask) const;
  void mark_alu_field(RegisterUpdate update) const;
  void mark_alu_address(unsigned index) const;
  void mark_alu_accumulator(bool destination_b, std::uint64_t mask = accumulator_mask) const;
  void omit_alu_status(Word mask) const;
  void publish_alu() const;
  void apply_alu(const AluWriteback &update);
  void publish_address_update(AddressWriteback update, Word address) const;
  void restore_published_results();
  [[nodiscard]] bool move_publication() const noexcept;
  [[nodiscard]] MoveWriteback move_destination(unsigned number) const;
  [[nodiscard]] static MoveWriteback long_destination(unsigned selector);
  void capture_move(MoveWriteback &update) const;
  void publish_move(MoveWriteback update) const;
  void apply_move(const MoveWriteback &update);
  [[nodiscard]] Word read_move_register(unsigned number);
  void write_move_register(unsigned number, Word value);
  void transfer_status(Word flags, std::optional<unsigned> source_register);
  [[nodiscard]] detail::Instruction captured_instruction(Word address) const;
  [[nodiscard]] Word sample_instruction(InstructionEffect effect) const;
  [[nodiscard]] bool perform_instruction(InstructionEffect effect, Word &response) const;
  void validate_staged_mutation() const;
  void invalidate_effect_tickets();
  [[nodiscard]] std::span<Word> area(Space space) const;
  void fail(FaultKind kind, Space space, Word address, Word value = 0);
  [[nodiscard]] Word read_memory(Space space, Word address);
  void write_memory(Space space, Word address, Word value);
  [[nodiscard]] bool overlaps_program(std::span<const Word> memory) const;
  void invalidate_program_word(const Word *location);
  [[nodiscard]] detail::Instruction instruction(Word pc);
  void step();
  void step_decoded(const detail::Instruction &decoded);
  [[nodiscard]] std::uint32_t run_general(std::uint32_t budget);
  [[nodiscard]] std::optional<detail::Instruction> budget_instruction() const;
  [[nodiscard]] std::optional<Word> budget_word(Word address) const;
  [[nodiscard]] BoundedRunResult run_bounded(RunBudget budget);
  [[nodiscard]] std::optional<RunStop> budget_stop(RunBudget budget,
                                                   const BoundedRunResult &result) const;
  [[nodiscard]] static std::uint32_t jit_execute(void *owner,
                                                 const detail::NativeOp *operation) noexcept;
  void synchronize_program_view();
  [[nodiscard]] static std::optional<Word> jit_program(void *owner, Word address) noexcept;
  [[nodiscard]] static std::uint32_t jit_moves(void *owner,
                                               const detail::NativeOp *operation) noexcept;
  void execute(const detail::Instruction &decoded);
  void execute_operation(const detail::Instruction &decoded);
  void execute_return(const detail::Instruction &decoded);
  void execute_immediate(const detail::Instruction &decoded);
  void execute_iteration(const detail::Instruction &decoded);
  void normalize_fast(Word opcode);
  void viterbi_shift(const detail::Instruction &decoded);
  void execute_address_calculation(const detail::Instruction &decoded);
  void execute_product(const detail::Instruction &decoded);
  void execute_unit(const detail::Instruction &decoded);
  void compare_unsigned(Word opcode);
  void execute_bit_field(const detail::Instruction &decoded);
  [[nodiscard]] Word bit_field_control(const detail::Instruction &decoded);
  void extract_field(const detail::Instruction &decoded, Word control);
  void insert_field(const detail::Instruction &decoded, Word control);
  void merge_field(const detail::Instruction &decoded, Word source);
  void count_leading_bits(Word opcode);
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
  void displacement_move(const detail::Instruction &decoded);
  void load_long(const detail::Instruction &decoded, std::uint64_t value);
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
  [[nodiscard]] bool numeric_mode_supported(Word ignored_modes = 0);
  [[nodiscard]] bool numeric_alu_supported(Word opcode);
  bool limit_arithmetic(std::uint64_t &value, bool rounded = false) const;
  void saturate_arithmetic(std::uint64_t &value, bool rounded = false);
  void execute_shift(const detail::Instruction &decoded);
  void arithmetic_shift(std::uint64_t value, detail::ShiftControl control);
  void logical_shift(detail::ShiftControl control, bool rotate = false);
  [[nodiscard]] Word read_register(unsigned number,
                                   detail::StackOwner owner = detail::StackOwner::operand);
  void write_register(unsigned number, Word value);
  [[nodiscard]] Word effective(detail::AddressForm form);
  [[nodiscard]] Word update_address(detail::AddressUpdate update);
  [[nodiscard]] Word transfer_word(std::uint64_t value,
                                   std::optional<unsigned> source_register = std::nullopt);
  [[nodiscard]] std::uint64_t transfer_long(std::uint64_t value,
                                            std::optional<unsigned> source_register = std::nullopt);
  [[nodiscard]] std::uint64_t source(unsigned selector, bool destination_b) const;
  void arithmetic_flags(std::uint64_t value, bool overflowed);
  void logical_flags(Word value);
  void round(std::uint64_t &value);
  void push(Word high, Word low, detail::StackOwner owner = detail::StackOwner::operand);
  [[nodiscard]] std::uint64_t pop(detail::StackOwner owner = detail::StackOwner::operand);
  void finish_loop();
  void restore_loop();
  void execute_loop(const detail::Instruction &decoded);
  [[nodiscard]] Word loop_count(const detail::Instruction &decoded);
  detail::Cpu cpu;
  Io *io;
  std::unique_ptr<detail::Jit> jit;
  std::unique_ptr<detail::ProgramCache> icache;
  std::array<std::uint64_t, 4> latched{};
  Buffers buffers;
  static constexpr std::size_t page_words = 1024;
  static constexpr std::size_t page_count = address_words / page_words;
  using Page = std::array<detail::Instruction, page_words>;
  std::array<std::unique_ptr<Page>, page_count> pages{};
  Word current_pc = 0;
  unsigned current_words = 1;
  detail::Instruction repeated;
  Fault fault;
  std::optional<InstructionOutcome> last_outcome;
  Vector vector;
  Options options;
  bool executing = false;
  std::array<bool, 3> aliases_program{true, false, false};
  Word debug_vector = 0;
  bool external = false;
  bool yield_requested = false;
  std::uint64_t program_revision = 0;
  Word program_omr = 0;
  std::unique_ptr<detail::StagedState> staged;
  std::shared_ptr<const bool> effect_owner;
  std::uint64_t effect_epoch = 0;
};
} // namespace dsp56300
