// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dataflow.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/resident.hpp"
#include "move.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
enum class ExecutionBounds : unsigned char { whole, precise };

class BlockEmitter {
public:
  BlockEmitter(JitRuntime runtime,
               std::array<Word, agu_registers> modifiers = {},
               RegionMode region = RegionMode::automatic,
               LoopResidency residency = LoopResidency::automatic,
               RangeMode ranges = RangeMode::automatic,
               WordReuseMode words = WordReuseMode::automatic,
               LoopWindowMode window = LoopWindowMode::automatic);
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const BlockInstruction> operations,
                                                   ExecutionBounds bounds = ExecutionBounds::whole);

  [[nodiscard]] std::optional<std::vector<std::uint8_t>>
  generate_guarded(std::span<const BlockInstruction> operations,
                   std::span<const std::uint8_t> checked);

  [[nodiscard]] std::vector<std::uint8_t> generate_repeat(const BlockInstruction &operation);

  [[nodiscard]] static bool closed_jump(std::span<const BlockInstruction> operations);

  [[nodiscard]] bool direct_memory(const BlockInstruction &operation) const;

  [[nodiscard]] bool packed_retirement() const noexcept { return packed_retirement_; }

  [[nodiscard]] bool cached_status() const noexcept { return cached_status_; }

  [[nodiscard]] bool cached_inputs() const noexcept { return cached_inputs_; }

  [[nodiscard]] bool mac_batches() const noexcept { return mac_batches_; }

  [[nodiscard]] bool guarded_region() const noexcept { return region_.valid; }

  [[nodiscard]] bool coalesced_accounting() const noexcept { return coalesced_accounting_; }

  [[nodiscard]] bool loop_residency() const noexcept { return loop_residency_; }

  [[nodiscard]] bool word_reuse() const noexcept { return reused_words_; }

  [[nodiscard]] bool loop_window() const noexcept { return loop_window_; }

private:
  struct WindowReuse {
    Word capture = 0, restore = 0;
  };

  [[nodiscard]] std::vector<WindowReuse>
  plan_window_reuse(std::span<const BlockInstruction> operations) const;
  [[nodiscard]] static bool lowerable(const BlockInstruction &operation);
  [[nodiscard]] static bool saturation_supported(const BlockInstruction &operation);
  [[nodiscard]] static bool has_moves(const BlockInstruction &operation);
  [[nodiscard]] bool observable_moves(const BlockInstruction &operation) const;
  [[nodiscard]] std::vector<std::uint8_t>
  generate_body(std::span<const BlockInstruction> operations);
  [[nodiscard]] std::vector<std::uint8_t>
  generate_selected(std::span<const BlockInstruction> operations);
  void emit_nops(std::span<const BlockInstruction> operations, bool publish);
  void emit_operations(std::span<const BlockInstruction> operations);
  void budget_guard();
  void region_guard(x64::Code &code, x64::Reg cpu, x64::Label &failed) const;
  void region_address(const NativeMove &move);
  [[nodiscard]] static bool is_addition(const BlockInstruction &operation);
  [[nodiscard]] static bool is_arithmetic_shift(const BlockInstruction &operation);
  void helper(JitHelper function, const BlockInstruction &operation);
  void native(const BlockInstruction &operation);
  [[nodiscard]] static bool is_control_flow(const BlockInstruction &operation);
  [[nodiscard]] bool boundary_operation(const BlockInstruction &operation, bool publish);
  void control_condition(unsigned selector, x64::Label &untaken);
  void control_pop();
  [[nodiscard]] static bool is_bits(const BlockInstruction &operation);
  [[nodiscard]] bool bits(const BlockInstruction &operation, bool publish);
  void bit_result(const BlockInstruction &operation, NativeMove move, bool publish);
  void bit_complete(const BlockInstruction &operation, bool publish);
  void instruction_fallback(const BlockInstruction &operation);
  [[nodiscard]] static bool is_peripheral_register(const BlockInstruction &operation);
  void peripheral_register(const BlockInstruction &operation);
  void moves(const BlockInstruction &operation);
  void general_moves(const BlockInstruction &operation);
  void capture_inputs();
  void restore_inputs();
  [[nodiscard]] static bool admit_cached_inputs(std::span<const BlockInstruction> operations);
  void register_move(const NativeRegisterMove &move, Word mode);
  void move_guard(const NativeMove &move, x64::Label &slow);
  void move_effective(const NativeMove &move);
  void move_memory_guard(const NativeMove &move, x64::Label &slow);
  void long_guards(const NativeMoves &moves, x64::Label &slow);
  void long_capture(const NativeMoves &moves, Word mode);
  void long_commit(const NativeMoves &moves);
  void move_guards(const NativeMoves &moves, x64::Label &slow);
  void move_values(const NativeMoves &moves, Word mode);
  void move_next(const NativeMove &move);
  void window_guard(const NativeMove &move, x64::Label &slow);
  void window_candidate(const NativeMove &move, const MemoryWindow &window, x64::Label &next);
  void window_known(const NativeMove &move, const MemoryWindow &window, x64::Label &slow);
  void capture_window(const NativeMove &move, const MemoryWindow &window);
  void mark_window_written(const NativeMove &move);
  void move_profiled_next(const NativeMove &move, Word modifier);
  void move_delta(const NativeMove &move);
  void address_update(const BlockInstruction &operation);
  void address_immediate(const NativeAddressImmediate &move);
  void move_ring_delta(const NativeMove &move);
  void emit_ring_guards();
  void emit_ring_check(x64::Label &entry);
  void move_update(const NativeMove &move);
  void move_capture(const NativeMove &move, Word mode);
  void move_commit(const NativeMove &move);
  void move_read_register(x64::Reg value, const NativeMove &move, Word mode);
  void move_write_register(x64::Reg value, unsigned reg);
  void multiply(const BlockInstruction &operation);
  [[nodiscard]] bool gpr_product(const BlockInstruction &operation, x64::Reg result);
  void invalidate_word_reuse(const WordReuseStep &step);
  void product_word(x64::Reg destination,
                    const BlockInstruction &operation,
                    unsigned selector,
                    bool scaled);
  [[nodiscard]] std::vector<WordReuseStep>
  word_effects(std::span<const BlockInstruction> operations) const;
  [[nodiscard]] static bool is_logic(const BlockInstruction &operation);
  void logic(const BlockInstruction &operation);
  void logical_flags(x64::Reg word);
  void addition(const BlockInstruction &operation);
  void arithmetic_shift(const BlockInstruction &operation);
  void immediate_shift(const BlockInstruction &operation);
  void left_shift(const BlockInstruction &operation);
  void round(Word mode, x64::Reg result = x64::Reg::ax);
  void saturate(Word mode, bool rounded = false);
  void saturate_canonical(bool rounded);
  void arithmetic_flags(Word mode, bool carry_changed = false, x64::Reg result = x64::Reg::ax);
  void merge_flag(x64::Reg value, bool &initialized);
  void zero_negative_flags(bool &initialized);
  void normalization_flags(Word mode, bool &initialized);
  void prepare_value_proof(const BlockInstruction &operation, NativeValueRanges &ranges);
  [[nodiscard]] bool admit_cached_status() const;
  void initialize_status();
  void materialize_status();
  void read_status(x64::Reg destination);
  void merge_status(x64::Reg source, Word changed = 0, x64::Reg scratch = x64::Reg::ax);
  [[nodiscard]] std::vector<Word> plan_flags(std::span<const BlockInstruction> operations) const;
  [[nodiscard]] static Word written_flags(const BlockInstruction &operation);
  [[nodiscard]] x64::Reg result_register(const BlockInstruction &operation) const;
  void input(x64::Reg destination, unsigned selector, bool destination_b, bool latched);
  void value(x64::Reg destination, unsigned index, bool latched);
  void current_value(x64::Reg destination, unsigned index);
  void store_accumulator(bool destination_b, x64::Reg source);
  [[nodiscard]] Word dirty_accumulators() const;
  void flush_accumulators(Word mask);
  void exit_if(x64::Branch condition);
  void finish();
  [[nodiscard]] bool admit_packed_retirement(std::span<const BlockInstruction> operations) const;
  void initialize_retirement();
  void materialize_retirement();
  void materialize_pending_accounting();
  void retire_progress(const BlockInstruction &operation);
  [[nodiscard]] bool publish_pc(std::span<const BlockInstruction> operations,
                                std::size_t index) const;
  void loop_guard(const BlockInstruction &operation);
  [[nodiscard]] static bool is_loop_setup(const BlockInstruction &operation);
  void loop_setup(const BlockInstruction &operation);
  void loop_push(x64::Reg frame);
  void loop_continue();
  void loop_backedge();
  void loop_window_backedge();
  void retire_loop_window(const BlockInstruction &operation, bool publish);
  void emit_loop_copy(std::span<const BlockInstruction> operations);
  void initialize_loop_residency();
  void initialize_loop_window();
  void initialize_loop_window_count();
  void plan_loop_window(std::span<const BlockInstruction> operations);
  void guard_region_header();
  void batch_entry(std::span<const BlockInstruction> operations);
  void retire(const BlockInstruction &operation, bool publish);
  void repeat_retire(const BlockInstruction &operation);
  x64::Code code_;

  struct RingGuard {
    x64::Label entry, resume, slow;
    NativeMove move;
    bool profiled = false;
  };

  std::vector<RingGuard> ring_guards_;
  JitRuntime runtime_;
  std::array<Word, agu_registers> modifiers_;
  RegionMode region_mode_;
  LoopResidency residency_mode_;
  RangeMode range_mode_;
  WordReuseMode word_mode_;
  LoopWindowMode window_mode_;
  bool loop_window_ = false;
  Word window_capture_ = 0;
  Word window_restore_ = 0;
  Word word_valid_ = 0, word_populate_ = 0, word_selected_ = 0;
  bool reused_words_ = false;
  NativeValueProof value_proof_;
  ResidentPlan region_;
  bool repeat_mode_ = false;
  bool mac_batches_ = false;
  bool packed_retirement_ = false;
  bool retirement_helpers_ = false;
  bool cached_status_ = false;
  bool cached_inputs_ = false;
  bool coalesced_accounting_ = false;
  bool coalesced_loop_ = false;
  bool copy_loop_ = false;
  bool loop_residency_ = false;
  bool entry_admitted_ = false;
  unsigned deferred_growth_shift_ = 0;
  Word pending_instructions_ = 0, pending_extra_cycles_ = 0;
  NativeDataflow dataflow_;
  // Dirty implies valid. Closed resident loops seed both slots before all effects.
  std::array<bool, 2> accumulator_dirty_{};
  std::array<x64::Label, 4> exits_;
  x64::Label loop_header_;
  x64::Label loop_body_;
  x64::Label window_exhausted_;
  Word loop_size_ = 0, loop_pc_ = 0;
  ExecutionBounds bounds_ = ExecutionBounds::whole;
  Word live_flags_ = carry | overflow | zero | negative | unnormalized | extension;
};
} // namespace dsp56300::detail
