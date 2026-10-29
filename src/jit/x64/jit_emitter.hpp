// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dataflow.hpp"
#include "jit.hpp"
#include "native_moves.hpp"
#include "native_region.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
class JitEmitter {
public:
  JitEmitter(JitRuntime runtime,
             bool optimize,
             BatchBackend batch = BatchBackend::automatic,
             std::array<Word, agu_registers> modifiers = {},
             RegionMode region = RegionMode::automatic,
             LoopResidency residency = LoopResidency::automatic,
             RangeMode ranges = RangeMode::automatic,
             WordReuseMode words = WordReuseMode::automatic,
             LoopWindowMode window = LoopWindowMode::automatic);
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const NativeOp> operations);

  [[nodiscard]] bool packed_retirement() const noexcept { return packed_retirement_; }

  [[nodiscard]] bool cached_status() const noexcept { return cached_status_; }

  [[nodiscard]] bool cached_inputs() const noexcept { return cached_inputs_; }

  [[nodiscard]] bool mac_batches() const noexcept { return mac_batches_; }

  [[nodiscard]] bool guarded_region() const noexcept { return region_.valid; }

  [[nodiscard]] bool coalesced_accounting() const noexcept { return coalesced_accounting_; }

  [[nodiscard]] bool loop_residency() const noexcept { return loop_residency_; }

  [[nodiscard]] bool loop_bank_cache() const noexcept { return loop_bank_cache_; }

  [[nodiscard]] bool word_reuse() const noexcept { return reused_words_; }

  [[nodiscard]] bool loop_window() const noexcept { return loop_window_; }

private:
  [[nodiscard]] bool lowerable(const NativeOp &operation) const;
  [[nodiscard]] static bool saturation_supported(const NativeOp &operation);
  [[nodiscard]] static bool has_moves(const NativeOp &operation);
  [[nodiscard]] bool observable_moves(const NativeOp &operation) const;
  [[nodiscard]] std::vector<std::uint8_t> generate_body(std::span<const NativeOp> operations);
  [[nodiscard]] std::vector<std::uint8_t> generate_selected(std::span<const NativeOp> operations);
  void region_guard(x64::Code &code, x64::Reg cpu, x64::Label &failed) const;
  void region_address(const NativeMove &move);
  [[nodiscard]] static bool is_addition(const NativeOp &operation);
  [[nodiscard]] bool is_right_shift(const NativeOp &operation) const;
  void helper(JitHelper function, const NativeOp &operation);
  void native(const NativeOp &operation);
  void moves(const NativeOp &operation);
  void general_moves(const NativeOp &operation);
  void capture_inputs();
  void restore_inputs();
  [[nodiscard]] x64::Xmm input_snapshot(unsigned index) const;
  void materialize_loop_banks();
  [[nodiscard]] bool admit_cached_inputs(std::span<const NativeOp> operations) const;
  void register_move(const NativeRegisterMove &move, Word mode);
  void move_guard(const NativeMove &move, x64::Label &slow);
  void move_next(const NativeMove &move);
  void move_profiled_next(const NativeMove &move, Word modifier);
  void move_delta(const NativeMove &move);
  void address_update(const NativeOp &operation);
  void address_immediate(const NativeAddressImmediate &move);
  void move_ring_delta(const NativeMove &move);
  void emit_ring_guards();
  void emit_ring_check(x64::Label &entry);
  void move_update(const NativeMove &move);
  void move_capture(const NativeMove &move, Word mode);
  void move_commit(const NativeMove &move);
  void move_read_register(x64::Reg value, const NativeMove &move, Word mode);
  void move_write_register(x64::Reg value, unsigned reg);
  void multiply(const NativeOp &operation);
  [[nodiscard]] bool gpr_product(const NativeOp &operation);
  void invalidate_word_reuse(const WordReuseStep &step);
  void
  product_word(x64::Reg destination, const NativeOp &operation, unsigned selector, bool scaled);
  [[nodiscard]] std::vector<WordReuseStep> word_effects(std::span<const NativeOp> operations) const;
  void addition(const NativeOp &operation);
  void right_shift(const NativeOp &operation);
  void round(Word mode);
  void saturate(Word mode, bool rounded = false);
  void saturate_canonical(bool rounded);
  void arithmetic_flags(Word mode, bool carry_changed = false);
  void merge_flag(x64::Reg value, bool &initialized);
  void zero_negative_flags(bool &initialized);
  void normalization_flags(Word mode, bool &initialized);
  void prepare_value_proof(const NativeOp &operation, NativeValueRanges &ranges);
  [[nodiscard]] bool admit_cached_status() const;
  void initialize_status();
  void materialize_status();
  void read_status(x64::Reg destination);
  void merge_status(x64::Reg source, Word changed = 0, x64::Reg scratch = x64::Reg::ax);
  [[nodiscard]] std::vector<Word> plan_flags(std::span<const NativeOp> operations) const;
  [[nodiscard]] Word written_flags(const NativeOp &operation) const;
  void input(x64::Reg destination, unsigned selector, bool destination_b, bool latched);
  void value(x64::Reg destination, unsigned index, bool latched);
  void current_value(x64::Reg destination, unsigned index);
  void store_accumulator(bool destination_b, x64::Reg source);
  void invalidate_moved_accumulators();
  [[nodiscard]] Word dirty_accumulators() const;
  void flush_accumulators(Word mask);
  void exit_if(x64::Branch condition);
  void finish();
  [[nodiscard]] bool admit_packed_retirement(std::span<const NativeOp> operations) const;
  void initialize_retirement();
  void materialize_retirement();
  void count_move();
  void materialize_pending_accounting();
  void retire_progress(const NativeOp &operation);
  [[nodiscard]] bool publish_pc(std::span<const NativeOp> operations, std::size_t index) const;
  void loop_guard(const NativeOp &operation);
  [[nodiscard]] static bool is_loop_setup(const NativeOp &operation);
  void loop_setup(const NativeOp &operation);
  void loop_push(x64::Reg frame);
  void loop_continue();
  void loop_backedge();
  void initialize_loop_residency();
  void initialize_loop_window();
  void guard_region_header();
  void batch_entry(std::span<const NativeOp> operations);
  void retire(const NativeOp &operation, bool publish);
  x64::Code code_;

  struct RingGuard {
    x64::Label entry, resume, slow;
    NativeMove move;
    bool profiled = false;
  };

  std::vector<RingGuard> ring_guards_;
  JitRuntime runtime_;
  std::array<Word, agu_registers> modifiers_;
  bool optimize_;
  BatchBackend batch_backend_;
  RegionMode region_mode_;
  LoopResidency residency_mode_;
  RangeMode range_mode_;
  WordReuseMode word_mode_;
  LoopWindowMode window_mode_;
  bool loop_window_ = false;
  Word word_valid_ = 0, word_populate_ = 0, word_selected_ = 0;
  bool reused_words_ = false;
  NativeValueProof value_proof_;
  NativeRegion region_;
  bool mac_batches_ = false;
  bool packed_retirement_ = false;
  bool retirement_helpers_ = false;
  bool cached_status_ = false;
  bool cached_inputs_ = false;
  bool coalesced_accounting_ = false;
  bool loop_residency_ = false;
  bool loop_bank_cache_ = false;
  Word pending_instructions_ = 0, pending_moves_ = 0, pending_extra_cycles_ = 0;
  NativeDataflow dataflow_;
  // Dirty implies valid. Closed resident loops seed both slots before all effects.
  std::array<bool, 2> accumulator_dirty_{};
  std::array<x64::Label, 4> exits_;
  x64::Label loop_header_;
  Word loop_size_ = 0, loop_pc_ = 0;
  Word live_flags_ = carry | overflow | zero | negative | unnormalized | extension;
};
} // namespace dsp56300::detail
