// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dataflow.hpp"
#include "jit.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct NativeContext {
  JitRuntime runtime;
  bool *dirty = nullptr;
  std::uint64_t *native_moves = nullptr;
  std::uint32_t budget = 0;
};

class JitEmitter {
public:
  JitEmitter(JitRuntime runtime, bool optimize);
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const NativeOp> operations);

  [[nodiscard]] bool packed_retirement() const noexcept { return packed_retirement_; }

  [[nodiscard]] bool cached_status() const noexcept { return cached_status_; }

private:
  [[nodiscard]] bool lowerable(const NativeOp &operation) const;
  [[nodiscard]] static bool has_moves(const NativeOp &operation);
  [[nodiscard]] static bool is_addition(const NativeOp &operation);
  void helper(JitHelper function, const NativeOp &operation);
  void native(const NativeOp &operation);
  void moves(const NativeOp &operation);
  void general_moves(const NativeOp &operation);
  void move_guard(const NativeMove &move, x64::Label &slow);
  void move_next(const NativeMove &move);
  void move_delta(const NativeMove &move);
  void move_ring_delta(const NativeMove &move);
  void emit_ring_guards();
  void emit_ring_check(x64::Label &entry);
  void move_update(const NativeMove &move);
  void move_capture(const NativeMove &move, Word mode);
  void move_commit(const NativeMove &move);
  void move_read_register(x64::Reg value, const NativeMove &move, Word mode);
  void move_write_register(x64::Reg value, unsigned reg);
  void multiply(const NativeOp &operation);
  void addition(const NativeOp &operation);
  void round(Word mode);
  void arithmetic_flags(Word mode, bool carry_changed = false);
  [[nodiscard]] bool admit_cached_status() const;
  void initialize_status();
  void materialize_status();
  void read_status(x64::Reg destination);
  void merge_status(x64::Reg source, Word changed = 0, x64::Reg scratch = x64::Reg::ax);
  [[nodiscard]] std::vector<Word> plan_flags(std::span<const NativeOp> operations) const;
  [[nodiscard]] static Word written_flags(const NativeOp &operation);
  void input(x64::Reg destination, unsigned selector, bool destination_b, bool latched);
  void value(x64::Reg destination, unsigned index, bool latched);
  void current_value(x64::Reg destination, unsigned index);
  void store_accumulator(bool destination_b, x64::Reg source);
  [[nodiscard]] Word dirty_accumulators() const;
  void flush_accumulators(Word mask);
  void exit_if(x64::Branch condition);
  void finish();
  [[nodiscard]] bool admit_packed_retirement(std::span<const NativeOp> operations) const;
  void initialize_retirement();
  void materialize_retirement();
  void retire_progress(const NativeOp &operation);
  [[nodiscard]] bool publish_pc(std::span<const NativeOp> operations, std::size_t index) const;
  void loop_guard(const NativeOp &operation);
  void loop_continue();
  void loop_backedge();
  void retire(const NativeOp &operation, bool publish);
  x64::Code code_;

  struct RingGuard {
    x64::Label entry, resume, slow;
    NativeMove move;
  };

  std::vector<RingGuard> ring_guards_;
  JitRuntime runtime_;
  bool optimize_;
  bool packed_retirement_ = false;
  bool retirement_helpers_ = false;
  bool cached_status_ = false;
  NativeDataflow dataflow_;
  // Only native writes establish a live host value; dirty also implies valid.
  std::array<bool, 2> accumulator_dirty_{};
  std::array<x64::Label, 4> exits_;
  x64::Label loop_header_;
  Word loop_size_ = 0, loop_pc_ = 0;
  Word live_flags_ = carry | overflow | zero | negative | unnormalized | extension;
};
} // namespace dsp56300::detail
