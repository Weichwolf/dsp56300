// SPDX-License-Identifier: GPL-3.0-only
#pragma once
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
};

class JitEmitter {
public:
  JitEmitter(JitRuntime runtime, bool optimize);
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const NativeOp> operations);

private:
  [[nodiscard]] bool lowerable(const NativeOp &operation) const;
  [[nodiscard]] static bool has_moves(const NativeOp &operation);
  [[nodiscard]] static bool is_addition(const NativeOp &operation);
  void helper(JitHelper function, const NativeOp &operation);
  void native(const NativeOp &operation);
  void moves(const NativeOp &operation);
  void general_moves(const NativeOp &operation);
  void move_guard(const NativeMove &move, x64::Label &slow);
  void move_update(const NativeMove &move);
  void move_capture(const NativeMove &move, Word mode);
  void move_commit(const NativeMove &move);
  void move_read_register(x64::Reg value, const NativeMove &move, Word mode);
  void move_write_register(x64::Reg value, unsigned reg);
  void multiply(const NativeOp &operation);
  void addition(const NativeOp &operation);
  void round(Word mode);
  void arithmetic_flags(Word mode, bool carry_changed = false);
  void input(x64::Reg destination, unsigned selector, bool destination_b, bool latched);
  void value(x64::Reg destination, unsigned index, bool latched);
  void current_value(x64::Reg destination, unsigned index);
  void store_accumulator(bool destination_b, x64::Reg source);
  [[nodiscard]] Word dirty_accumulators() const;
  void flush_accumulators(Word mask);
  void exit_if(x64::Branch condition);
  void finish();
  void loop_guard(const NativeOp &operation);
  void retire(const NativeOp &operation);
  x64::Code code_;
  JitRuntime runtime_;
  bool optimize_;
  NativeDataflow dataflow_;
  // Only native writes establish a live host value; dirty also implies valid.
  std::array<bool, 2> accumulator_dirty_{};
  std::array<x64::Label, 4> exits_;
};
} // namespace dsp56300::detail
