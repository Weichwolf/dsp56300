// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "x64.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail {
struct NativeContext {
  JitRuntime runtime;
  bool *dirty = nullptr;
};

class JitEmitter {
public:
  JitEmitter(JitRuntime runtime, bool optimize);
  [[nodiscard]] std::vector<std::uint8_t> generate(std::span<const NativeOp> operations);

private:
  [[nodiscard]] bool lowerable(const NativeOp &operation) const;
  [[nodiscard]] static bool has_moves(const NativeOp &operation);
  void helper(JitHelper function, const NativeOp &operation);
  void native(const NativeOp &operation);
  void multiply(const NativeOp &operation);
  void round(Word mode);
  void arithmetic_flags(Word mode, bool carry_changed = false);
  void input(x64::Reg destination, unsigned selector, bool destination_b, bool latched);
  void value(x64::Reg destination, unsigned index, bool latched);
  void loop_guard(const NativeOp &operation);
  void retire(const NativeOp &operation);
  x64::Code code_;
  JitRuntime runtime_;
  bool optimize_;
};
} // namespace dsp56300::detail
