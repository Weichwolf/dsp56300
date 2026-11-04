// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "translation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300::detail {
struct NativeOp {
  Instruction instruction;
  Word pc = 0, mode = 0, loop_address = inactive_loop;
};

constexpr bool at_loop_end(const NativeOp &operation) noexcept {
  return operation.loop_address == ((operation.pc + operation.instruction.words - 1) & word_mask);
}

using JitHelper = std::uint32_t (*)(void *, const NativeOp *) noexcept;

struct JitMemory {
  Word *data = nullptr;
  std::size_t words = 0;
  bool aliases_program = false;
};

struct JitRuntime {
  void *owner = nullptr;
  std::array<std::uint64_t, 4> *latched = nullptr;
  JitHelper execute = nullptr, moves = nullptr;
  std::optional<Word> (*program)(void *, Word) noexcept = nullptr;
  std::array<JitMemory, 2> memory;
  std::array<std::span<const MemoryWindow>, 2> windows{};
  // Conservative for standalone low-level contexts without a declared chip map.
  bool cacheable_program = true;
};

struct NativeContext {
  JitRuntime runtime;
  bool *exit_requested = nullptr;
  std::uint64_t *native_moves = nullptr;
  std::uint64_t *batch_instructions = nullptr, *avx2_instructions = nullptr, *batch_calls = nullptr;
  std::array<std::uint8_t *, 2> write_known{};
  std::uint32_t budget = 0;
};

class Jit {
public:
  Jit(Cpu &cpu, std::span<const Word> program, Options options, JitRuntime runtime);
  ~Jit();
  Jit(const Jit &) = delete;
  Jit &operator=(const Jit &) = delete;
  [[nodiscard]] static bool available() noexcept;
  [[nodiscard]] std::uint32_t run(std::uint32_t budget);
  [[nodiscard]] std::uint32_t run_bounded(std::uint32_t instructions, std::uint64_t cycles);
  [[nodiscard]] std::uint32_t run_repeat(NativeOp operation, std::uint32_t budget);
  [[nodiscard]] std::uint32_t run_captured(NativeOp operation);
  [[nodiscard]] bool exit_requested() const noexcept;
  void request_yield() noexcept;
  void invalidate() noexcept;
  void bind_windows(std::array<std::span<const MemoryWindow>, 2> windows) noexcept;
  void reset();
  void helper_retired() noexcept;
  [[nodiscard]] JitStatistics statistics() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300::detail
