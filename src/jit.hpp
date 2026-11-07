// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "translation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300::detail {
struct BlockInstruction {
  Instruction instruction;
  Word pc = 0, mode = 0, loop_address = inactive_loop;
};

constexpr bool at_loop_end(const BlockInstruction &operation) noexcept {
  return operation.loop_address == ((operation.pc + operation.instruction.words - 1) & word_mask);
}

using JitHelper = std::uint32_t (*)(void *, const BlockInstruction *) noexcept;

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

enum class NativeBoundary : std::uint8_t { none, owner, context };

enum class BusAction : std::uint8_t { read, write };

struct NativeAccess {
  Fault failure{};
  Word address = 0, value = 0, pc = 0;
  Space space = Space::program;
  BusAction action = BusAction::read;
};

struct NativeWindow {
  Word address = 0;
  std::size_t words = 0;
  Word *data = nullptr;
  std::uint8_t *known = nullptr;
  bool writable = false, aliases_program = false;
};

struct NativeWindows {
  const NativeWindow *data = nullptr;
  std::size_t count = 0;
};

struct NativeBus {
  JitMemory program{};
  std::array<NativeWindows, 2> windows{};
  CacheRegion cache{};
  void *owner = nullptr;
  void (*access)(void *, NativeAccess &) noexcept = nullptr;
  Fault *failure = nullptr;
  bool external = false;
};

struct NativeContext {
  JitRuntime runtime;
  bool *exit_requested = nullptr;
  std::array<std::uint8_t *, 2> write_known{};
  std::uint32_t budget = 0;
  // Helpers return to the owner; retained loop-context changes may stay native.
  NativeBoundary boundary = NativeBoundary::none;
  Fault failure{};
  NativeBus bus{};
  bool program_written = false;
};

class Jit {
public:
  Jit(Cpu &cpu, std::span<const Word> program, JitRuntime runtime, NativeBus bus = {});
  ~Jit();
  Jit(const Jit &) = delete;
  Jit &operator=(const Jit &) = delete;
  [[nodiscard]] static bool available() noexcept;
  [[nodiscard]] std::uint32_t run(std::uint32_t budget);
  [[nodiscard]] std::uint32_t run_repeat(BlockInstruction operation, std::uint32_t budget);
  [[nodiscard]] std::uint32_t run_captured(BlockInstruction operation);
  [[nodiscard]] bool exit_requested() const noexcept;
  void request_yield() noexcept;
  void invalidate() noexcept;
  [[nodiscard]] bool bind_windows(std::array<std::span<const MemoryWindow>, 2> windows) noexcept;
  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300::detail
