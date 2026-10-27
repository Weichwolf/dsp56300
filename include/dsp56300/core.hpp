// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace dsp56300 {
using Word = std::uint32_t;
inline constexpr Word word_mask = 0x00ffffff;
inline constexpr std::size_t address_words = std::size_t{1} << 24;
inline constexpr std::uint64_t long_mask = 0x0000ffffffffffff;
inline constexpr std::uint64_t accumulator_mask = 0x00ffffffffffffff;
inline constexpr std::size_t agu_registers = 8;
inline constexpr std::size_t stack_entries = 16;

enum class Space : std::uint8_t { program, x, y };
enum class Backend : std::uint8_t { cached, reference, jit };
enum class BitAction : std::uint8_t { test, clear, set, change };
enum class FaultKind : std::uint8_t {
  none,
  io_read,
  io_write,
  io_bit,
  memory_read,
  memory_write,
  unsupported_instruction,
  unsupported_mode,
  stack,
  invalid_instruction
};

// Architectural values use unshifted DSP coordinates, including raw 56-bit A/B.
// These registers are not a complete checkpoint of in-flight REP/IRQ continuation.
struct Registers {
  std::uint64_t x = 0, y = 0, a = 0, b = 0;
  std::array<Word, agu_registers> r{}, n{}, m{};
  Word pc = 0, sr = 0, omr = 0, la = 0, lc = 0;
  Word sp = 0, sc = 0, sz = 0, ep = 0, vba = 0;
  std::array<std::uint64_t, stack_entries> stack{};
  bool operator==(const Registers &) const = default;
};

struct Progress {
  std::uint64_t instructions = 0;
  // Documented instruction base costs, excluding pipeline interlocks, IRQ entry
  // overhead and board/bus wait states. Not a calibrated hardware clock.
  std::uint64_t estimated_cycles = 0;
  bool operator==(const Progress &) const = default;
};

struct Fault {
  FaultKind kind = FaultKind::none;
  Space space = Space::program;
  Word address = 0, value = 0, pc = 0;
  bool operator==(const Fault &) const = default;
};

struct RunResult {
  // Legacy name: the number of retired guest instructions, on every backend.
  std::uint32_t dispatches = 0;
  Progress progress;
  Fault fault;
};

struct Options {
  Backend backend = Backend::cached;
  // Maximum straight-line translation length, bounded to [1,256].
  // A native hardware-loop backedge can repeat the body within the run budget.
  Word instructions_per_block = 64;
  // False keeps generated blocks but executes every operation through the helpers.
  bool optimize = true;
};

struct JitStatistics {
  std::uint64_t compiled_blocks = 0, native_alu_instructions = 0, helper_instructions = 0;
  // Successful instructions whose complete parallel transfer ran in emitted code.
  // This overlaps native_alu_instructions, which also includes move-only retirement.
  std::uint64_t native_move_instructions = 0;
  std::uint64_t executable_bytes = 0, invalidations = 0;
};

// No derivative peripherals are implemented by this CPU library. False means a
// visible fail-stop error. Callbacks run synchronously on the execution thread.
// Callbacks may read state and read/write memory, including P invalidation. They
// must not reenter run/reset/register initialization/interrupt admission.
class Io {
public:
  Io() = default;
  Io(const Io &) = default;
  Io &operator=(const Io &) = default;
  Io(Io &&) = default;
  Io &operator=(Io &&) = default;
  virtual ~Io() = default;
  virtual bool read(Space space, Word address, Word pc, Word &value) noexcept = 0;
  virtual bool write(Space space, Word address, Word pc, Word value) noexcept = 0;
  virtual bool
  bit(Space space, Word address, Word pc, Word bit, BitAction action, bool &previous) noexcept = 0;
};

// Three RAM address spaces, each word stored in uint32_t; sizes may differ.
// Memory beyond the supplied spans faults, except I/O routed to the host.
// Spans may overlap. The embedder owns contents/lifetime; mutate words aliasing
// P only through Core::load/write, including accesses through X/Y spans.
struct Buffers {
  std::span<Word> program, x, y;
};

// Linux virtual address reservation: 192 MiB of zero-on-demand memory, without
// touching cold pages. Only actually accessed pages contribute to the working set.
class MemoryArena {
public:
  MemoryArena();
  ~MemoryArena();
  MemoryArena(const MemoryArena &) = delete;
  MemoryArena &operator=(const MemoryArena &) = delete;
  MemoryArena(MemoryArena &&) noexcept;
  MemoryArena &operator=(MemoryArena &&) noexcept;
  [[nodiscard]] Buffers buffers() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Single-thread-owned CPU. Construction, cold compilation and loading may allocate.
// The public header deliberately exposes no native dependency types.
class Core {
public:
  explicit Core(Buffers buffers, Io *io = nullptr, Options options = {});
  ~Core();
  Core(const Core &) = delete;
  Core &operator=(const Core &) = delete;
  Core(Core &&) = delete;
  Core &operator=(Core &&) = delete;
  void reset();
  [[nodiscard]] Registers registers() const;
  // Changes visible registers, not hidden execution continuation. Use after reset
  // for initialization; this is not checkpoint restore.
  void set_registers(const Registers &state);
  [[nodiscard]] Progress progress() const noexcept;
  [[nodiscard]] Fault fault() const noexcept;
  // Native ALU instructions may still use an instrumented parallel-move helper.
  [[nodiscard]] JitStatistics jit_statistics() const noexcept;
  [[nodiscard]] Word read(Space space, Word address) const;
  void write(Space space, Word address, Word value);
  void load(Space space, Word address, std::span<const Word> words);
  // Exact guest-instruction budget on every backend, including each REP iteration.
  // Translated blocks never exceed the budget; a short tail can use the interpreter.
  // Faults require reset. CPU state after a failed dispatch is not committed evidence.
  [[nodiscard]] RunResult run(std::uint32_t instruction_budget);
  [[nodiscard]] bool interruptible() const;
  // Host-admitted vector/priority at a stopped boundary, not an on-chip IRQ controller.
  // The kernel verifies mask/REP constraints. Derivative arbitration stays with the host.
  void interrupt(Word vector, unsigned level);
  void clear_code_cache();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300
