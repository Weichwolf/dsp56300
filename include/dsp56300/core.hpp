// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>

namespace dsp56300 {
inline constexpr std::uint64_t long_mask = 0x0000ffffffffffff;
inline constexpr std::uint64_t accumulator_mask = 0x00ffffffffffffff;
inline constexpr std::size_t agu_registers = 8;
inline constexpr std::size_t stack_entries = 16;

enum class ExecutionState : std::uint8_t { running, waiting, stopped, debugging };

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
  invalid_instruction,
  host_event,
  unprepared_execution
};

// Architectural values use unshifted DSP coordinates, including raw 56-bit A/B.
// These registers do not include in-flight REP/IRQ continuation.
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
  // Number of retired guest instructions.
  std::uint32_t retired = 0;
  // Sleeping returns early without a fault; further run calls retire nothing.
  ExecutionState state = ExecutionState::running;
  Progress progress;
  Fault fault;
  bool yielded = false;
};

// Read-only snapshot. DSP accumulators are converted from native JIT coordinates
// only when requested; inspection adds no work to ordinary execution.
struct State {
  Registers registers;
  Fault failure;
  ExecutionState execution = ExecutionState::running;
  Progress progress;
};

struct PreparationReport {
  std::uint64_t prepared_entries = 0, unresolved_entries = 0, native_bytes = 0;
  Fault failure;
};

struct CodeSpan {
  Word address;
  std::span<const Word> words;
};

struct CodeVariant {
  std::uint64_t id;
  std::span<const CodeSpan> code;
  std::span<const Word> entries;
};

struct CodeImage {
  std::span<const CodeVariant> variants;
};

struct RegionLayout {
  Space space;
  Word first;
  std::size_t size;
  bool writable = true, tracked = false, aliases_program = false;
};

struct VariantLayout {
  std::uint64_t id;
  std::span<const RegionLayout> memory;
};

struct InterruptRequest {
  Word vector;
  unsigned level;
};

struct CodeTransition {
  std::uint64_t from, to;
  Word entry;
};

struct CompileOptions {
  std::span<const VariantLayout> memory;
  std::span<const InterruptRequest> interrupts;
  // An absent resident target may yield only to this explicitly prepared version.
  std::span<const CodeTransition> transitions;
};

class NativeImage {
public:
  NativeImage() = default;
  [[nodiscard]] PreparationReport report() const noexcept;

private:
  struct Impl;
  explicit NativeImage(std::shared_ptr<const Impl> image) noexcept;
  std::shared_ptr<const Impl> impl_;
  friend class Core;
};

// No derivative peripherals are implemented by this CPU library. False means a
// visible fail-stop error. Callbacks run synchronously on the execution thread.
// Callbacks may read state and read/write memory, including P invalidation. They
// must not reenter run/reset/register initialization/interrupt admission.
class Io {
public:
  virtual ~Io() = default;

  // Guest RESET preserves CPU state. Accept only after resetting derivative
  // peripherals and their interrupt priorities; default rejection is explicit.
  virtual bool reset_peripherals(Word /*pc*/) noexcept { return false; }

  // Queried only after a completed instruction changes the SR interrupt mask.
  // True requests an owner boundary; it neither admits nor acknowledges an IRQ.
  [[nodiscard]] virtual bool interrupt_pending(Word /*status*/) const noexcept { return false; }

  // Optional side-effect-free code view. Unknown/unstable words retain actual
  // owned fetches. The owner changes the revision on code or mapping mutation.
  [[nodiscard]] virtual std::optional<Word> inspect_program(Word /*address*/) const noexcept {
    return std::nullopt;
  }

  // Revision also covers granted data-window descriptors, pointers and permissions.
  [[nodiscard]] virtual std::uint64_t program_revision() const noexcept { return 0; }

  // Descriptors/storage remain valid until the owner changes its revision at an
  // execution boundary or in a synchronous callback. Sorted, disjoint X/Y grants
  // below the peripheral interval; executable aliases require explicit marking.
  [[nodiscard]] virtual std::span<const MemoryWindow> memory_windows(Space /*space*/) noexcept {
    return {};
  }

  virtual bool read(Space space, Word address, Word pc, Word &value) noexcept = 0;
  virtual bool write(Space space, Word address, Word pc, Word value) noexcept = 0;
  virtual bool
  bit(Space space, Word address, Word pc, Word bit, BitAction action, bool &previous) noexcept = 0;
};

struct InstructionWords {
  Word address = 0, opcode = 0;
  std::optional<Word> extension = std::nullopt;
};

struct InstructionFetch {
  Word address = 0;
  std::optional<Word> latched_opcode;
  // True only while native exception/interrupt entry owns this fetch.
  bool exception_vector = false;
};

// Single-thread-owned CPU. Construction, cold compilation and loading may allocate.
// The public header deliberately exposes no native dependency types.
class Core {
public:
  explicit Core(Memory memory = Memory::external(), Io *io = nullptr);
  ~Core();
  Core(const Core &) = delete;
  Core &operator=(const Core &) = delete;
  [[nodiscard]] static NativeImage compile(CodeImage image, CompileOptions options);
  // Cold owner boundary: validate code/layout and bind RAM/events before publishing.
  void install(const NativeImage &image, std::uint64_t variant);
  void reset();
  [[nodiscard]] State state() const;
  // Completed nominal work; does not materialize a register snapshot.
  [[nodiscard]] Progress progress() const noexcept;
  // Explicit host programming; ordinary data may be accessed through borrowed memory.
  void load(Space space, Word address, std::span<const Word> words);
  // Compile established immutable code without executing instructions or device I/O.
  // Call on a stopped owner; preparation failures do not latch into state/progress.
  [[nodiscard]] PreparationReport prepare(std::span<const Word> established_entries);
  [[nodiscard]] RunResult step();
  [[nodiscard]] Registers registers() const;
  // Changes visible registers, not hidden execution continuation. Use after reset
  // for initialization; hidden continuation remains owned by Core.
  void set_registers(const Registers &state);
  [[nodiscard]] InstructionFetch instruction_fetch() const;
  [[nodiscard]] bool interrupt_ready(InstructionWords words) const;
  // Eligibility needs only the captured opcode; this query performs no fetch.
  [[nodiscard]] bool interrupt_ready_opcode(InstructionWords words) const;
  // Capture an admitted request without fetching a vector or acknowledging a device.
  void request_interrupt(Word vector, unsigned level, InstructionWords foreground);
  // Admit from actual CPU state without requiring a foreground snapshot. No
  // fetch/vector execution occurs; unresolved completed-REP eligibility stays explicit.
  void request_interrupt(Word vector, unsigned level);
  // Exact guest-instruction budget, including each REP iteration.
  // Translated code respects the exact execution boundary.
  // Faults require reset. CPU state after a failed dispatch is not committed evidence.
  [[nodiscard]] RunResult
  run(std::uint32_t instruction_budget = std::numeric_limits<std::uint32_t>::max());
  // Execute at native speed to the first complete instruction boundary at or
  // beyond this absolute base-cycle deadline; yields, faults and sleep return
  // earlier. The result-counter width can also end a call, as with run().
  // This is nominal ISA progress, not a derivative peripheral/bus clock.
  // Translation and retained code never depend on the supplied deadline.
  [[nodiscard]] RunResult run_until(std::uint64_t base_cycle_deadline);
  // Execute exactly the captured ordinary instruction through this CPU/JIT,
  // including synchronous real Io. The caller owns its external clock delay.
  // Refuses pending vector/debug/cache fetch ownership and mismatched PC/latch.
  // Backing code may have changed since capture; the supplied words own this execution.
  [[nodiscard]] RunResult run_instruction(InstructionWords words);
  // A synchronous Io callback can return this run after actual instruction
  // retirement. Guest state and compiled code survive; no cross-thread mutation.
  void request_yield() noexcept;
  // Complete host-modeled IRQA wake/clock stabilization from STOP. The host then
  // admits its highest pending interrupt, or runs the next foreground instruction.
  // This does not wake WAIT; use an admitted interrupt, reset or the debug interface.
  void resume_from_stop();
  [[nodiscard]] bool interrupt_ready() const;
  // Host-admitted vector/priority at a stopped boundary, not an on-chip IRQ controller.
  // The kernel verifies mask/REP constraints. Derivative arbitration stays with the host.
  void interrupt(Word vector, unsigned level);

private:
  [[nodiscard]] Fault fault() const noexcept;
  [[nodiscard]] ExecutionState processing_state() const noexcept;
  void clear_code_cache();
  [[nodiscard]] bool interrupt_ready_captured(InstructionWords words, bool complete) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300
