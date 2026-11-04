// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/checkpoint.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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
enum class Backend : std::uint8_t { cached, reference, jit, automatic };
enum class BatchBackend : std::uint8_t { automatic, disabled, scalar, avx2 };
enum class ProcessingState : std::uint8_t { running, waiting, stopped, debugging };

// Qualified IME/SWO view, not a complete OnCE register or JTAG implementation.
struct DebugStatus {
  bool interrupt_mode = false, software_occurrence = false;
  bool operator==(const DebugStatus &) const = default;
};
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
  host_event
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

// A receipt for an actually retired staged instruction, never a prediction.
// Explicit taken control flow remains visible even when its target is sequential.
struct InstructionOutcome {
  std::uint64_t instructions = 0;
  Word address = 0, opcode = 0;
  bool control_transfer = false;
  bool operator==(const InstructionOutcome &) const = default;
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
  // Sleeping returns early without a fault; further run calls retire nothing.
  ProcessingState state = ProcessingState::running;
  Progress progress;
  Fault fault;
  bool yielded = false;
};

struct RunBudget {
  std::uint32_t instructions = 0;
  std::uint64_t cycles = 0;
};

enum class RunStop : std::uint8_t {
  instruction_budget,
  cycle_budget,
  external_effect,
  unknown_code,
  control_event,
  state,
  yielded,
  fault
};

struct Options {
  Backend backend = Backend::automatic;
  // Maximum straight-line translation length, bounded to [1,256].
  // A native hardware-loop backedge can repeat the body within the run budget.
  Word instructions_per_block = 64;
  // False keeps generated blocks but executes every operation through the helpers.
  bool optimize = true;
  // Half-open external P interval eligible for the architectural instruction cache.
  // Empty by default: the embedder must declare its derivative's external map.
  Word cacheable_begin = address_words, cacheable_end = address_words;
  // Eight sectors, each 128 (1K total) or 256 (2K total) DSP words.
  unsigned cache_sector_words = 128;
  // Guarded native MAC-loop batches. Unavailable AVX2 uses the scalar kernel.
  BatchBackend batch_backend = BatchBackend::automatic;
};

struct JitStatistics {
  std::uint64_t compiled_blocks = 0, native_alu_instructions = 0, helper_instructions = 0;
  // Successful transfers completed by native lowering or a compiled batch.
  // This overlaps native_alu_instructions, which also includes move-only retirement.
  std::uint64_t native_move_instructions = 0;
  std::uint64_t executable_bytes = 0, invalidations = 0;
  // Compiled bulk execution, included in native_alu_instructions/native moves.
  std::uint64_t batch_instructions = 0, avx2_instructions = 0, batch_calls = 0;
};

// Explicit ordinary storage grant. Empty known means all words are initialized;
// otherwise each byte is zero for unknown or nonzero for a known 24-bit word.
// Grants never include peripherals, side effects, wait states or unstable maps.
struct MemoryWindow {
  Word address = 0;
  std::span<Word> words;
  std::span<std::uint8_t> known;
  bool writable = false, aliases_program = false;
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

  // Guest RESET preserves CPU state. Accept only after resetting derivative
  // peripherals and their interrupt priorities; default rejection is explicit.
  virtual bool reset_peripherals(Word /*pc*/) noexcept { return false; }

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

struct BoundedRunResult {
  RunResult execution;
  std::uint64_t cycles = 0;
  RunStop stop = RunStop::instruction_budget;
  Word pc = 0;
  std::optional<Word> next_cycles;
  // Snapshot of genuinely known code at the stopped boundary. An owner may
  // latch this instruction before clocks/events advance, retaining original
  // semantics if code changes during the in-flight reservation.
  std::optional<InstructionWords> next;
};

struct InstructionFetch {
  Word address = 0;
  std::optional<Word> latched_opcode;
  // True only while native exception/interrupt entry owns this fetch.
  bool exception_vector = false;
};

enum class InstructionPublication : std::uint8_t {
  retirement,
  address_updates,
  move_updates,
  alu_updates,
  unit_updates,
  pcu_updates
};

enum class EffectKind : std::uint8_t {
  read,
  write,
  bit,
  reset,
  retire,
  address_update,
  move_update,
  alu_update,
  fetch_opcode,
  fetch_extension,
  unit_update,
  pcu_update
};

struct AddressWriteback {
  std::uint8_t index = 0;
  Word previous = 0, value = 0;
  bool operator==(const AddressWriteback &) const = default;
};

// Physical storage, in the same unshifted coordinates as Registers. The mask
// records a native write even when its value equals the previous value.
enum class RegisterStorage : std::uint8_t {
  x,
  y,
  a,
  b,
  r,
  n,
  m,
  sr,
  omr,
  la,
  lc,
  sp,
  sc,
  sz,
  ep,
  vba,
  stack,
  pc
};
enum class MoveProducer : std::uint8_t { register_write, accumulator_read, stack_read, long_load };

struct RegisterUpdate {
  RegisterStorage storage = RegisterStorage::x;
  std::uint8_t index = 0;
  std::uint64_t mask = 0, previous = 0, value = 0;
  bool operator==(const RegisterUpdate &) const = default;
};

struct MoveWriteback {
  MoveProducer producer = MoveProducer::register_write;
  // Native register encoding, or the L: selector for long_load.
  std::uint8_t selector = 0, count = 0;
  std::array<RegisterUpdate, 3> updates{};
  bool operator==(const MoveWriteback &) const = default;
};

// Native data-ALU/BFU and conditional-transfer results, held before PCU retirement.
struct AluWriteback {
  Word opcode = 0;
  std::uint8_t count = 0;
  std::array<RegisterUpdate, 3> updates{};
  bool operator==(const AluWriteback &) const = default;
};

enum class RegisterUnit : std::uint8_t { bit, control_byte, address_calculation };

// Actual remaining scalar-unit writes, separate from ALU/MOVE and PCU retirement.
struct UnitWriteback {
  RegisterUnit unit = RegisterUnit::bit;
  Word opcode = 0;
  std::uint8_t count = 0;
  std::array<RegisterUpdate, 3> updates{};
  bool operator==(const UnitWriteback &) const = default;
};

// Actual native program-control register writes; hidden state retires separately.
struct PcuWriteback {
  Word opcode = 0;
  std::uint8_t count = 0;
  std::array<RegisterUpdate, 8> updates{};
  bool operator==(const PcuWriteback &) const = default;
};

struct InstructionEffect {
  EffectKind kind = EffectKind::retire;
  Space space = Space::program;
  Word address = 0, value = 0, pc = 0, omr = 0, bit = 0;
  BitAction action = BitAction::test;
  // Native automatic R writeback. address is the computed EA; space is unused.
  // This is an owned CPU publication boundary, never a host bus transaction.
  std::optional<AddressWriteback> address_update = std::nullopt;
  std::optional<MoveWriteback> move_update = std::nullopt;
  std::optional<AluWriteback> alu_update = std::nullopt;
  std::optional<UnitWriteback> unit_update = std::nullopt;
  std::optional<PcuWriteback> pcu_update = std::nullopt;
  bool operator==(const InstructionEffect &) const = default;
};

class EffectTicket {
public:
  EffectTicket(const EffectTicket &) = default;
  EffectTicket &operator=(const EffectTicket &) = default;
  EffectTicket(EffectTicket &&) noexcept = default;
  EffectTicket &operator=(EffectTicket &&) noexcept = default;
  ~EffectTicket() = default;

private:
  friend class Core;

  struct Position {
    std::uint64_t epoch;
    std::size_t ordinal;
  };

  EffectTicket(std::shared_ptr<const bool> owner, Position position);
  std::shared_ptr<const bool> owner_;
  std::uint64_t epoch_ = 0;
  std::size_t position_ = 0;
};

struct PendingInstruction {
  EffectTicket ticket;
  InstructionEffect effect;
  // Actual captured opcode, absent until the native fetch has completed.
  // It identifies the owning instruction, not a pipeline or atomicity guarantee.
  std::optional<Word> opcode = std::nullopt;
  // Native computation before this effect; observation is not publication.
  Registers computed;
};

struct InstructionProgress {
  std::optional<PendingInstruction> pending = std::nullopt;
  Progress progress;
  Fault fault;
  bool retired = false;
};

// Three RAM address spaces, each word stored in uint32_t; sizes may differ.
// Memory beyond the supplied spans faults, except I/O routed to the host.
// Spans may overlap. The embedder owns contents/lifetime; mutate words aliasing
// P only through Core::load/write, including accesses through X/Y spans.
struct Buffers {
  std::span<Word> program, x, y;
};

class Memory;

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
  [[nodiscard]] Memory memory() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Single-thread-owned CPU. Construction, cold compilation and loading may allocate.
// The public header deliberately exposes no native dependency types.
class Core {
public:
  explicit Core(Memory memory, Io *io = nullptr, Options options = {});
  // Raw-view compatibility entry; new integrations use Memory explicitly.
  explicit Core(Buffers buffers, Io *io = nullptr, Options options = {});
  // External memory owner supplies every real P/X/Y/peripheral transfer.
  // A pure known code view enables ordinary native execution. Staged interfaces
  // additionally expose effect boundaries. Architectural cache state remains
  // separate from translated code; the owner pairs external bus snapshots.
  explicit Core(Io &bus, Options options = {});
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
  // Capture/restore complete stopped CPU continuation and architectural cache.
  // The caller restores matching RAM and I/O separately. Host code caches clear.
  [[nodiscard]] Checkpoint checkpoint() const;
  // Capture the complete computed successor state at an owned retirement
  // boundary without publishing it on this CPU. This is speculative: the
  // caller must preserve instruction ordering and pair external memory/I/O
  // with the already accepted effects. Tickets remain owned by this CPU.
  [[nodiscard]] Checkpoint candidate_checkpoint(const EffectTicket &ticket) const;
  void restore(const Checkpoint &checkpoint);
  [[nodiscard]] InstructionFetch instruction_fetch() const;
  // Empty during an active instruction, after initialization/fault, or after
  // nonzero atomic run. Checkpoints retain stopped, actual retirement receipts.
  [[nodiscard]] std::optional<InstructionOutcome> instruction_outcome() const noexcept;
  [[nodiscard]] InstructionProgress
  begin_instruction(InstructionWords words,
                    InstructionPublication publication = InstructionPublication::retirement);
  // Own logical opcode/extension bus reads, preserving REP's genuine opcode latch.
  [[nodiscard]] InstructionProgress
  begin_fetch(InstructionPublication publication = InstructionPublication::retirement);
  [[nodiscard]] std::optional<PendingInstruction> pending_instruction() const;
  // Actual computed instruction/cycle increment at an owned pcu_updates
  // retirement boundary. Observation neither publishes nor performs I/O.
  // These native base units do not establish an external bus/pipeline clock.
  [[nodiscard]] Progress retirement_delta(const EffectTicket &ticket) const;
  [[nodiscard]] InstructionProgress advance_instruction(const EffectTicket &ticket);
  [[nodiscard]] bool interruptible(InstructionWords words) const;
  // Eligibility needs only the captured opcode; this query performs no fetch.
  [[nodiscard]] bool interruptible_opcode(InstructionWords words) const;
  // Capture an admitted request without fetching a vector or acknowledging a device.
  void request_interrupt(Word vector, unsigned level, InstructionWords foreground);
  // Admit from actual CPU state without requiring a foreground snapshot. No
  // fetch/vector execution occurs; unresolved completed-REP eligibility stays explicit.
  void request_interrupt(Word vector, unsigned level);
  [[nodiscard]] Progress progress() const noexcept;
  [[nodiscard]] Fault fault() const noexcept;
  // Native ALU instructions may still use an instrumented parallel-move helper.
  [[nodiscard]] JitStatistics jit_statistics() const noexcept;
  [[nodiscard]] Backend backend() const noexcept;
  [[nodiscard]] Word read(Space space, Word address) const;
  void write(Space space, Word address, Word value);
  void load(Space space, Word address, std::span<const Word> words);
  // Exact guest-instruction budget on every backend, including each REP iteration.
  // Translated blocks never exceed the budget; a short tail can use the interpreter.
  // Faults require reset. CPU state after a failed dispatch is not committed evidence.
  [[nodiscard]] RunResult run(std::uint32_t instruction_budget);
  // Execute a completed CPU-only prefix within both bounds. Return before any
  // unproven external effect/control transition, with actual next PC/base cost.
  // Unknown code is an explicit boundary; this query never invents a fetch.
  [[nodiscard]] BoundedRunResult run(RunBudget budget);
  // Execute exactly the captured ordinary instruction through this CPU/JIT,
  // including synchronous real Io. The caller owns its external clock delay.
  // Refuses pending vector/debug/cache fetch ownership and mismatched PC/latch.
  // Backing code may have changed since capture. Like atomic run(), this does
  // not expose staged publication boundaries or a staged flow receipt.
  [[nodiscard]] RunResult run_instruction(InstructionWords words);
  // A synchronous Io callback can return this run after actual instruction
  // retirement. Guest state and compiled code survive; no cross-thread mutation.
  void request_yield() noexcept;
  [[nodiscard]] ProcessingState processing_state() const noexcept;
  [[nodiscard]] DebugStatus debug_status() const noexcept;
  // Configure the functional OnCE interrupt-mode selection at stopped execution.
  void set_debug_interrupt_mode(bool enabled);
  // Complete a host DE/JTAG debug request; required clock stabilization is host-owned.
  void request_debug();
  void resume_from_debug();
  // Complete host-modeled IRQA wake/clock stabilization from STOP. The host then
  // admits its highest pending interrupt, or runs the next foreground instruction.
  // This does not wake WAIT; use an admitted interrupt, reset or the debug interface.
  void resume_from_stop();
  [[nodiscard]] bool interruptible() const;
  // Host-admitted vector/priority at a stopped boundary, not an on-chip IRQ controller.
  // The kernel verifies mask/REP constraints. Derivative arbitration stays with the host.
  void interrupt(Word vector, unsigned level);
  void clear_code_cache();

private:
  [[nodiscard]] InstructionProgress begin_owned(std::optional<InstructionWords> captured,
                                                InstructionPublication publication);
  [[nodiscard]] bool interruptible_captured(InstructionWords words, bool complete) const;
  friend class Checkpoint;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300
