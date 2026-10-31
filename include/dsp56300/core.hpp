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
enum class Backend : std::uint8_t { cached, reference, jit };
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
};

struct Options {
  Backend backend = Backend::cached;
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
  // External instruction owner supplies captured words and advances real effects.
  // Staged transfers use this bus except for retained program-cache hits;
  // ordinary run requires RAM buffers.
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
  void restore(const Checkpoint &checkpoint);
  [[nodiscard]] InstructionFetch instruction_fetch() const;
  [[nodiscard]] InstructionProgress
  begin_instruction(InstructionWords words,
                    InstructionPublication publication = InstructionPublication::retirement);
  // Own logical opcode/extension bus reads, preserving REP's genuine opcode latch.
  [[nodiscard]] InstructionProgress
  begin_fetch(InstructionPublication publication = InstructionPublication::retirement);
  [[nodiscard]] std::optional<PendingInstruction> pending_instruction() const;
  [[nodiscard]] InstructionProgress advance_instruction(const EffectTicket &ticket);
  [[nodiscard]] bool interruptible(InstructionWords words) const;
  // Eligibility needs only the captured opcode; this query performs no fetch.
  [[nodiscard]] bool interruptible_opcode(InstructionWords words) const;
  // Capture an admitted request without fetching a vector or acknowledging a device.
  void request_interrupt(Word vector, unsigned level, InstructionWords foreground);
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
