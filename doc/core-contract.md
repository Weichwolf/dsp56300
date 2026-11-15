# Common core contract, version 3

Every processor core in this family (cpu12, dsp56300, armv5, sharc and later
ones) follows this contract. The file is identical in all core repositories; a
change is made in all of them together. Where a core's own documents disagree,
this contract wins. An emulator must be able to embed any core the same way.

## 1. Boundary

A core translates one instruction set and owns the architectural processor
state. It contains no peripheral or device model, board wiring, firmware
knowledge, clock source, scheduler or multi-core coupling, and no special case
for one firmware's code patterns. The consumer owns all of those and supplies
its knowledge through the interface below. Test for every interface decision:
could a second device on this processor use the core without a core change?

## 2. Public surface

- Exactly two headers: `include/<core>/core.hpp` and `include/<core>/memory.hpp`.
  Namespace `<core>`, CMake target `<core>::core`.
- No LLVM types, native entry ABIs or implementation layouts in public headers.
- The same type and function names in every core, differing only in
  ISA-specific widths and register sets:
  `Registers`, `State`, `Snapshot`, `Progress`, `Fault`, `ExecutionState`,
  `RunResult`, `Memory`, `Io`, `CodeSpan`, `CodeVariant`, `CodeImage`,
  `RegionLayout`, `VariantLayout`, `CodeTransition`, `InterruptRequest`,
  `CallbackBinding`, `CompileOptions`, `SemanticImage`, `NativeImage`,
  `NativeSymbols`, `PreparationReport`, `Core`.

## 3. Code preparation

- The consumer declares executable code: spans, instruction entries, variants
  (coherent code versions), interrupt vectors, transitions between variants and
  per-variant memory layouts. Readable data is never treated as code.
- `Core::lower(CodeImage, CompileOptions) -> SemanticImage` is static and
  side-effect-free: it returns unoptimized LLVM bitcode, the exported entry
  names and an identity. It needs no Core instance, RAM or native engine.
- The consumer may link its own IR (devices, callbacks) before optimization and
  owns final optimization and native code. `Core::bind(SemanticImage,
  NativeSymbols) -> NativeImage` binds addresses; `Core::compile` is the
  stand-alone shortcut through the same path.
- `Core::install(NativeImage, variant)` runs on a stopped owner, validates the
  actual code words and memory layout, and changes no guest state or time.
- One code generator: LLVM. No interpreter and no hand-written per-host emitter
  in production. Unsupported, undeclared or changed code fails explicitly with
  identity, address and reason; a code or mapping change revokes bindings.

## 4. Execution

- One owner thread executes a Core. `run(instruction_budget)` and
  `run_until(base_cycle_deadline)` return at the first complete instruction
  boundary at or beyond the bound, or earlier on fault, sleep, yield or a
  declared transfer. Translation never depends on the bound.
- `Progress` reports completed instructions and nominal base cycles. Cycles are
  documented instruction costs, not a device clock.
- `request_yield()` may be called from a synchronous callback on the owner.
- The consumer arbitrates and acknowledges interrupt sources. The core owns
  eligibility (masks, priorities, shadows) and native vector entry:
  `interrupt_ready(...)`, `interrupt(InterruptRequest)`, `reset(...)`.
  Wake-up without vector entry, where the instruction set has it, is the
  ISA-specific `resume_from_stop()`; it is never modelled as reset or as an
  interrupt.
- There is no `step`, `prepare`, captured-instruction or statistics entry point:
  `run(1)` steps, `install` validates and reports, and translation counters are
  not part of the interface.

## 5. State

- `registers()` / `set_registers()` expose the visible registers.
- `state()` is a read-only inspection that adds no work to ordinary execution.
- `save() -> Snapshot` and `restore(const Snapshot&)` transfer the complete
  processor state at an instruction boundary, including hidden continuation
  (repeat and loop state, pending vector, interrupt delay, latched fault).
  They do not touch prepared code or consumer memory. A consumer uses them for
  checkpoints, speculation with rollback and state files.

## 6. Memory and I/O

- The consumer owns all storage and lends it through `Memory`. Declared fixed,
  initialized layouts permit direct native loads and stores; everything else
  keeps exact dynamic semantics. Optional per-word knowledge tracking is the
  consumer's choice.
- `set_memory(Memory)` rebinds storage on a stopped owner, for devices whose
  memory map changes at run time. It revokes dependent bindings; a matching
  `install` follows before execution continues. The core learns no device map.
- `Io` is a plain struct of `noexcept` function pointers plus an owner pointer.
  A callback returning false is an explicit fault. Callbacks run synchronously
  on the owner and must not re-enter execution. `CallbackBinding` lets the
  consumer replace a role by a symbol in its linked IR.

## 7. Verification

Keep the suite small enough to read. Three layers, nothing else:

- `test/isa`: independently authored instruction-set programs with independently
  computed expectations, organized by operation class, addressing mode and
  numeric mode. This is where coverage lives.
- `test/core`: the public interface and lifecycle of sections 3 to 6, including
  exact run bounds, faults, interrupts, code change and save/restore.
- `test/bench`: the common benchmarks of section 8, which also run as
  correctness tests because every kernel checks its complete output.

No tests of internal data structures, of one lowering strategy, or of a path
that production does not execute. No manufacturer firmware or sequences
extracted from it in a core; firmware-level tests belong to the consumer.

## 8. Common benchmarks

Every core carries the same six kernels in `test/bench`, written in that core's
assembly from one shared definition. Fixed inputs, complete bit-exact expected
output computed by an independent reference, sizes chosen so one run retires at
least ten million guest instructions.

| Id | Kernel | Exercises |
|---|---|---|
| K1 | Block FIR, 64 taps over 1024 samples | multiply-accumulate in a hardware or counted loop |
| K2 | Cascade of 4 biquads over 1024 samples | feedback state, rounding, saturation |
| K3 | 256-point radix-2 complex FFT | address arithmetic, tables, nested loops |
| K4 | Control: table-driven state machine with calls | branches, subroutine calls, stack, indirect transfers |
| K5 | Block copy with running checksum, 4096 words | loads, stores, loop overhead |
| K6 | K1 under event load | a run bound every 256 base cycles and one interrupt per 4096, handler increments a counter |

A core whose instruction set lacks a concept uses its natural equivalent
(fixed-point or floating-point as the processor provides) and documents it in
`test/bench/README.md`. K6 measures the cost of bounds and events, which
dominates a real multi-processor emulator and appears in no hardware benchmark.

Reported for every kernel, on each host, warm, from the public interface only:

- host instructions and host cycles per guest instruction (hardware counters);
- guest instructions per second;
- ratio to the same algorithm in portable C++ compiled with Clang O3 for the host,
  under the same numeric contract;
- preparation seconds and native code bytes.

External reference points, reported next to the kernels and not replacing them:

- Microcontroller cores: CoreMark 1.0, built with a stated compiler and flags and
  reported by its run rules (Apache 2.0 with EEMBC's acceptable-use terms; do
  not publish a modified version under the CoreMark name). Sources stay untracked.
- DSP cores: two or three programs from the manufacturer's own benchmark
  material (DSP56300 Family Manual, Appendix B "Benchmark Programs"; the
  processor benchmark table of the SHARC data sheet), run as published. They
  also check the core's base-cycle accounting against the documented counts.

These numbers are core results. They never stand in for a consumer's
whole-instrument throughput.

## 9. Outside the contract

Anything a single consumer needs for its scheduler or devices stays in that
consumer. A capability that every co-simulation needs is proposed as a contract
change, not added to one core.
