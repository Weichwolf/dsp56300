# Common core contract, version 9

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

A core is generic. What a particular firmware does or uses is no argument for
or against anything in a core. Firmware enters in two places only: as a
declaration of the consumer (for example the memory mode an image is prepared
in), and as a whole-instrument workload that measures a consumer. A core
feature is justified by the architecture and by the common kernels of section
8; no test, table or special case in a core is shaped after a firmware.

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
- Preparation is cacheable without lowering. `Core::compiler_identity()` is a
  cheap constant that changes whenever lowering output could change (lowering
  source revision, LLVM version). `SemanticImage::artifact()` returns the
  complete serialized image: bitcode, exports, identity and every decoded
  contract `bind` needs. `Core::load(CodeImage, CompileOptions, artifact) ->
  SemanticImage` reconstructs the image without lowering and rejects an artifact
  whose compiler identity or input digest does not match the given image and
  options. The consumer owns the cache: its key, storage, the optimized bitcode
  and the relocatable native objects. No instance pointer, memory content or
  device state enters an artifact.
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

Checks in translated code: as many as necessary, as few as possible. Firmware
is fixed code written for this processor, not arbitrary software. The executed
path checks only what the original processor would observe and what the
translation itself assumed; a check against behavior the installed image cannot
show does not belong there.

- Bounds and event admission are required for exactness. They are evaluated
  once per grant: at the entry of a unit, or once for the number of loop turns
  the bound admits, with accounting at the exit. Never per instruction, never
  per turn.
- A memory access over a declared fixed layout whose address is constant, or
  affine in the loop turn, is proven once before the unit or the loop. Only an
  access whose address is computed from data is checked at the access, by one
  comparison.
- An assumption of the translation (mode, mapping, code identity, return
  target, loop state) is checked where it can change. Its failure is a cold
  exit to exact dynamic semantics. It is never wrong execution and never
  undefined behavior of the host.
- Diagnostics are not in the product path: per-word knowledge tracking,
  counters, observers and consistency assertions exist only in a build or
  binding the consumer selects, and cost nothing when not selected.
- A check stays on the executed path only if removing it would change what the
  original processor does in normal operation, or if it guards an assumption of
  the translation at the one place where that assumption can change. "It can
  happen on this architecture" is a reason for a cold exact mode the consumer
  can select, not for a check on the hot path.
- When a whole-unit or whole-turn grant fails, the remainder up to the bound is
  the exact tail: it may execute instruction by instruction with a check each,
  cold and out of line, using the same instruction lowering as the unit.

## 5. State

- `registers()` / `set_registers()` expose the visible registers.
- `state()` is a read-only inspection that adds no work to ordinary execution.
- `save() -> Snapshot` and `restore(const Snapshot&)` transfer the complete
  processor state at an instruction boundary, including hidden continuation
  (repeat and loop state, pending vector, interrupt delay, latched fault).
  They do not touch prepared code or consumer memory. A consumer uses them for
  checkpoints, speculation with rollback and state files.
- After `restore` the core holds exactly the execution state of the snapshot and
  no residue of anything executed after it. A pending declared code transfer is
  part of that state: `save` records whether one is pending and to which
  variants, and `restore` reinstates or clears it accordingly. Which image is
  installed remains binding state owned by the consumer.

## 6. Memory and I/O

- The consumer owns all storage and lends it through `Memory`. Declared fixed,
  initialized layouts permit direct native loads and stores; everything else
  keeps exact dynamic semantics. Optional per-word knowledge tracking is the
  consumer's choice.
- Storage is lent as regions in the address spaces the processor defines
  (program, X and Y; program and data memory; the bus and tightly coupled
  memories). A storage interface private to the processor is such an address
  space with its own regions, addressed from zero. It is never a buffer hidden
  in the core and never a second naming scheme beside regions. Which space
  serves an access is architectural state of the guest.
- A consumer may lend an address space completely, as one reservation of host
  virtual memory covering every address the guest can form, with host page
  protection expressing what is memory, what is read-only and what is absent.
  For such a space the core performs no address admission: an access is the
  view base plus the guest address. Where the processor has an MMU, the core
  derives the host view from the actual guest translation state for every
  mapping host pages can express, and runs exact where they cannot.
- `CompileOptions::MemoryMode { exact, direct }`, default `exact`, is the
  consumer's declaration. In `direct` mode an access the guest mapping or the
  host protection forbids is a host fault that ends the run with a reported
  fault naming address, operation and guest PC; it is not delivered to the
  guest. In `exact` mode the core checks and delivers what the architecture
  defines. The product build may enforce protection at host-page granularity;
  the consumer's checked build is the exact reference.
- `[[nodiscard]] static bool Core::host_fault(std::uintptr_t address, bool
  writing, std::uintptr_t native_pc) noexcept` is called from the consumer's
  process-wide fault handler, with the interrupted thread's signal mask in
  effect (the handler restores it first and reinstates its own mask if the
  call returns false). The core returns false unless the active owner is inside
  generated code and the address lies in a view it was lent; otherwise it
  records a fixed fault and escapes at its native-call adapter, inside all C++
  execution scopes, without a system call on a successful native entry. The
  handler does no allocation, formatting, decoding or state reconstruction.
  After such a fault the instance is not resumed.
- `NativeSymbols::fault_map` carries, as immutable data copied by `bind`, the
  executable text ranges of the linked image (`TextRange {address, size}`) and
  the mapping from native code intervals to guest instructions (`Site
  {address, size, variant, guest_pc}`), taken from the linker and from the
  line table of the image. Lowered guest instructions carry a debug location
  with line `guest_pc + 1`; nothing is emitted on the access path for it.
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
| K1 | Block FIR, 64 taps, 256 blocks of 1024 samples | multiply-accumulate in a hardware or counted loop |
| K2 | Cascade of 4 biquads, 256 blocks of 1024 samples | feedback state, rounding, saturation |
| K3 | 256-point radix-2 complex FFT | address arithmetic, tables, nested loops |
| K4 | Control: table-driven state machine with calls | branches, subroutine calls, stack, indirect transfers |
| K5 | Block copy with running checksum, 4096 words | loads, stores, loop overhead |
| K6 | K1 under event load | a run bound every 256 base cycles and one interrupt per 4096, handler increments a counter |

K1, K2 and K6 share one numerical definition. Samples are signed Q1.n; every
quantized node rounds to nearest, ties to even, then saturates. The input is
the 32-bit generator `x = x * 1664525 + 1013904223` from seed `0x243f6a88`,
taken at half amplitude, with the positive rail at every sample index divisible
by 97 and the negative rail at the next one. K1 is causal with history
continuous across blocks and zero at start. K2 is transposed direct form II
with the output and both updated states quantized. Each core uses its native
precision: cpu12 and armv5 Q1.15, dsp56300 Q1.23, sharc Q1.31. The generator of
coefficients, initial state and expected output exists once, in dsp56300
`test/bench/kernel_reference.hpp`; the other cores bind only to these SHA-256
values of the complete result, taken over signed 32-bit little-endian words:

| Precision | Result | SHA-256 |
|---|---|---|
| Q1.15 | K1 output | `01e342abb432c2b44339220d04d3376c55ec4ae30cb71ca783c7db9d3639c81b` |
| Q1.15 | K2 output | `b598303fc1e7736b8ee0bc5175afe8baf38d6d9ef81f833f79724ab5a0a785ba` |
| Q1.15 | K2 final state | `c013f34a4563d1948b54c15f1d7ebc48337af2792c24968c43f1a98e1e063c35` |
| Q1.23 | K1 output | `e4cee01f3826c7629135ecb224a4f8fcd18a225d6c248d1505dd7b15e6f54489` |
| Q1.23 | K2 output | `fc87b101ab8b0be1ddb5aa13db865b2c6a44f059450432ea5c872f3849351381` |
| Q1.23 | K2 final state | `d3fb6119d7cc9de2918f841f54772c88902132bb8d19357c0fae8522f8866439` |
| Q1.31 | K1 output | `e0dd28045a3afa7150f68203e73f97539933139b0823efd64705cc1c63ea7462` |
| Q1.31 | K2 output | `ab87d136a23ea837637b11274be389f0f31473c1d447d4f6fc752e823b20fe3a` |
| Q1.31 | K2 final state | `7fa60e5ab99a82116e669d32aec3e3af0cba013e494fd3b2af331d553c27e94f` |

K6 produces the K1 output. Guest kernels are written as firmware for that
processor would be, each in under 100 source lines. For K3 to K5 a core uses
the natural equivalent of a concept its instruction set lacks and documents it
in `test/bench/README.md`. K6 measures the cost of bounds and events, which
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
