# Objective

Build an independent, maximally efficient DSP56300 emulator library for x86-64,
optimized for all usual DSP56300 applications and supporting one DSP instance
running arbitrary programs with the complete ISA.
Preserve exact fixed-point precision, rounding, saturation, flags, parallel
transfer ordering, AGU, loops, stack, interrupts and observable state. Translate
guest code into native regions, keeping live values in suitable GPR/SIMD
representations and minimizing host instructions, memory traffic and working set.
Use guarded native side exits, explicit bounded memory/event/I/O contracts and
independent correctness expectations. Keep the public integration interface
ordinary and transparent. Implement the CPU only, with no board emulation or
dependency on another DSP emulator in build, API, tests or runtime.

Current user requirements

- High performance is the library's central purpose. Ordinary consumers must
  automatically receive the fastest correctly supported execution path on the
  current host, including native JIT execution. Callers must not need to know
  private backend details or accidentally select a diagnostic interpreter by
  using the normal memory/I/O interface. Keep only JIT production execution;
  tests use independently authored expectations. Extend native execution to
  external memory and ordinary bounded event/I/O contracts; a fast isolated
  buffer benchmark is insufficient when real consumers take a slow path.
  Qualify actual native/helper execution and sustained consumer throughput.
- The complete DSP56300 ISA must be implemented and independently tested across
  all legal encodings, operands, numerical modes, aliases and observable effects.
  Representative workloads and a green current suite are insufficient evidence.
- All tests and benchmarks use independently authored DSP programs. External
  application/firmware programs and extracts are excluded from active inputs.
  The user canceled all application-specific coupling to prevent overfitting.
- Design representative audio, image/video, speech, communications, measurement
  and control scenarios. Optimize across these application domains and qualify
  cross-workload performance tradeoffs. Include filters, oscillators, effects,
  resampling, FFT,
  fixed-point transforms and dependency-heavy workloads. Compare independently
  calculated expected registers/RAM/progress and retain reproducible binary fixtures.
- Provide a CMake target named bench, analogous to correctness tests. Reuse the
  same fixtures, support all or selected fixture sets/cases and report actual
  guest MIPS, latency, host work, native/helper execution and code size. Keep
  initialization, cold compilation and warm execution scopes explicit. Do not
  turn faulting/nonrepeatable fixtures into misleading warm throughput numbers.
- Treat native compilation as research into one DSP program, including SIMD and
  multicore execution of proven independent work expressed serially. Preserve
  feedback, shared state, quantization, ordering and event/interrupt boundaries.
- Develop state/dataflow diagrams and small tables before major optimizations.
  Account for the complete architectural and in-flight state, aliases, old
  operands and all observation/exit classes. Use concrete layouts and cost models.
- Select register allocation and representation per region/pattern at runtime,
  including MAC/filter, FFT, address-heavy and control code. Compare qualified
  candidates using live/read/write sets and measured costs. A single fixed host
  assignment is not the final design. Use available GPR/SIMD registers and cache
  as operation-appropriate storage, retaining complete canonical Core state.
- Investigate both-branch evaluation with SIMD masks for suitable short pure
  regions. Commit only the selected state/effects; preserve dependent recurrences,
  failures, I/O and memory bounds. Measure whether prediction is a real bottleneck
  rather than assuming branchless or speculative work must improve execution.
- The user raised the performance reference to 275 MIPS and clarified that
  throughput benchmarks must model continuous DSP operation, not single-call
  microfixtures. Each scenario processes one real-time second of meaningful
  work near full DSP capacity in at most one host second. Report actual
  instructions and base cycles separately, retain all shortfalls, and do not
  replace a per-scenario minimum with an average. This is not a ceiling or
  stopping criterion. Continue maximizing
  throughput and minimizing CPU time/load and latency at that fixed load.
  Qualify sustained realtime and energy per work block separately. Core count
  and GHz alone do not establish energy efficiency; faster host completion must
  not accelerate guest time or bypass observable event boundaries.
- The user canceled hardware virtualization. Execute generated x64 directly in
  the host process; do not add a hardware-VM backend, tests or permission setup.
- Consumers use ordinary headers, owned/shared memory, events and synchronous
  I/O. Keep ownership, bounds, aliases, host mutation and invalidation explicit.
  Derivative clocks, peripherals and board/device behavior belong to consumers.
  Keep complete readable state owned by Core, with optional inspection at safe
  boundaries. The normal consumer does not manage CPU state. Instruction and
  memory effects preserve ISA ordering; physical pipeline phases and electrical
  wiring are not implementation goals. Remove opaque checkpoint APIs and their
  implementation. Do not expose phased effect-ticket APIs; execute through
  normal native execution.
- Use C++23, LLVM 19, CMake/Ninja, strict warnings, hardening, clang-format/tidy,
  and nonrecovering ASan/UBSan with native dependencies instrumented in strict
  builds. Keep functions at most 120 lines and do not suppress/weaken checks.
  Use one portable host-JIT concept for x86/SSE/AVX and future ARM/NEON emitters.
  Keep the public API aligned with cpu12; expose no production backend selector.
  Parameterize
  host SIMD lane width separately from vector width; validate combinations and
  retain exact DSP precision and runtime host-ISA admission.
- Keep entry points in src/core.cpp and src/jit.cpp; architectural helpers
  directly in src/ as in cpu12, JIT helpers in src/jit/ and native emission in
  src/jit/x64/. Keep borrowed memory in memory.hpp. Organize sources consistently
  and remove empty project directories.
- Keep neighboring application/firmware repositories read-only. Preserve
  GPL-3.0-only licensing and source/dependency attribution. Keep manufacturer
  reference copies and private analysis outside the tracked source tree.
- Commit verified
  increments and push reviewed source commits to the configured origin/main
  (Weichwolf/dsp56300), as explicitly authorized by the user. Do not publish
  reference assets or backup branches.
- Speak German with the user; write code, comments, documentation and commits
  in English. Keep local account names out of tracked source and instructions.

Discovered slow helper fallbacks and similar performance defects must be corrected promptly in this library, with independent correctness expectations and counters proving actual native execution; application-side workarounds are not a substitute.

JIT execution must not retire legal instructions through the general instruction
executor or general transfer helper. Audit every emitted helper and every
dispatcher fallback, including dynamic AGU and numerical modes, memory
guards, REP/DO boundaries, cache transitions and owner continuations. Native
code may call the explicit host I/O/event interface; that is distinct from
interpreting the instruction. Existing correct fallbacks remain temporary open
coverage while native replacements are implemented and independently verified.
Do not hide or relabel those gaps or discard their architectural/fault behavior.

Current development priority: use the fastest optimized Release build for
functional firmware experiments and performance work. Run affected core correctness
tests, including independent expectations and
appropriate sanitizer coverage, before publishing source changes. Core tests
remain active before the integrating instrument reaches real time. Keep strict
compilation, formatting and static analysis; never weaken checks.

Public setup follows Core(Memory, Io*): explicit typed borrowed views,
named unified/separate/partition/external layouts. The consumer owns allocations.
Keep native architectural units, physical aliases and owner lifetimes explicit.
No general virtual memory manager belongs in ordinary direct-RAM execution.

Public API agreement

CPU12 and DSP56300 expose the same small outward Core API; ISA and peripheral
contracts supply architecture-specific types. Core owns its full readable state;
normal consumers need no state or checkpoint management. Optional state access
must not add work to the execution path. Only JIT execution belongs in production;
future host architectures provide new implementations of the host-JIT concept.
Preserve every implemented instruction, including operations unused by Blofeld.

Before further emulator integration or optimization, both CPU12 and DSP56300
must approach matched host-ASM throughput on representative warmed workloads.
The complete instrument objective remains unchanged. Optimize native blocks,
loops, register/flag liveness and code layout in the cores, validate equivalent
results and report per-workload JIT/host percentages. Passing correctness tests
alone do not establish the performance gate.

Sequence: optimize CPU12 first, then optimize and simplify DSP56300. Target >=70%
of matched optimized host throughput on each realistic sustained warm scenario;
short reset/run/inspect microfixtures remain diagnostics and do not establish or
block that target. Use identical inputs, relevant ISA semantics and independent
result checks. Report each scenario rather than hiding weak cases in an average.

Provide a `bench` command under the tests that reports assumed hardware throughput
(target >=1x) and warm JIT/host throughput, with clock/cost assumptions and the
comparison scope. Cold runtime compilation must also fit actual processing packet
deadlines: report the worst cold and warm packets, first-use translations and
missed deadlines separately from average throughput. A cold microkernel time alone
is not a processing deadline. Add a persistent cache only if measurements justify
it as necessary to avoid first-use underruns.

Use focused validation during slow-core iteration and broaden it as performance
improves. Keep code small, with complexity justified by correctness or measured
speed. Subsequent complete Blofeld integration must sustain >=4x offline real-time
throughput on this host under representative high load while retaining correct
guest time and bounded live latency.
