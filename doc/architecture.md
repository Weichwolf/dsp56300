# Target JIT architecture

Shared design for cpu12 and dsp56300; this is a target, not an implementation claim.

```text
consumer memory + I/O -> Core -> retained native regions and loops
                                  |                     |
                             direct RAM          real event/I/O/fault
```

Core owns complete readable state; consumers own memory and device scheduling.
Only core.hpp and memory.hpp are public. One owner executes each core; different
cores may run independently. No board model, universal core hierarchy, checkpoint
API, effect-ticket API or production interpreter belongs in the library.

Compile from actual code entries along natural control flow. Retain immutable
translations for the core lifetime and link successors natively. No instruction
count caps, budget-specific recompilation, continuous hashing or cache eviction.
Changed executable identities require an explicit owner transition; unrelated
programs use a fresh core. Provide explicit preparation before execution: capture
initialized immutable code from established entries, share decoded fetch metadata
with ordinary compilation, and follow explicit direct branch/call/return-site
edges. Keep unknown code, register/stack targets, IRQs and implicit REP/DO flow
unresolved until their contracts are proved. Never scan unrelated data or execute
guest instructions, I/O or clocks during preparation.

Lift exact fixed-width ISA semantics into LLVM 19 CFG/SSA, preserving flags,
aliases, ordered effects and precise event/cycle prefixes. Use constant/range
propagation, dead-store elimination, forwarding and loop optimization before
native lowering; optimizing the C++ emitter alone is insufficient. Both hosts
share this semantic pipeline; target lowering and native execution ABI differ.
Install code only at a stopped owner boundary after validating the captured code
revision. Compilation failure leaves architecture/progress unchanged. Keep LLVM
handles private, instrument emitted native code explicitly in sanitizer builds,
and initialize LLVM once per process before parallel preparation. Core destruction
must not shut down LLVM while another core owns code. Preparation has no fixed
time cap; persistent caching is optional when measured latency justifies it.

On Linux x86-64, resolve both 24-bit PCs and complete 32-bit numeric/loop keys
with one indexed load from non-executable, demand-zero target tables. Reserve
128 MiB of virtual addresses per PC table and 32 GiB for the core's context
catalog, without committing RAM or swap for untouched entries. Physical backing
follows populated pages; clearing visits installed targets rather than the whole
key range. Mapping failures remain visible. Other hosts retain sparse radix routing.

Keep live values in GPR/SIMD registers across instructions and backedges. Preserve
old operands, aliases, flags and ordered effects. Eliminate dead work and hoist
bounds/permission checks only with valid proofs. Exact faults, interrupts and
owner observations publish complete architectural state. Actual I/O calls may
continue natively; generic ISA helper execution remains an explicit coverage gap.

Use compact executable arenas, deliberate layout and cold failure paths. Avoid
per-instruction dispatch, state copies, allocations and telemetry. Translation
boundaries do not depend on execution grants; event handling retains precise
observable ISA prefixes. Consumer buffering and pacing do not change guest time.
Masked external IRQs leave ordinary native execution running. Query the consumer
at real interrupt-mask changes and return before another instruction when an IRQ
needs arbitration. The consumer owns priority and acknowledgement; the core owns
eligibility, REP, interrupt shadows and native vector entry.

Native instructions and cold helpers consume the same current RAM grants.
Known reads and writable non-program-alias stores use borrowed memory directly.
Insufficient grants retain owned callbacks, including unknown words, denied
writes and executable aliases; genuine stores update their knowledge bytes.

Measure realistic continuous workloads against matched optimized host ASM with
identical inputs, relevant semantics and independently checked results. Report
cold translation, warm execution, API/boundary costs, code size and each slow case.
>=70% warm host-ASM throughput is a reference, not an integration gate. Keep
hardware-factor assumptions explicit. SIMD is optional; add complexity only for
correctness or measured speed. Test ISA/flags, RAM/aliases, control, IRQ/wait,
Mandelbrot/FFT and application-appropriate streams. Benchmarks live in test/bench/.

Preserve logical P/X/Y spaces and physical aliases over borrowed host memory,
24-bit words, 56-bit accumulators, fixed-point rounding/saturation/sticky flags,
parallel old-value transfers, AGU modulo/reverse carry, REP/DO and IRQ semantics.
Prove dependencies and ranges before batching independent loop iterations;
retain exact feedback, lane effects and partial exits. Cover FIR, biquad, FFT,
nonlinear feedback, communications, transforms and control workloads.

Prove every accumulator prefix before reducing channel results. Structured RAM
loads and stores require both iteration independence and preserved read/write
order for aliases. Retain ordered native execution when either proof fails.

Primary references: references.json.
