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
programs use a fresh core. Consider eager reachable-code preparation or persistent
caching only when measured first-use latency justifies it.

Keep live values in GPR/SIMD registers across instructions and backedges. Preserve
old operands, aliases, flags and ordered effects. Eliminate dead work and hoist
bounds/permission checks only with valid proofs. Exact faults, interrupts and
owner observations publish complete architectural state. Actual I/O calls may
continue natively; generic ISA helper execution remains an explicit coverage gap.

Use compact executable arenas, deliberate layout and cold failure paths. Avoid
per-instruction dispatch, state copies, allocations and telemetry. Translation
boundaries do not depend on execution grants; event handling retains precise
observable ISA prefixes. Consumer buffering and pacing do not change guest time.

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

Primary references: references.json.
