# Target JIT architecture

This is the design target, not a statement of implementation completeness.
The same execution architecture applies to cpu12 and dsp56300; their ISA lowering,
register allocation and memory semantics differ. Optimize CPU12 first, then DSP.

```text
consumer: borrowed memory, peripherals, events and virtual time
                       |
                 Core(Memory, Io*)
                       |
       cold: decode -> control-flow graph -> semantic IR
                       -> liveness -> allocation -> native emission
                       -> immutable executable arena
                       |
       warm: native entry -> linked native regions -> native loop
                       |                           |
                  direct memory              precise cold exit
                                                   |
                                      I/O / event / sleep / fault
```

## Public contract and ownership

The public surface consists of core.hpp and memory.hpp. ISA encodings live in
test/support/isa.hpp; production decoding/transfer helpers stay private. Core owns
all architectural state and translations. Optional read-only state inspection is
available at execution-owner boundaries; it does not copy state on every step.
There are no checkpoint, restore, pipeline-ticket or backend-selection APIs.
Production execution is JIT-only. Unsupported operations fail visibly with PC,
operation, address and context; an interpreter or instruction helper is not a
silent fallback. Real peripheral callbacks are distinct from ISA helpers.

One thread owns execution of a Core. The consumer owns memory lifetimes,
peripheral state, shared-buffer synchronization, virtual clocks and pacing.
Neither core contains Blofeld protocols, an audio scheduler or physical wiring.
The host emitter is a private implementation boundary: x86 today, other hosts
later. Do not add abstractions to an executed instruction merely for symmetry.

## Cold path: compile control flow once

Decode from the actual entry PC into a compact semantic representation. Basic
blocks end at architectural control-flow boundaries. Combine connected blocks
into regions with native backedges; a function may span many basic blocks.
An indirect branch, unavailable code, I/O or an event boundary may require an exit.
No fixed 64/256-instruction cap, execution-budget-sized compilation or arbitrary
function-size promise determines translation boundaries. At load time, eagerly
walk statically reachable paths from consumer-supplied code roots such as reset
and interrupt entries, and link known direct targets before execution. Code and
data may share an image; never linearly decode every byte as executable code.
Unknown indirect targets are resolved and translated on first use, then retained.
The same compiler serves eager preparation and runtime discovery; neither path
introduces a second execution engine or claims to know all indirect destinations.

The IR represents register reads/writes, exact arithmetic, flag effects, memory
effects and control flow. Analyze liveness across loop backedges to a fixed point.
Fold constants and remove dead work only when all observable ISA effects survive.
This is general compilation, not recognition or replacement of benchmark programs.

Finish code and relocations before publishing an entry. Retain executable code
for the entire Core lifetime in compact shared arenas, not a page per instruction.
Known successors use native branches. Unresolved or indirect successors use a
cold resolver and retained target slots; never return through a C++ dispatcher
for every instruction or loop iteration. Mutable link data is separate from
immutable machine code. Preserve the register convention across linked regions.
Place connected hot regions together and move fault/diagnostic paths out of line.
Align deliberately; measure code expansion and instruction-cache behavior.

## Warm path: keep values in host registers

Load live architectural values at native entry and keep them in allocated host
registers across instructions and backedges. Spill only under actual register
pressure or at an observable boundary. Do not reload/store the entire register
file, advance a memory-resident PC, or invoke a generic helper after every opcode.
The current guest PC is normally compile-time metadata, materialized when needed.

Compute only flags with a future use before their next overwrite. Preserve sticky
flags and flags visible through explicit status reads, interrupts, I/O callbacks
or exits. Host flags are usable only where their semantics match. Keep a compact
pending expression when that is cheaper; materialize the exact guest status at
observation points. Flag elimination must include exceptional and cold exits.

ISA-specific native lowering handles widths, carries, saturation, wrapping and
address updates. Memory and parallel operations retain their required ordering.
No counters, tracing, instruction hashes or heap allocation belong in the default
hot path. Architectural time progress is required state, not telemetry. Optional
telemetry selects a separate instrumented path at a safe runtime boundary.

## Memory and code identity

Validate borrowed mappings on binding: backing, bounds, permissions, aliases and
lifetimes. Lower proven RAM accesses to native loads/stores. Eliminate a guard
only if its premise remains true throughout the execution region. Dynamic
knownness, permissions or ambiguous addresses retain necessary checks. Peripheral
holes and unknown accesses take explicit cold paths, never fabricated RAM values.
Cross-region accesses preserve word ordering and actual callback semantics.

Normal execution assumes a stable program identity and avoids continuous code
hashing. A changed bank or explicitly programmed region receives a new identity
at a safe boundary; old native code stays retained but is no longer selected for
that identity. A genuine RAM-resident updater keeps its architectural state.
Loading an unrelated program creates a new Core. A writable-code contract must
explicitly handle modification or refuse it; silently executing stale code is
incorrect. Core code contains no firmware-update protocol.

## Precise exits without a per-instruction dispatcher

Annotate potential exits with guest PC, live-value locations and completed effects.
Publish canonical architectural state when returning to the owner or invoking a
callback that observes it. Keep fault exits cold. Preserve completed external
writes; a failed callback cannot be replayed as an execution-yield mechanism.
Do not use one universal rollback rule for ISA operations with different effects.

Execution is bounded by the next relevant event, not by a compilation cap.
Use static cycle progress and region/loop guards where valid. A deadline inside a
region takes an exact instruction-boundary path within the retained translation;
it does not create a new budget-specific translation. Data-dependent timing and
I/O require the additional boundaries their semantics demand. An interrupt poll
at a loop backedge is sufficient only if its latency contract permits it.
Exact observation can cost work; choose the least expensive correct placement
rather than promising zero checks for arbitrary asynchronous behavior.

The consumer defines offline lookahead and live buffering. Host speed never
changes guest clock rates. Sleep/STOP returns or parks execution until a real
wake condition; it does not busy-loop or invent an interrupt.

## Implementation and acceptance order

1. Join native instructions into retained control-flow regions and native loops.
2. Keep registers resident; eliminate dead flags, PC stores and redundant loads.
3. Specialize proven memory paths; link regions and minimize boundary work.
4. Measure each bottleneck before adding SIMD or further compiler machinery.

Use short optimized experiments and focused independent semantic checks while
speed is low; broaden validation as performance improves. Keep strict compilation
and check definitions intact. Benchmarks live outside the core and use named ISA
encodings. Compare identical inputs, relevant guest semantics and independently
verified results against optimized handwritten host ASM. Report cold compilation,
warm native-entry throughput and full Core.run throughput separately. Include
code size and boundary costs; do not disguise slow fixtures with an average.

The gate is at least 80% of matched host-ASM throughput for every representative
warm fixture in each core. This is an ambitious measured acceptance requirement,
not a theorem that arbitrary guest code can match an unconstrained native
algorithm. Start with ALU, flags, memory and branches; include substantial loops,
Mandelbrot/FFT and architecture-appropriate instrument workloads before declaring
the set representative. Both cores must pass before further emulator integration.
Then establish the complete instrument and at least 4x sustained offline realtime
under representative high load. No current performance or completion is implied.

## DSP56300 lowering

Preserve logical P/X/Y spaces and aliases over ordinary host memory; Harvard
addressing is guest semantics, not a requirement for separate physical host RAM.
Keep 24-bit words, 56-bit accumulators, extension/limiting, saturation, rounding
and sticky flags exact. Parallel transfers read the required old values before
publishing ordered destinations. Preserve AGU modulo/reverse-carry addressing,
REP/DO loop semantics and interrupt boundaries.

Use measured SSE/AVX register packing where it reduces native instructions and
spills. Keep accumulators and hot AGU state resident. SIMD is an implementation
tool, not permission to change fixed-point results or operation order. Lower the
implemented ISA natively; retire redundant execution engines, helper fallback
machinery and public internals as their real dependencies are removed. Preserve
implemented instructions and the independent tests that establish their meaning.
FIR, biquad, FFT and nonlinear kernels must exercise transfers and address updates
as well as arithmetic; a fast isolated MAC is insufficient gate evidence.

Primary references are listed in references.json.
