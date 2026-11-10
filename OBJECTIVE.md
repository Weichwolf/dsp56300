# Objective

Build a complete, independent, high-performance DSP56300 native JIT library for
arbitrary consumer programs. Preserve the entire implemented ISA and complete
architectural state, flags, addressing, memory ordering, reset, IRQ and wait
semantics; complete unsupported legal forms with independent tests. Unsupported
host/ISA/access cases fail explicitly with useful context.

Expose the same minimal Core/Memory/Io shape as the other core, through core.hpp
and memory.hpp. Core owns state; consumers own memory and supply device I/O,
clocks, scheduling and synchronization. No application firmware, foreign core,
checkpoint interface or public backend selector. Production execution is native-only;
x86-64 and ARM64 lower the same semantics. See doc/architecture.md.

Target Ahead-of-Time Binary Translation integrated with consumer host code.
The consumer orchestrates complete executable-image analysis, code/variant
identity and compilation; this library supplies exact semantic ISA translation.
Require complete preparation of known executable code and variants before their
execution. Incomplete translation fails with identity, address and reason;
optimized-subset coverage is not complete native translation coverage. Known
loader output and finite instruction variants can be prepared in advance while
preserving genuine loads/program writes and selecting validated native entries.
Mutable data stays in borrowed host RAM; it does not prevent AOT.

Prefer consumer startup loading and complete AOT before playback, with the same
native code quality as build-time compilation. Report preparation/startup and
compiler memory separately from warm execution. Loading at application startup
does not permit lazy translation during playing.

Allow the consumer to generate a local translation at build time and compile/link
it with its own host application. Mandatory preparation may be a build phase;
it need not occur in the launched instrument. Keep firmware-derived artifacts
outside tracked core sources/tests and validate exact image/binding/host identity.
Fuse compatible typed consumer binding IR with ISA IR before LLVM 19 O3 to
optimize across that boundary. Keep LLVM handles private and preserve ordered
I/O, faults, interrupts and precise observable state.

Support build-time AOT in the core project itself: separate semantic translation
and optimization from process-local executable installation. The same frontend
and LLVM-O3 pipeline must support startup materialization and export of linkable
LLVM bitcode or relocatable native objects for a consumer build. Do not duplicate
ISA semantics or require copying core internals into the application. Include exact
code/variant, binding ABI, target/features and compiler identity; resolve symbols
and instance addresses through validated bindings/relocations rather than baking
process pointers into persistent artifacts. Keep LLVM handles private and the
public API minimal. The application orchestrates inputs, linking and lifecycle.

Prioritize connected instrument integration and fix generic defects here.
Keep native registers live, RAM direct and translations immutable and retained.
Use natural control-flow regions without instruction-count translation caps.
Do not hide ISA helper gaps, fabricate retirement or discard supported operations.

Provide an explicit side-effect-free preparation phase before execution. Discover
reachable instructions from established entries without running guest/device
operations or scanning data; retain unresolved and indirect frontiers. Lift exact
ISA semantics into LLVM 19 CFG/SSA, optimize and lower for the native host. Keep
LLVM types private and preparation ownership stopped; failures must not mutate
architectural state or progress. Preparation time has no fixed artificial cap.
Retain exact instruction/cycle prefixes and ordered effects in optimized regions.

Provide reproducible independently authored fixtures using test/support/isa.hpp;
organize test/isa, test/core, test/jit, test/bench and test/support consistently.
The bench target reports cold/warm latency, packet deadlines, guest instructions
and base cycles, code size, and native throughput against equivalent portable
C++ algorithms compiled by Clang 19 O3 for the same host CPU/features. Target
parity and verify independently checked complete results, exact fixed-width
arithmetic and observable memory/state. Prevent dead-code elimination. Retain
handwritten ASM/SIMD comparisons and existing gates separately. Report actual
guest lowering and preparation coverage; ordinary native-emitter measurements
do not establish complete LLVM-O3/AOT throughput. Keep hardware-factor assumptions
explicit; report every slow scenario and first-use miss, not averages alone.
>=70% realistic warm host-ASM throughput is a reference, not an integration gate.
Optimize continuous meaningful workloads across domains; SIMD is optional.

Prioritize complete instrument behavior and sustained throughput. Use strict
builds and focused independent Release checks for affected code. Routine
sanitizers may wait for the instrument goal; earlier runs need a concrete
technical reason. Cover our code and emitted JIT code, not packaged foreign
libraries; never rebuild LLVM/compiler-rt as part of the toolchain.
Manufacturer assets and application program extracts stay outside source/tests.
The Blofeld consumer requires >=4x sustainable offline instrument throughput and
bounded live latency; isolated core benchmarks do not establish that result.

Cover exact fixed-point arithmetic, all numerical modes, parallel transfers,
AGU, REP/DO and stack effects. Representative streams include audio/speech,
image/video transforms, communications, measurement and control. The declared
275 MHz/base-cycle benchmark reference is a model, not a measured chip speed;
qualify sustained throughput and energy per work separately from clock ratios.
