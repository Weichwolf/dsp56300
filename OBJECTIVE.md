# Objective

Build a complete, independent, high-performance DSP56300 native JIT library for
arbitrary consumer programs. Preserve the entire implemented ISA and complete
architectural state, flags, addressing, memory ordering, reset, IRQ and wait
semantics; complete unsupported legal forms with independent tests. Unsupported
host/ISA/access cases fail explicitly with useful context.

Expose the same minimal Core/Memory/Io shape as the other core, through core.hpp
and memory.hpp. Core owns state; consumers own memory and supply device I/O,
clocks, scheduling and synchronization. No application firmware, foreign core,
checkpoint interface or public backend selector. Production execution is JIT-only;
x86-64 and ARM64 lower the same semantics. See doc/architecture.md.

Prioritize connected instrument integration and fix generic defects here.
Keep native registers live, RAM direct and translations immutable and retained.
Use natural control-flow regions without instruction-count translation caps.
Do not hide ISA helper gaps, fabricate retirement or discard supported operations.

Provide reproducible independently authored fixtures using test/support/isa.hpp;
organize test/isa, test/core, test/jit, test/bench and test/support consistently.
The bench target reports cold/warm latency, packet deadlines, guest instructions
and base cycles, code size, and JIT/host throughput against matched handwritten
ASM with independently checked complete results. Keep hardware-factor assumptions
explicit; report every slow scenario and first-use miss, not averages alone.
>=70% realistic warm host-ASM throughput is a reference, not an integration gate.
Optimize continuous meaningful workloads across domains; SIMD is optional.

Use strict builds and affected independent correctness/sanitizer tests before
publishing code. Tests remain required while Blofeld is below real time.
Manufacturer assets and application program extracts stay outside source/tests.
The Blofeld consumer requires >=4x sustainable offline instrument throughput and
bounded live latency; isolated core benchmarks do not establish that result.

Cover exact fixed-point arithmetic, all numerical modes, parallel transfers,
AGU, REP/DO and stack effects. Representative streams include audio/speech,
image/video transforms, communications, measurement and control. The declared
275 MHz/base-cycle benchmark reference is a model, not a measured chip speed;
qualify sustained throughput and energy per work separately from clock ratios.
