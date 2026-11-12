# Objective

Build complete DSP56300 native translation for the Blofeld Desktop SL instrument
on Linux x86-64 and macOS ARM64. Preserve ISA/state, memory ordering, flags, reset,
IRQ/wait and precise nominal progress. Implement missing legal forms with independent tests;
unsupported inputs fail explicitly with identity, address and reason.

Choose minimal public APIs, state ownership/layouts and LLVM composition for the
complete instrument's maximum performance and efficiency. Core repositories are
organizational boundaries. Keep LLVM private and production execution native-only.

## Compilation and execution

- Complete AOT of all known executable code, variants, indirect/IRQ entries and
  retained fetch forms before use. Separate executable streams from readable
  data. A partial optimizing subset is not complete translation.
- One semantic frontend serves startup installation and build-time LLVM bitcode/
  relocatable-object export. Fuse compatible host IR before LLVM19 O3. Validate
  source/variant, binding ABI, target/features and relocations before publication.
- Preserve original loader/program-write effects while selecting prepared native
  versions. Changed code/mappings revoke dependent bindings. No first-use decoding,
  emission or replay; preparation changes no guest state, memory, I/O or time.
- Optimize connected natural CFG/SSA without instruction-count caps. Keep values
  live, RAM direct and arithmetic inline; preserve aliases and every observable
  fault, event, ordered effect and partial execution boundary.
- Validate invariant layout/rights/alias/initialization facts during AOT/binding.
  Release omits optional diagnostic/sanitizer checks and redundant proved checks;
  Debug instruments our generated code. Required dynamic semantics remain equal.
- Allow substantial preparation without a fixed budget. Validated code artifacts
  keyed by code, compiler, target, build mode and bindings may be reused in
  development and deployment. Never cache instance pointers or device state.
- Preserve P/X/Y aliases, 24-bit words, 56-bit accumulators, raw partial fields,
  all numeric modes, parallel old operands, AGU, REP/DO and vector state. All
  56-bit arithmetic, rounding/saturation and required flags must remain inline.

## Qualification

Use authored fixtures in test/isa, test/core, test/jit and test/bench. Compare
complete independently calculated results with equivalent portable C++ compiled
by Clang19 O3 for the same target, aiming at throughput parity. Prevent dead-code
elimination; report actual lowering/coverage, cold and warm timing, native size,
API costs and every slow case. Keep ASM/SIMD comparisons separate; >=70% matched
host-ASM is a reference, not an instrument gate.

Prioritize functionality/performance with strict builds and focused independent
Release checks. Complete own-code/generated-JIT ASan/UBSan and separate concurrent
TSan qualification at the goal; run earlier for concrete risks. Use packaged
foreign dependencies, never foreign source builds or sanitizer instrumentation.
No GitHub workflows. Validate each native host without damaging the other.

The Blofeld consumer targets >=10x sustained original offline audio, >=4x
intermediate, and physical key-to-analog latency 2-5 ms/max 10 ms. Microbenchmarks
do not establish that outcome. See [doc/architecture.md](doc/architecture.md);
retain necessary primary references and provenance, not development narratives.
