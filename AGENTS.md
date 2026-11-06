# dsp56300

- Build an independent, general DSP56300 native JIT library for audio, image/video,
  communications, measurement and control. Complete ISA and exact fixed-point,
  flags, AGU, parallel transfers, loops and interrupts remain open. See OBJECTIVE.md.
- Keep board clocks, derivative peripherals and application firmware in consumers.
  No foreign DSP execution dependency; use independently authored test programs.
- Use Core(Memory, Io*), typed memory and explicit ownership, aliases,
  invalidation and event limits. The core has one execution owner.
- Automatically choose the fastest qualified host path. Fix slow instruction and
  transfer helpers here; JIT fallback gaps remain visible until native lowering
  preserves all effects and precise exits. I/O callbacks are separate from ISA
  interpretation. Do not add a hardware virtualization backend.
- Keep Core/JIT entry points in src/core.cpp and src/jit.cpp; use block_parse,
  block_emit, run, arena and code under src/jit/ as in cpu12. Keep x86 lowering
  in src/jit/x64/ and borrowed memory in memory.hpp.
- Publish reviewed increments only to configured origin/main. Keep neighboring
  application sources read-only; coordinate core revisions with consumer pins.

- Expose only core.hpp and memory.hpp under include/. Keep ISA encodings in
  test/support/isa.hpp and production decoding/transfer helpers private; production code must not include test headers.
- Keep the same minimal public Core/Memory/Io API shape in both cores.
  Core owns its state; optional read-only state inspection is outside normal use.
  Remove checkpoint/restore APIs and their implementation completely. Preserve
  ISA, wait and interrupt behavior; state inspection is read-only. Expose no
  pipeline tickets in the normal API.
- Production execution is JIT-only. Future x86/SSE/AVX and ARM/NEON host emitters
  implement the same host concept. Remove interpreter/reference/cached backend
  selection; preserve independent ISA tests and all implemented instructions.

- Translate complete logical blocks/functions independently of execution grants.
  No artificial instruction-count translation caps or budget-specific fragments.
  Keep generated code immutable and retained for the entire Core lifetime; no
  eviction/recompilation. The consumer creates a new Core for a new program.

## Performance priority

- Before further integration or optimization in blofeld-emu, bring both cores
  close to matched host-ASM throughput on representative warm workloads. Use the
  same inputs, relevant ISA semantics and verified outputs; trivial-only cases
  or weak host baselines do not establish this gate. Report every shortfall.

- Optimize cpu12 first until every representative warm fixture reaches at least
  70% of matched optimized host ASM on realistic sustained scenarios, then
  optimize and simplify dsp56300 likewise. Tiny reset/run/inspect microfixtures
  diagnose overhead and do not establish or block this target. Require real-time
  processing even when a packet discovers and compiles new code; report the
  assumed device rate, worst packet and missed deadlines, not just averages.
- While throughput is low, favor short optimized experiments and focused result
  checks. Increase validation breadth with performance progress; preserve strict
  check definitions and independent ISA coverage.
- Keep only code needed for correct behavior or demonstrated performance. Remove
  unnecessary DSP implementation/API during its optimization stage, preserving
  implemented ISA and the independent tests required to qualify it.

## Working rules

- Speak German with the user; write code, comments, documentation and commits in English.
- Prefer simple native execution, direct memory and large bounded runs. Keep device
  effects and event boundaries explicit; avoid per-clock work and unnecessary copies.
  Telemetry is runtime-selectable and off by default; keep its work out of hot paths.
- Use C++23, LLVM 19 and CMake/Ninja. Keep strict compiler/linker flags, warnings
  as errors, clang-format, clang-tidy with zero findings and functions <=120 lines.
  Never weaken checks or hide findings with suppressions or diagnostic baselines.
- Release is the performance build. Preserve ASan/UBSan for project code and native
  dependencies, and separate TSan for concurrent code; preserve dependency notices.
- Run affected cpu12/dsp56300 correctness tests for core changes, including
  independent instruction/state expectations and appropriate sanitizer coverage.
  Core tests are not subject to the instrument's real-time milestone restriction.
  Avoid duplicate or full-suite runs after small changes; report gaps and scope.
- Until the instrument reaches real time, suspend long Blofeld firmware runs.
  Startup/throughput probes stop after at most 10 host seconds and fail unless
  playable; never extend or automatically repeat a failed probe. Keep compilation,
  formatting and static analysis active.
- Keep doc/ for one target architecture and important primary references only.
  Requirements belong in OBJECTIVE.md and test/; build metadata belongs in cmake/.
  No activity diaries or historical result manifests. Issue communication belongs
  in private Weichwolf/blofeld-src, with concrete consolidated problems only.
- Keep firmware, samples, manuals, disassemblies and private analysis untracked.
  Preserve source URLs, sizes and hashes in reference manifests; never publish assets.
  Local reference inputs live under ignored build/local/; run artifacts belong
  under ignored build/. Preserve inputs when cleaning generated files.
- Original code is GPL-3.0-only; add matching SPDX identifiers to new source.
- Commit reviewed, verified increments; push only to the configured project remote
  on main. Never publish backup branches or use --all/--mirror. Coordinate core
  revisions with consumer dependency pins; preserve unrelated work.
