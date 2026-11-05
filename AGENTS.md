# dsp56300

- Build an independent, general DSP56300 native JIT library for audio, image/video,
  communications, measurement and control. Complete ISA and exact fixed-point,
  flags, AGU, parallel transfers, loops and interrupts remain open. See OBJECTIVE.md.
- Keep board clocks, derivative peripherals and application firmware in consumers.
  No foreign DSP execution dependency; use independently authored test programs.
- Use Core(Memory, Io*, Options), typed memory and explicit ownership, aliases,
  invalidation, checkpoints and event limits. The core has one execution owner.
- Automatically choose the fastest qualified host path. Fix slow instruction and
  transfer helpers here; JIT fallback gaps remain visible until native lowering
  preserves all effects and precise exits. I/O callbacks are separate from ISA
  interpretation. Do not add a hardware virtualization backend.
- Keep core/JIT entry points in src/core.cpp and src/jit.cpp, helpers in src/core/
  and src/jit/, native emission in src/jit/x64/, memory in src/core/memory.cpp.
- Publish reviewed increments only to configured origin/main. Keep neighboring
  application sources read-only; coordinate core revisions with consumer pins.

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
- Until the instrument reaches real time, suspend component, functional, regression
  and diagnostic execution tests. Only short optimized startup/throughput probes
  may establish that milestone: stop after at most 10 host seconds, fail unless
  playable, and never extend or automatically repeat a failed probe. Compilation,
  formatting and static analysis remain active. Run fuzzing only on explicit request.
- Afterwards validate affected behavior against independent expectations; avoid
  duplicate or full-suite runs after small changes. Report gaps and measured scope.
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
