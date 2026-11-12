# dsp56300

- Build an independent complete DSP56300 translator; follow OBJECTIVE.md and
  doc/architecture.md. Only core.hpp and memory.hpp are public. Core owns state;
  consumers own memory, devices, scheduling and firmware. One owner executes a core.
- Require complete known AOT/variants before execution. Use one semantic pipeline
  for startup installation and build-time IR/object export, with private LLVM
  handles, exact identities and validated relocations. No production interpreter,
  instruction-count caps, checkpoints, public backend selectors or effect tickets.
- Fuse eligible host IR before LLVM19 O3. Keep RAM direct, registers live and
  arithmetic inline. Validate proved invariants during preparation/binding;
  Debug instrumentation stays out of Release. Preserve dynamic effects/faults.
- Preserve every supported ISA form and exact state/time/IRQ semantics. Report
  missing lowering explicitly. Test independently authored programs through
  test/support/isa.hpp; never change correct tests to hide a defect.
- Target parity with equivalent Clang19 O3 C++; report coverage, preparation,
  API costs and every slow case. >=70% matched host-ASM is an additional reference.
  The instrument target is >=10x sustained offline, >=4x intermediate, with
  physical key-to-analog latency 2-5 ms/max 10 ms; core benchmarks do not qualify it.

## Work

- German with the user; English code/comments/docs/commits. C++23, LLVM19,
  CMake/Ninja, strict warnings/linking, clang-format, zero-finding clang-tidy,
  functions <=120 lines. Never suppress or weaken checks.
- Prioritize functionality/performance with focused independent Release checks.
  Defer routine own-code/generated-JIT ASan/UBSan and separate TSan until the goal;
  earlier runs need a concrete reason. Use packaged foreign dependencies without
  source builds or sanitizer instrumentation. No GitHub workflows.
- Purpose-specific finite firmware guards allow startup; stop promptly on faults.
  No fixed ten-second cap or unchanged failed repetition. Telemetry is selectable,
  off by default and absent from the normal hot path.
- README documents use, OBJECTIVE acceptance, doc/architecture.md the current
  target; code is the implementation. Preserve primary source/size/hash references
  and qualification evidence, not diaries, obsolete attempts or history archives.
  Git holds history. Manufacturer inputs/private analysis stay in ignored
  build/local/, generated artifacts in build/. Preserve GPL-3.0-only/SPDX and all
  dependency notices; no GPL-2.0-only octa-capture code or altered reference checkouts.
  Run as a regular user.
- Keep canonical main current. Commit reviewed increments and push every qualified
  core increment only to origin/main; no backup branches, force pushes,
  --all/--mirror or unrelated edits. Coordinate consumer pins in commits.
  Each developer validates their native macOS ARM64 or Linux x86-64 host.
- Coordinate only in private Weichwolf/blofeld-src: #23 instrument performance and
  platform ownership; #28 DSP AOT/entries/backend; #25 arithmetic/filter/LFO;
  #26 MIDI/SysEx; #27 MCU IRQ/display/USB ordering; #29 documentation cleanup.
  Use existing specialist issues and local implementation references; no status spam.
