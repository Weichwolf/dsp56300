# dsp56300

Goal: OBJECTIVE.md. Target design: doc/architecture.md. Both serve one outcome:
the complete original Blofeld instrument in blofeld-emu at >=10x offline
(>=4x intermediate) and 2-5 ms live latency. Decide everything else yourself.

## Fixed limits

- Preserve DSP56300 semantics exactly: P/X/Y spaces and aliases, 24-bit words, 56-bit accumulators, rounding/saturation/flags, AGU, REP/DO and IRQs.
- Unsupported or unprepared input fails explicitly with identity, address and
  reason. Never change a correct test to hide a defect.
- One native path: LLVM 19 AOT. No production interpreter and no second code
  generator. Delete what the target does not need; no archives or compatibility
  layers. Git keeps history.
- Manufacturer inputs and private analysis stay untracked in build/local/,
  generated artifacts in build/. GPL-3.0-only with SPDX; keep dependency notices;
  no GPL-2.0-only octa-capture code. Run as a regular user. No GitHub workflows.

## How work is judged

- By whole-instrument throughput with bit-identical original PCM on the reference
  workloads, measured by the developers. Component numbers are supporting
  evidence, not results.
- Less code is better. Report the src/ and test/ line delta with each published
  increment; see the reduction plan in Weichwolf/blofeld-src#31.

## Freedom

APIs, state layouts, LLVM composition, tools, test strategy, caching of validated
code in development, and the order of work are your choice. Repository
boundaries are organizational. Change an interface when it obstructs the
instrument.

## Working together

- German with the user; English code, comments, docs and commits. C++23, LLVM 19,
  CMake/Ninja, warnings as errors, clang-format, clang-tidy clean.
- Commit reviewed increments and push them to origin/main only. No force pushes
  or backup branches. Validate your native host (Linux x86-64 or macOS ARM64)
  without breaking the other; coordinate consumer pins in commits.
- Coordinate in private Weichwolf/blofeld-src issues. Comments starting with
  [Lead] come from the technical lead and set priorities: #30 experiments,
  #31 reduction, #23 whole-instrument performance, #28 DSP AOT and backend,
  #25 arithmetic/filter/LFO, #26 MIDI/SysEx, #27 MCU IRQ/display/USB.
  Write short, result-first comments: table, what changed, what is open.
- Sanitizer qualification (ASan/UBSan, separate TSan) of own and generated code
  happens at the goal, earlier only for a concrete risk.
