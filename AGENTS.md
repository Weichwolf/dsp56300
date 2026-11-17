# dsp56300

Goal: OBJECTIVE.md. Target design: doc/architecture.md. Both serve one outcome:
the complete original Blofeld instrument in blofeld-emu at >=4x offline in every
situation on the i5 reference hosts (>=10x as direction on faster hosts) and
2-5 ms live latency. Decide everything else yourself.

## Fixed limits

- Follow doc/core-contract.md, the contract shared by all cores. It wins over
  this file and doc/architecture.md; change it only together with the other cores.
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
code in development, and the order of work are your choice. Change an interface
when it obstructs the instrument, but keep the core generic: it must stay usable
for other audio hardware built on a DSP56300. No Blofeld addresses, firmware
hashes, device models or instrument-specific special cases in this repository.
The consumer supplies all knowledge about the executed code (code inventory,
entries, variants, memory layout, devices, timing) through the public interface.

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

## Branches

- `main` is the only long-lived branch, locally and on the remote. The remote
  holds `main`; no review or backup branches, no tags used as backups. One
  exception: to hand unpublished work to another developer or host, push the
  step's work branch (`work/<issue>-<step>`) and name it in the issue. Its
  owner deletes it on the remote when the step is published or dropped. Code
  is handed over as a branch, never as a patch pasted into an issue.
- Local `main` equals the remote `main`. Fast-forward it (`git fetch --prune`,
  then `git merge --ff-only`) before starting work, before any measurement that
  names a published commit, and before every push. Nothing is committed on
  local `main` that is not pushed in the same step.
- Unpublished work lives on at most one work branch per open step, named
  `work/<issue>-<step>` (for example `work/37-r1`), rebased on the remote
  `main`, without merge commits. Publishing is: fast-forward `main` to it,
  push, delete the work branch, all in one step.
- A measurement baseline may be a detached worktree at a published commit. It
  is removed when the result is posted.
- A branch, worktree or stash is orphaned when its step is published, cancelled
  or superseded, or when no open issue step names it. Orphans are deleted, not
  archived: history on `main` and the issue record are the archive. A stash is
  never storage.
- At every publication `git branch` shows `main` plus the active work branches
  and `git worktree list` shows nothing else. No force pushes to `main`.
