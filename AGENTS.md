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
  increment.

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
- Commit reviewed increments. Validate your native host without breaking the
  other. Record the exact core revision in consumer changes.
- Use GitHub issues in this repository for concrete remaining work. Keep closed
  issues with useful technical evidence and write short, result-first comments.
- Sanitizer qualification (ASan/UBSan, separate TSan) of own and generated code
  happens at the goal, earlier only for a concrete risk.

## Branches

- Each host or agent may work on one `{hostname}/{issueid}_{issuename}` branch
  with one open issue at a time. The owner creates and merges the pull request
  with `gh` after required checks and approvals pass.
- `main` is the only long-lived branch. Keep local `main` synchronized with
  `origin/main` by fetching and fast-forwarding before work, before measuring
  a published revision and after a PR merge. Never force-push `main`.
- Remove work branches when merged, cancelled or superseded. Remove detached
  measurement worktrees when the result is posted.
