# dsp56300

- Build an independent complete DSP56300 native JIT. Keep application firmware,
  device clocks, peripherals and scheduling in consumers; no foreign core.
- Expose only core.hpp and memory.hpp, aligned with the other core. Core owns
  readable state; consumers own memory. One owner executes each core.
- Production is JIT-only. Keep all implemented ISA semantics. Compile natural
  control-flow regions once, retain immutable code and link native successors.
  No instruction-count translation caps, checkpoints or public effect tickets.
- Lower ordinary ISA natively; distinguish actual I/O calls from ISA fallback
  gaps. Keep unsupported cases visible. Fix generic integration defects here.
- Keep code small, registers live and RAM direct. SIMD needs measured benefit.
  Representative warm >=70% matched host-ASM throughput is a reference, not an
  instrument-integration gate. Report cold latency and every slow scenario.
- Test programs are independently authored and use test/support/isa.hpp.

## Working rules

- Speak German with the user; code, comments, documentation and commits are English.
- C++23, LLVM 19, CMake/Ninja; strict warnings/linking, clang-format, zero-finding
  clang-tidy, functions <=120 lines. Never suppress or weaken checks.
- Prioritize complete instrument functionality and >=4x sustained MIDI-to-WAV
  throughput. Use Release and focused independent checks while iterating. Strict
  build/test rules cover our code; use packaged foreign libraries, never rebuild
  LLVM/compiler-rt or instrument foreign code. Defer routine sanitizers until the
  instrument goal; earlier ASan/UBSan/TSan runs need a concrete defect/risk reason.
  Sanitizer coverage includes our generated JIT code. No GitHub workflows.
- Blofeld firmware probes stop within 10 host seconds and fail unless playable.
  Until real time, no long firmware runs or unchanged failed-probe repetitions.
- Telemetry is runtime-selectable and off by default; no hot-path statistics.
- README describes implemented use; OBJECTIVE defines acceptance; doc/ contains
  one target architecture and primary references. No diaries or result archives.
- Keep firmware, samples, manuals, disassemblies and private analysis untracked.
  Reference inputs: ignored build/local/; artifacts: build/. Preserve inputs and
  source/size/hash manifests. Original source is GPL-3.0-only with SPDX notices.
- Keep canonical main current. Commit reviewed increments; push only configured
  upstream/main (blofeld-emu) or origin/main (cores). No backup branches or
  --all/--mirror; preserve unrelated work and coordinate consumer dependency pins.
- Issues belong only in private Weichwolf/blofeld-src; no routine status spam.
  Consult local implementations such as Gearmulator for known register maps,
  protocols and algorithms; write our implementation independently. Keep
  reference checkouts unchanged and preserve existing dependency notices.
  Do not import GPL-2.0-only octa-capture code. Run as a regular user.
- Validate ARM64 on macOS and x86-64 on Linux; communicate revisions in commits.
