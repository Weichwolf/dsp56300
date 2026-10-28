# Scope and objective

- Speak German with the user. Write code, comments, documentation and commits in English.
- Build only an independent DSP56300 emulator library for general applications.
- Optimize for all usual DSP56300 application domains, including audio,
  image/video, communications, measurement and control. Check performance
  tradeoffs across independently authored workloads from multiple domains.
- Preserve the complete user objective in doc/objective.txt and audit doc/coverage.txt against it.
- Keep neighboring application and firmware repositories read-only. Do not copy
  board emulation, firmware or application programs here.
- Implement the DSP independently. No dependency on dsp56kEmu in build, API, tests or runtime.
- Keep CPU/JIT entry points in src/core.cpp and src/jit.cpp, with implementation
  helpers in src/core/ and src/jit/. Host memory lives in src/core/memory.cpp.
- Original code is GPL-3.0-only; preserve dependency and reused-source notices.
- Manufacturer programs, manual copies, source extracts and private analysis stay in ignored doc/local/ or build/.

# Implementation and validation

- Use C++23, CMake/Ninja and LLVM 19 with strict warning, hardening, clang-format
  and clang-tidy policies. Keep functions at most 120 lines.
- Instrument project code and native dependencies with ASan/UBSan in strict builds.
- Do not weaken checks or add NOLINT markers to suppress findings. Analyze upstream separately.
- Verify architectural state, fixed-point precision, transfer boundaries, flags, AGU,
  loops and interrupt behavior. Specialized paths require explicit guards and a general fallback.
- Use reproducible single-instance benchmarks with independently authored DSP
  programs covering audio, image/video and communications patterns. Do not use
  manufacturer or application firmware in tests or benchmarks.
  Distinguish arithmetic experiments, emulator execution and whole-device performance.
- Keep host memory and event/I/O contracts explicit. Do not silently fabricate peripheral results.
- Use small state/dataflow tables for optimization design. Choose host register
  placement per region/pattern; retain complete checkpoints and guarded fallback.
- Share correctness fixtures with the bench CMake target and support all or
  selected fixture sets. Keep measurement scope and skipped case counts explicit.
- Use core concepts to compare implementations in one build. Parameterize host
  SIMD lane width separately from vector width; keep DSP precision exact and
  dispatch only to implemented, host-supported instruction sets.
- The user canceled the hardware virtualization path. Build and optimize direct
  native JIT execution; do not add a KVM/VMX backend, tests or permissions setup.
- Keep local account names out of tracked instructions, code and documentation.
- Do not run fuzz tests unless the user explicitly requests them.
- Commit verified increments locally. No publication destination is authorized.
