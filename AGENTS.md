# Scope and objective

- Speak German with the user. Write code, comments, documentation and commits in English.
- Build only an independent DSP56300 emulator kernel for later integration into blofeld-emu.
- Preserve the complete user objective in doc/objective.txt and audit doc/coverage.txt against it.
- Keep ~/Git/blofeld-emu and ~/Git/blofeld-src read-only. Do not copy board emulation here.
- Implement the DSP independently. No dependency on dsp56kEmu in build, API, tests or runtime.
- Use src/Core/core.cpp rather than Kernel/kernel.cpp for the emulator implementation.
- Original code is GPL-3.0-only; preserve dependency and reused-source notices.
- Manufacturer programs, manual copies, source extracts and private analysis stay in ignored doc/local/ or build/.

# Implementation and validation

- Use C++23, CMake/Ninja and LLVM 19, matching blofeld-emu's strict warning, hardening,
  clang-format and clang-tidy policies. Keep functions at most 120 lines.
- Instrument project code and native dependencies with ASan/UBSan in strict builds.
- Do not weaken checks or add NOLINT markers to suppress findings. Analyze upstream separately.
- Verify architectural state, fixed-point precision, transfer boundaries, flags, AGU,
  loops and interrupt behavior. Specialized paths require explicit guards and a general fallback.
- Use reproducible single-instance benchmarks, including real source-built Blofeld routines.
  Distinguish arithmetic experiments, emulator execution and whole-device performance.
- Keep host memory and event/I/O contracts explicit. Do not silently fabricate peripheral results.
- Do not run fuzz tests unless the user explicitly requests them.
- Commit verified increments. No publication destination is authorized or configured.
