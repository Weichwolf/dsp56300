Independent DSP56300 core

This repository develops an independent C++23 DSP56300 CPU emulator for x86-64,
with Blofeld audio programs as the primary optimization workload. It contains
no dependency on another DSP emulator, no DSP56371 peripheral implementation
and no manufacturer program assets.

Current state: early implementation, not a complete DSP56300 emulator. The
reference and cached paths share an instruction executor and have independent
numeric expectations in tests. The instruction cache stores decoded words in
pages allocated on demand. A/B use a left-aligned 56-bit representation so
ordinary 64-bit wrapping arithmetic discards bits beyond DSP precision.
An independent Linux x86-64 JIT emits selected scalar arithmetic and uses guarded
general helpers for other implemented forms. Select Options.backend=Backend::jit;
the cached interpreter remains the default. See doc/jit.txt for supported native
operations, invalidation, execution budgets and instrumentation boundaries.
Full ISA/mode/timing qualification and Blofeld performance measurements remain
open; see doc/coverage.txt and the complete objective in doc/objective.txt.

Build and check (LLVM 19, CMake >=3.25, Ninja, clang-format-19, clang-tidy-19):

  cmake --preset strict
  cmake --build --preset strict
  ctest --preset strict
  cmake --build build/strict --target format-check

Use the release preset for performance work. No fuzzing runs automatically.
Project setup and strict policies derive from blofeld-emu; see THIRD_PARTY.txt.

Integration: link dsp56300::core and include <dsp56300/core.hpp>. Provide mutable
P/X/Y spans and an optional Io implementation. DSP data words occupy uint32_t
elements; architectural A/B values in Registers are raw 56-bit quantities.
The host retains buffer ownership and must use Core::load/write for program
mutation. Core is owned by one execution thread; callbacks run synchronously.
Unimplemented instructions/modes, unmapped accesses and rejected I/O stop the
core with explicit context. Faulted execution requires reset. Register views
are not complete checkpoints of REP/interrupt continuation.

The host supplies derivative interrupt arbitration and admits an interrupt at
a stopped boundary. Hardware clocks, memory aliases, boot ROM, DMA, timers,
serial interfaces, GPIO and board behavior remain outside this library.
estimated_cycles sums instruction base costs only; pipeline interlocks,
interrupt-entry overhead and external wait states remain unqualified.

Primary instruction/architecture references:
  https://www.nxp.com/docs/en/reference-manual/DSP56300FM.pdf
  https://sites.ecse.rpi.edu/courses/CStudio/dsp56303/56300%20Instruction%20Set.pdf
Private reference downloads and Blofeld source extracts belong under ignored
doc/local/ or build/. They are excluded from repository commits.
