Independent DSP56300 core

This repository develops an independent C++23 DSP56300 CPU emulator for x86-64,
targeting efficient execution across all usual DSP56300 applications: audio,
image/video, communications, measurement and control. Validation uses
independently authored general DSP application workloads. It contains
no dependency on another DSP emulator, no DSP56371 peripheral implementation
and no manufacturer program assets.

Current state: early implementation, not a complete DSP56300 emulator. The
reference and cached paths share an instruction executor and have independent
numeric expectations in tests. The host decode cache stores decoded words in
pages allocated on demand. Separate architectural DSP cache storage implements
the seven cache-control instructions; see doc/program-cache.txt for the external
P mapping/geometry options, CE-enabled general fallback and qualification gaps. A/B use a left-aligned 56-bit representation so
ordinary 64-bit wrapping arithmetic discards bits beyond DSP precision.
An independent Linux x86-64 JIT emits selected scalar arithmetic and uses guarded
general helpers for other implemented forms. Select Options.backend=Backend::jit;
the cached interpreter remains the default. See doc/jit.txt for supported native
operations, invalidation, execution budgets and instrumentation boundaries.
Full ISA/mode/timing qualification and representative performance measurements remain
open; see doc/coverage.txt and the complete objective in doc/objective.txt.

Source layout:
  src/core.cpp    CPU API and execution orchestration
  src/jit.cpp     native translation cache and executable storage
  src/core/       architectural state, instruction execution and host memory
  src/jit/        dataflow, transfer plans and general-execution bridges
  src/jit/x64/    native x64 emission and instruction encoding
  include/dsp56300/  public CPU interface
  test/           independent fixture generators, host tests and benchmarks

Build and check (LLVM 19, CMake >=3.25, Ninja, clang-format-19, clang-tidy-19):

  cmake --preset strict
  cmake --build --preset strict
  ctest --preset strict
  cmake --build build/strict --target format-check

Use the release preset for performance work. No fuzzing runs automatically.
Project setup and strict policies retain their source attribution in THIRD_PARTY.txt.

Benchmarks run continuous one-second DSP workloads with independent binary fixtures:

  cmake --preset release
  cmake --build --preset release --target bench
  ctest --test-dir build/release/bench -R '^(fir|iir)-stream$' --verbose

One input second warms the same DSP before the measured second, without host
state resets. Current continuous scenarios are an eleven-channel 507-tap FIR
and a 92-channel IIR bank/mix; spectral analysis remains pending. Select all or a numeric case with
DSP56300_BENCH_CASES, workloads with DSP56300_BENCH_FILTER, and implementations
with DSP56300_BENCH_BACKEND. Release bench fails when a one-second package exceeds
one host second. See doc/bench.txt for CPU, latency, correctness and scope.

CoreImplementation and typed BackendCore<Backend::...> adapters allow generic
consumers and the same fixture runner to compare reference/cached/JIT in one
build. Host SIMD lane and vector widths are separate constrained parameters;
see doc/backend-contracts.txt for API examples, supported operations and limits.
Guarded MAC-loop scalar/AVX2 batching and proven reduction windows are described
in doc/mac-batching.txt and doc/mac-reductions.txt. Compilation-time entry
planning is described in doc/mac-plan-compilation.txt.

Integration: link dsp56300::core and include <dsp56300/core.hpp>. Provide mutable
P/X/Y spans and an optional Io implementation. DSP data words occupy uint32_t
elements; architectural A/B values in Registers are raw 56-bit quantities.
The host retains buffer ownership and must use Core::load/write for program
mutation. Core is owned by one execution thread; callbacks run synchronously.
Unimplemented instructions/modes, unmapped accesses and rejected I/O stop the
core with explicit context. Faulted execution requires reset. Register views
are not complete checkpoints of REP/interrupt continuation, DSP cache or sleep state.
See doc/io.txt for synchronous word/atomic-bit callbacks and mutation boundaries.
run returns an explicit waiting/stopped state for WAIT/STOP. Guest RESET needs
Io::reset_peripherals acknowledgement; the default is an explicit event fault.
See doc/processing.txt for wake, callback, clock and qualification contracts.
DEBUG/DEBUGcc support a functional halt or IME-selected debug interrupt. Host
request/resume and IME/SWO inspection are described in doc/debugging.txt; this
is not a complete OnCE/JTAG debugger or a full ISA qualification claim.

The host supplies derivative interrupt arbitration and admits an interrupt at
a stopped boundary. Hardware clocks, memory aliases, boot ROM, DMA, timers,
serial interfaces, GPIO and board behavior remain outside this library.
estimated_cycles sums instruction base costs only; pipeline interlocks,
interrupt-entry overhead and external wait states remain unqualified.

Primary instruction/architecture references:
  https://www.nxp.com/docs/en/reference-manual/DSP56300FM.pdf
  https://sites.ecse.rpi.edu/courses/CStudio/dsp56303/56300%20Instruction%20Set.pdf
Reference downloads belong under ignored doc/local/ or build/. Application and
manufacturer programs are not used by tests or benchmarks. See
doc/application-validation.txt for the independently authored workload plan.
