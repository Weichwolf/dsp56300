# dsp56300

Independent DSP56300 native translator. C++23, Linux x86-64/AVX2 and macOS ARM64,
GPL-3.0-only. [OBJECTIVE.md](OBJECTIVE.md) defines acceptance;
[doc/architecture.md](doc/architecture.md) defines the current target.

## Build

Use packaged LLVM19/Clang with development headers/shared library, llvm-extract,
llvm-link, CMake, Ninja, clang-format and clang-tidy. Linux uses llvm-19-dev;
macOS uses brew install llvm@19 llvm ninja ccache. No foreign source builds.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
cmake --build --preset release --target bench
```

Release selects optimized code without sanitizers. strict selects Debug with
ASan/UBSan; thread selects separate TSan. Preserve strict warnings/linking,
clang-tidy and formatting. Instrument our code/generated code, never packaged
foreign libraries. On current macOS, keep the LLVM19 compiler and select the
packaged current Clang sanitizer runtime:

```sh
export LLVM19_RESOURCE_DIR="$("$(brew --prefix llvm)/bin/clang" -print-resource-dir)"
cmake --preset strict
```

Export the built-in ISA/retirement input with the `dsp56300-semantic-ir` target
at build/release/semantic/dsp56300.bc. Consumer composition specializes this
shared semantic input before final optimization.

## Use

```cmake
add_subdirectory(path/to/dsp56300)
target_link_libraries(your_app PRIVATE dsp56300::core)
```

```cpp
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>

#include <algorithm>
#include <array>

int main() {
  std::array<dsp56300::Word, 4096> ram{};
  constexpr std::array<dsp56300::Word, 2> program{0x000000, 0x000087}; // NOP; STOP
  std::ranges::copy(program, ram.begin());
  dsp56300::Core core(dsp56300::Memory::unified(ram));
  const std::array code{dsp56300::CodeSpan{.address = 0, .words = program}};
  const std::array entries{dsp56300::Word{0}};
  const std::array versions{
      dsp56300::CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  const auto image = dsp56300::Core::compile({.variants = versions}, {});
  core.install(image, 0);
  const auto result = core.run();
  return result.fault.kind == dsp56300::FaultKind::none ? 0 : 1;
}
```

Memory belongs to the consumer and must outlive Core. One owner executes a core.
Use public Io for devices and `run_until()` for an absolute local cycle deadline;
the library does not schedule an instrument or its audio.

Declare complete code streams, valid entries, source variants, memory layouts
and vector/priority contracts. Preserve logical P/X/Y aliases and captured vector
words. Declared cross-version transfers yield for the exact prepared destination;
reset cancels a pending handoff.

`Core::compile` lowers, materializes and binds a shareable `NativeImage`.
`Core::install` selects a variant on the stopped owner, validates source/layout
and retains shared executable ownership without changing architecture or time.
Code/mapping changes require matching stopped installation. Unknown entries,
undeclared events and unprepared cache/debug forms fail explicitly. Ordinary
control flow remains in the native image; no first-use decoding or emission.

For joint native composition, `Core::lower` returns owned pre-O3 `SemanticImage`
bitcode, exact guest/event exports and identity. Link both cores and eligible
host implementations before final optimization, resolve every export, then use
`Core::bind` with the exact component symbol map and shared native owner.
Public state/progress/yield definitions join eligible IR; consumers never copy
private layouts or invoke private entry ABIs. Count shared native text once.

Complete known code/variants before execution. Development runs full AOT;
deployment may reuse validated relocatable artifacts, never device snapshots.
Startup and build-time translation share one semantic pipeline. Release keeps
arithmetic inline and removes proved checks; Debug adds diagnostic instrumentation.
Both retain exact dynamic memory/device/fault behavior. Detailed preparation,
binding and invalidation contracts belong in the target architecture.

## Benchmarks and references

Use independently authored fixtures and equivalent Clang19 O3 C++ references
under the same CPU/features and exact numeric/state contract. Report preparation,
coverage, cold/warm time, code size and API costs separately; verify complete
results and prevent dead-code elimination. FIR/IIR/nonlinear/spectral host references and
ASM/SIMD ratios are separate comparisons. JIT/host percent is
100 * host_time / jit_time. Select cases with `DSP56300_BENCH_FILTER`; fixtures live
in test/bench/. Model clock ratios are not measured hardware speed, and isolated
core results do not qualify whole-instrument throughput.

Primary references are indexed from [doc/architecture.md](doc/architecture.md).
[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
