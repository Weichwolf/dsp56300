# dsp56300

Independent DSP56300 native JIT. C++23, Linux x86-64/AVX2,
macOS ARM64, GPL-3.0-only. See OBJECTIVE.md for complete-ISA acceptance.

## Build

LLVM 19.1 development headers/shared library, the LLVM 19 toolchain, CMake, Ninja,
clang-format-19, clang-tidy-19 and Python 3 are required.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
cmake --build --preset release --target bench
```

The built-in ISA/retirement compiler input can also be exported:

```sh
cmake --build --preset release --target dsp56300-semantic-ir
```

This emits verified LLVM 19 bitcode at `build/release/semantic/dsp56300.bc`.
Consumer image compilation specializes that input and combines compatible host
bindings before O3. Templates and native helpers share one ISA implementation.
Requested ASan coverage is applied once to the final generated functions;
frontend UBSan checks remain in the compiler input.

`strict` enables ASan/UBSan; `thread` selects separate TSan. Ccache is used when
available. Use the packaged LLVM 19 toolchain; on macOS install it with
`brew install llvm@19 llvm ninja ccache`. The compiler and builtin headers remain
LLVM 19. For current macOS versions select the packaged current Clang sanitizer runtime:

```sh
export LLVM19_RESOURCE_DIR="$("$(brew --prefix llvm)/bin/clang" -print-resource-dir)"
cmake --preset strict
```

LLVM and compiler-rt are not built from source.
Sanitizers cover project code and explicitly instrumented generated JIT code,
not packaged third-party libraries.

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

For external native materialization, `Core::lower` returns an immutable
`SemanticImage` with owned pre-O3 bitcode, ABI exports and a component identity.
Link prepared components before final native O3, resolve every export, then pass
its exact `NativeSymbols` view to `Core::bind`. Bound images retain the supplied
shared code owner; RAM and device state remain instance-owned. Shared native
text size describes that owner once, rather than a sum across bound components.
`Core::compile` uses the same lower/materialize/bind path for one component.
`CompileOptions::callbacks` selects public Io roles and external symbols;
`lower` declares them without host definitions. Standalone `compile` accepts
their native-target LLVM bitcode through `CompileOptions::bindings` and validates
it before linking and final O3. Unselected roles retain normal Io behavior.

Fixed `CompileOptions::memory` layouts specialize native RAM paths after cold
installation checks. Backing addresses, contents and initialization bits stay
dynamic; actual remapping still requires a matching installation.

Memory belongs to the consumer and must outlive Core. One thread owns execution;
state inspection is optional. Use Io for device effects and run_until() for an
absolute local cycle deadline. Core does not schedule an instrument or its audio.
Unsupported operations fail explicitly. Installed images retain shared code
ownership and never fall back to ordinary decoding or emission. Code/mapping
changes require a matching installation; unknown entries and unprepared
cache/debug forms fail explicitly. Declare vector/priority pairs in CompileOptions
before compilation; captured vector words survive selection of prepared newer
resident code without replaying a peripheral action. Declare cross-version
entries with `CompileOptions::transitions`: a real transfer to an absent target
yields without a fault. Install exactly its declared destination after validating
the resident words; rejected installations retain the pending transfer. Interrupt
admission and captured execution wait for that installation. Explicit reset
cancels it. Ordinary control flow stays within the native image.

The [target architecture](doc/architecture.md) requires complete Ahead-of-Time
Binary Translation of consumer-owned executable images and known variants.
Prefer startup loading and complete preparation before execution. Support
build-time export of linkable LLVM bitcode or relocatable native objects through
the same semantic translation and optimization pipeline.
Development always runs complete AOT/optimization. Preserve validated code
artifacts for operational startup reuse, with exact code/binding/host identity
and safe relocation. Rebuild rejected artifacts before execution.

`Core::prepare(entries)` analyzes an immutable code/mapping view without guest
execution, real I/O or architectural state/time changes. Lift the complete known
executable program into semantic LLVM 19 IR, optimize connected functions and
loops with target-aware O3, and install code against validated bindings.
Fuse compatible consumer-owned host IR before optimization. Keep LLVM handles
private and original memory/device effects ordered.

Require complete translation before execution; report a gap with code identity,
address and reason. Distinguish total executable coverage from optimization
coverage. Include interrupt/indirect entries and known loader/template variants;
preserve original writes while selecting prepared native code. Keep mutable data
in borrowed host RAM and retain observable program words. Compilation and
validated artifact loading run on a stopped owner; code/cache identity includes
input hashes, binding ABI, compiler and host target/features.

Benchmarks check independent results and report cold/warm timing and matched
host ratios. FIR, IIR, nonlinear and spectral streams include verified C++ host
references compiled with Clang 19 `-O3`, reporting `host_cpp_o3_s` and
`jit_to_cpp_o3_percent` separately from retained ASM/SIMD comparisons and gates.
The C++ FIR reference preserves exact wrapping MACs and sticky overflow.
Require explicit LLVM O3 preparation and report its complete executable coverage
separately from warm timing and host/API costs.

JIT/host ratios use `100 * host_time / jit_time`. Hardware factors use stated models,
not measured device speed. Select cases with DSP56300_BENCH_FILTER;
fixtures are in test/bench/. See doc/architecture.md and test/ for details.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
