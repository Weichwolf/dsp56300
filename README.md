# dsp56300

Independent DSP56300 native JIT. C++23, Linux x86-64/AVX2,
macOS ARM64, GPL-3.0-only. See OBJECTIVE.md for complete-ISA acceptance.

## Build

LLVM 19.1 development headers/shared library, the LLVM 19 toolchain, CMake, Ninja,
clang-format-19 and clang-tidy-19 are required.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
cmake --build --preset release --target bench
```

`strict` enables ASan/UBSan; `thread` selects separate TSan. Ccache is used when
available. Use the packaged LLVM 19 toolchain; on macOS install it with
`brew install llvm@19 ninja ccache`. LLVM and compiler-rt are not built from source.
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
  const auto result = core.run();
  return result.fault.kind == dsp56300::FaultKind::none ? 0 : 1;
}
```

Memory belongs to the consumer and must outlive Core. One thread owns execution;
state inspection is optional. Use Io for device effects and run_until() for an
absolute local cycle deadline. Core does not schedule an instrument or its audio.
Unsupported operations fail explicitly.

`Core::prepare(entries)` follows established code entries without executing ISA or
device I/O. It installs LLVM O3 code for ordinary RAM-transfer DO bodies with
linear cursors; other operations retain their native execution path. Preparation
reports installed entries, unresolved coverage, newly emitted text and a
non-latching diagnostic. Code remains owned by Core; changed external mappings
require a new `Io::program_revision()` before preparation or execution.

Benchmarks check independent results and report cold/warm timing and matched
JIT/host ratios (`100 * host_time / jit_time`). Hardware factors use stated models,
not measured device speed. Select cases with DSP56300_BENCH_FILTER;
fixtures are in test/bench/. See doc/architecture.md and test/ for details.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
