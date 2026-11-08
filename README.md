# dsp56300

Independent DSP56300 native JIT. C++23, Linux x86-64/AVX2,
macOS ARM64, GPL-3.0-only. See OBJECTIVE.md for complete-ISA acceptance.

## Build

LLVM 19, CMake, Ninja, clang-format-19 and clang-tidy-19 are required.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
cmake --build --preset release --target bench
```

`strict` enables ASan/UBSan; `thread` selects separate TSan. Ccache is used when
available. On macOS use `brew install llvm@19 ninja ccache`. When required by the
macOS version, set `LLVM19_RESOURCE_DIR` from `python3 tools/darwin_runtime.py`
before configuring strict; the tool prepares the patched LLVM 19 sanitizer runtime.

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

Benchmarks check independent results and report cold/warm timing and matched
JIT/host ratios (`100 * host_time / jit_time`). Hardware factors use stated models,
not measured device speed. Select cases with DSP56300_BENCH_FILTER;
fixtures are in test/bench/. See doc/architecture.md and test/ for details.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
