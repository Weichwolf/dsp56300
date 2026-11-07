# dsp56300

Independent DSP56300 emulator with an x86-64 JIT.

C++23 · Linux x86-64 with AVX2 · GPL-3.0-only

## Build

Requires LLVM 19, CMake, Ninja, clang-format-19 and clang-tidy-19.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

On macOS, install the matching tools with `brew install llvm@19 ninja ccache`.
Presets discover the Homebrew LLVM 19 installation without replacing system tools.
ARM64 contains private instruction emitters and component tests; the public
`Core` reports an unsupported host on ARM64.

The `strict` preset enables ASan/UBSan. On macOS 26.4 and later, LLVM 19's
original sanitizer runtime deadlocks during dyld initialization. Build the same
19.1.7 runtime with upstream fixes for [allocation-free shadow checks](https://github.com/llvm/llvm-project/commit/6a89439423351be2d63e13e191acd4cb33e0aaff)
and [dyld lookup](https://github.com/llvm/llvm-project/commit/2e7d07a33725a82ecfc514e27f047ece3ff13d4c):

```sh
export LLVM19_RESOURCE_DIR="$(python3 tools/darwin_runtime.py)"
cmake --preset strict
cmake --build --preset strict
```

This keeps the LLVM 19 compiler and headers, verifies source identities and retains
all sanitizers. Sources and runtime artifacts remain under ignored `build/local/`.
A second core checkout can use the same `LLVM19_RESOURCE_DIR`.

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

Memory is borrowed and must outlive the core. One execution thread owns each
core. The example executes NOP and STOP; a consumer supplies its own program and
optional I/O. `run()` and `step()` execute native code; `state()` gives a read-only
snapshot. Unsupported instructions or modes report explicit faults.
`Memory::unified` explicitly aliases P/X/Y; `Memory::separate` supplies distinct
logical address spaces.

`run_until(base_cycle_deadline)` uses the same retained native blocks for an
absolute nominal ISA-cycle deadline. It completes the final instruction and
may return earlier for I/O yield, sleep or a fault. Peripheral clocks and bus
waits belong to the consumer; this does not estimate their elapsed hardware time.

## Benchmarks

```sh
cmake --build --preset release --target bench
```

The benchmarks run continuous FIR, IIR, spectral processing and nonlinear
feedback streams with retained state. Select scenarios with the CMake cache
variable `DSP56300_BENCH_FILTER`. Fixture definitions are in [test/bench](test/bench).

Reports include cold/warm packet times, deadline misses, code publications and
`jit_to_host_percent` (`100 * host_time / jit_time`). JIT results are checked
against independent register and memory expectations. Host comparisons use
fixed-point C++ and ASM references; the output identifies their semantic scope.

The conservative `hardware_factor` uses declared ISA base cycles at 275 MHz,
excluding pipeline and memory stalls. `realtime_factor` uses the streams' 48 kHz
input duration. Neither metric establishes Blofeld instrument throughput.
Release benchmarks enforce packet deadlines and a modeled hardware factor of
at least one; `DSP56300_BENCH_ENFORCE_DEADLINE=OFF` selects reporting only.

[License](LICENSE)
