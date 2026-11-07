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

The `strict` preset enables ASan/UBSan.

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

## Benchmarks

```sh
cmake --build --preset release --target bench
```

The command runs continuous FIR, IIR, Hann/FFT/power and nonlinear feedback banks,
sequentially on one execution thread. Each stream retains state across packets:
one input second includes first-use compilation, followed by one warm input second.
Select scenarios with the CMake cache variable `DSP56300_BENCH_FILTER` (a regex).

Each case reports `hardware_factor` (target >=1) and `jit_to_host_percent` (target
>=70), plus worst cold/warm packets, deadline misses and actual code publications.
The conservative hardware model uses declared ISA base cycles at 275 MHz, without
pipeline or memory stalls; it is an assumption, not a calibrated Blofeld clock.
The separate `realtime_factor` uses the stream's 48 kHz input duration. Release
bench fails on any cold/warm packet deadline miss or modeled hardware factor <1;
`DSP56300_BENCH_ENFORCE_DEADLINE=OFF` selects reporting only.

Host references use the same fixed-point inputs, rounding, saturation, retained
history and memory effects, with independently generated expectations. FIR uses
checked scalar/AVX2 assembly and verifies complete registers and memory. IIR and
nonlinear banks use inline scalar MAC assembly with optimized native scheduling;
spectral processing compares compiler-generated scalar/vector code with handwritten
ASM windowing, FFT butterflies, output permutation and power/ring-buffer kernels.
The latter three are numerical algorithm baselines and omit architectural
register/flag bookkeeping;
the output identifies that stricter comparison scope and the reference implementation.
The fastest available host variant is used, including the additional `host_asm_s`
reference where provided; `jit_to_asm_host_percent` reports it separately.
The percentage is
`100 * host_time / jit_time`. The scalar-only percentage is reported separately.
JIT results always verify complete registers and all P/X/Y memory.
Initialization and final verification are outside continuous timing on both sides.
Short `bench-fir-host` calls remain overhead diagnostics, not sustained qualification.

[License](LICENSE)
