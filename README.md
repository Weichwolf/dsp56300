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
ARM64 executable storage, typed instruction emission, native ABI entry/context
routing, NEON MAC products and the scalar arithmetic emitter are implemented
and independently tested. Native arithmetic covers add/subtract with carry,
comparison including magnitude, shifted add/subtract, MAX/MAXM, signed
multiply/MAC, rounding, SM saturation, ALU TFR forms and
logical operations in parallel/immediate instruction forms against independent raw-width models.
Shift/rotate emission covers immediate and register counts, with precise native
invalid-count and numeric-mode fault exits. Control emission covers all conditions,
absolute/relative/effective jumps and calls, returns, stack faults, WAIT/STOP and
byte control operations. Effective addressing handles linear, reverse-carry,
modulo and multiple-wrap updates, including updates on untaken conditional jumps.
Conditional emission covers Tcc accumulator/address/dual transfers and all IFcc
ALU bytes, preserving optional SR updates and the different mode-fault ordering.
Independent native matrices cover every scalar ALU byte without a parallel transfer;
register/immediate parallel transfers and address updates preserve pre-transfer ALU
inputs and post-transfer partial accumulators. Native register-bus emission covers
partial/full accumulators, sticky transfer flags, R/N/M and control registers,
including SSH/SSL stack effects and precise bounds faults. LRA/LUA address calculations
and register/immediate MOVEC forms are implemented. Memory emission now covers ordinary
parallel X/Y moves, class-II moves, displaced/control/program/peripheral transfers,
direct RAM, guarded external windows and separate actual-bus callbacks. Native tests
check full-width RAM/I/O priority, initialized-word grants, callback-visible state,
same-instruction remapping and exact executable-alias exits. Long, paired X/Y and
class-I mixed transfers now preserve captured inputs, scaled bus limits, partial
fault effects and first-fault priority. Paired external accesses stop after a fault;
ordinary flat-memory partial effects follow the existing implementation. Integer
emission covers raw-LSB INC/DEC and low-48 unsigned CMPU. DIV preserves
the raw quotient carry and unaffected CCR bits across SM limiting. NORM supports
flags-selected shifts and compatibility-mode exponent wrapping; NORMF checks
signed register counts and bypasses SM limiting. Native state matrices and
mixed instruction sequences check these effects and exact budget/fault exits.
Other instruction families and complete owner integration remain open.
Complete ARM64 Core execution and qualification remain in progress. The production
Core still reports an unsupported host on ARM64.

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
nonlinear banks compare optimized C++ with complete handwritten scalar ASM kernels,
including frame/channel/stage loops, quantization, histories and output rings;
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
