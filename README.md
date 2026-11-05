# dsp56300

**An independent DSP56300 native JIT core built for high throughput.**

C++23 · LLVM 19 · Linux x86-64 · GPL-3.0-only

Execute DSP programs with exact fixed-point arithmetic and explicit memory and
event contracts. The library targets audio, image/video, communications,
measurement and control. Device peripherals and application firmware belong to
consumers; no other DSP emulator is an execution dependency.

## Status

The core provides native JIT, cached and reference execution, typed memory views,
I/O boundaries, interrupts and checkpoints. Automatic selection uses the fastest
qualified host path. Complete ISA/mode qualification, removal of remaining general
helpers and representative sustained performance remain open. A passing fixture
set does not establish complete DSP or instrument compatibility.

## Build

Requires CMake >=3.25, Ninja, Clang 19, clang-format-19 and clang-tidy-19.

```sh
cmake --preset release
cmake --build --preset release
cmake --build build/release --target format-check
```

`strict` retains ASan/UBSan; `thread` selects separate TSan. Until the integrating
instrument reaches real time, execution tests are suspended; only optimized
startup/throughput probes of at most 10 host seconds are allowed. Fuzzing requires
an explicit request. The `bench` target uses independently authored workloads.

## Embed

```cmake
add_subdirectory(path/to/dsp56300)
target_link_libraries(your_app PRIVATE dsp56300::core)
```

Include `<dsp56300/core.hpp>` and use `Core(Memory, Io*, Options)`.
Typed views describe P/X/Y storage in architecture-native units. Keep memory and
I/O owners alive; give each core one execution owner. Consumers own clocks,
peripherals, interrupt arbitration and pacing. Preserve program invalidation and
pair CPU checkpoints with matching memory and peripheral state. JIT compilation
can allocate; keep it outside real-time audio callbacks. Native code uses W^X.

Embedded tests are opt-in with `DSP56300_BUILD_TESTING`; match
`DSP56300_SANITIZER` to the parent configuration. The embedded format target is
`dsp56300-format-check`.

## Design and references

- [Target architecture](doc/architecture.md)
- [Complete objective](OBJECTIVE.md)
- [ISA coverage catalogue](test/isa-coverage.tsv)
- [Primary references](doc/references.json)
- [Development rules](AGENTS.md) · [License](LICENSE) · [Attribution](THIRD_PARTY.txt)

Manufacturer firmware, manuals and application programs are not distributed.
