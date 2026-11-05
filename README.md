# dsp56300

Independent DSP56300 emulator with an x86-64 JIT.

C++23 · Linux x86-64 · GPL-3.0-only

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
#include <dsp56300/isa.hpp>
#include <dsp56300/memory.hpp>

#include <algorithm>
#include <array>

int main() {
  std::array<dsp56300::Word, 4096> ram{};
  constexpr auto program = dsp56300::isa::program(dsp56300::isa::nop, dsp56300::isa::stop);
  std::ranges::copy(program, ram.begin());
  dsp56300::Core core(dsp56300::Memory::unified(ram));
  const auto result = core.run();
  return result.fault.kind == dsp56300::FaultKind::none ? 0 : 1;
}
```

Memory is borrowed and must outlive the core. One execution thread owns each
core. The example executes NOP and STOP; a consumer supplies its own program and
optional I/O. Unsupported instructions or modes report explicit faults.
`Memory::unified` explicitly aliases P/X/Y; `Memory::separate` supplies distinct
logical address spaces.

[License](LICENSE)
