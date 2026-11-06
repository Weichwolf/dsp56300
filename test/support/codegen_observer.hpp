// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <cstdint>

namespace dsp56300::test::codegen {
struct Publications {
  std::uint64_t count = 0;
  std::size_t bytes = 0;
  bool operator==(const Publications &) const = default;
};

// Test executables observe actual successful executable permission changes.
// No core instrumentation is needed; inspect on the execution owner's thread.
[[nodiscard]] Publications publications() noexcept;
}
