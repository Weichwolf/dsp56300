// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace dsp56300::detail {
// Append-only code; published entry addresses remain valid until destruction.
// Linux uses RW/RX aliases; Apple silicon uses thread-local MAP_JIT write protection.
class ExecutableArena {
public:
  ExecutableArena();
  ~ExecutableArena();
  ExecutableArena(const ExecutableArena &) = delete;
  ExecutableArena &operator=(const ExecutableArena &) = delete;
  [[nodiscard]] const void *append(std::span<const std::uint8_t> bytes);

private:
  [[nodiscard]] const void *publish(std::span<const std::uint8_t> bytes);
  class Segment;
  std::vector<std::unique_ptr<Segment>> segments_;
};
}
