// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace dsp56300::detail {
// Append-only dual mappings: published bytes and entry addresses remain valid
// until destruction. Only private storage is writable; callers receive RX addresses.
class ExecutableArena {
public:
  ExecutableArena();
  ~ExecutableArena();
  ExecutableArena(const ExecutableArena &) = delete;
  ExecutableArena &operator=(const ExecutableArena &) = delete;
  [[nodiscard]] const void *append(std::span<const std::uint8_t> bytes);

private:
  class Segment;
  std::vector<std::unique_ptr<Segment>> segments_;
};
}
