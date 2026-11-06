// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace dsp56300::detail {
// Published images and their entry addresses remain valid until destruction.
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
