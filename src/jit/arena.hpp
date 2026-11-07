// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace dsp56300::detail {
enum class WriteProtection : std::uint8_t { automatic, owner };

// Append-only code; published entry addresses remain valid until destruction.
// Linux uses RW/RX aliases; macOS uses thread-local or execution-owner W^X protection.
class ExecutableArena {
public:
  explicit ExecutableArena(WriteProtection protection = WriteProtection::automatic);
  ~ExecutableArena();
  ExecutableArena(const ExecutableArena &) = delete;
  ExecutableArena &operator=(const ExecutableArena &) = delete;
  [[nodiscard]] const void *append(std::span<const std::uint8_t> bytes);

private:
  [[nodiscard]] const void *publish(std::span<const std::uint8_t> bytes);
  class Segment;
  WriteProtection protection_;
  std::vector<std::unique_ptr<Segment>> segments_;
};
}
