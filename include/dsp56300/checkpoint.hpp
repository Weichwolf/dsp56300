// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <memory>

namespace dsp56300 {
class Core;

// Opaque stopped CPU state. Caller-owned memory and I/O are saved separately.
class Checkpoint {
public:
  // Empty handles, including moved-from handles, cannot be restored.
  Checkpoint() = default;
  Checkpoint(const Checkpoint &) = default;
  Checkpoint &operator=(const Checkpoint &) = default;
  Checkpoint(Checkpoint &&) noexcept = default;
  Checkpoint &operator=(Checkpoint &&) noexcept = default;
  ~Checkpoint();

private:
  friend class Core;
  struct State;
  explicit Checkpoint(std::shared_ptr<const State> state);
  std::shared_ptr<const State> state_;
};
} // namespace dsp56300
