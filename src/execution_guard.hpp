// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stdexcept>

namespace dsp56300::detail {
class ExecutionGuard {
public:
  explicit ExecutionGuard(bool &executing) : executing_(executing) {
    if (executing_) { throw std::logic_error("DSP execution cannot be reentered from a callback"); }
    executing_ = true;
  }

  ~ExecutionGuard() { executing_ = false; }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;
  ExecutionGuard(ExecutionGuard &&) = delete;
  ExecutionGuard &operator=(ExecutionGuard &&) = delete;

private:
  bool &executing_;
};

} // namespace dsp56300::detail
