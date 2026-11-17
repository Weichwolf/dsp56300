// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/llvm/image.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <setjmp.h> // IWYU pragma: export
#include <span>

namespace dsp56300::detail {
// Escape stays inside the native call, under the C++ execution-owner scopes.
class NativeCall {
public:
  NativeCall(std::span<const NativeSymbols::TextRange> text,
             const std::array<Memory::Views, 3> &views) noexcept
      : text_(text), views_(views) {}

  [[nodiscard]] std::uint64_t invoke(ImageEntry entry,
                                     void *owner,
                                     Word pc,
                                     std::uint32_t instructions,
                                     std::uint64_t cycles) noexcept;
  [[nodiscard]] bool failed() const noexcept;
  [[nodiscard]] Fault fault(std::span<const NativeSymbols::Site> sites) const;
  [[nodiscard]] static bool
  capture(std::uintptr_t address, bool writing, std::uintptr_t native_pc) noexcept;

private:
  sigjmp_buf saved_;
  std::span<const NativeSymbols::TextRange> text_;
  const std::array<Memory::Views, 3> &views_;
  std::atomic<std::uintptr_t> address_{0}, pc_{0};
  std::atomic<bool> writing_{false}, failed_{false};
  static thread_local std::atomic<NativeCall *> active_;
};
}
