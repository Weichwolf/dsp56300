// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace llvm::orc {
class LLJIT;
}

namespace llvm_native_sites {
struct TextRange {
  std::uintptr_t address = 0;
  std::uint64_t size = 0;
};

struct Site {
  std::uintptr_t address = 0;
  std::uint64_t size = 0, variant = 0;
  std::uint32_t guest_pc = 0;
};

struct FaultMap {
  std::span<const TextRange> text_ranges;
  std::span<const Site> sites;
};

struct SourceMap {
  void attach(llvm::orc::LLJIT &jit);
  [[nodiscard]] FaultMap map() const noexcept;
  std::vector<TextRange> text;
  std::vector<Site> sites;
  std::mutex mutation;
};
}
