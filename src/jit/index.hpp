// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/block.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dsp56300::detail {
// Single-owner stable target table. Linux x86 PC/context tables use demand-zero
// virtual memory; other tables retain sparse radix pages. Updates happen outside execution.
class NativeTable {
public:
  explicit NativeTable(unsigned levels);
  NativeTable(const NativeTable &) = delete;
  NativeTable &operator=(const NativeTable &) = delete;
  [[nodiscard]] const void *find(std::uint32_t key) const noexcept;
  void insert(std::uint32_t key, const void *entry);
  void clear() noexcept;

  [[nodiscard]] const NativeIndex &index() const noexcept { return index_; }

private:
  struct Unmap {
    unsigned levels = 0;
    void operator()(const void **entries) const noexcept;
  };

  NativePage root_{};
  NativeIndex index_{.entries = root_.data()};
  std::vector<std::unique_ptr<NativePage>> nodes_;
  std::unique_ptr<const void *, Unmap> direct_{nullptr, Unmap{}};
  std::vector<std::uint32_t> touched_;
};
}
