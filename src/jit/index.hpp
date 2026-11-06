// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/block.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dsp56300::detail {
// Single-owner sparse byte-radix table. Native code reads its stable pointer
// pages; construction and clearing happen outside native execution.
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
  NativeIndex index_{};
  unsigned levels_;
  std::vector<std::unique_ptr<NativePage>> nodes_;
};
}
