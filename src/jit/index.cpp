// SPDX-License-Identifier: GPL-3.0-only
#include "jit/index.hpp"

#include "jit/block.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>

namespace dsp56300::detail {
namespace {
constexpr std::uint32_t byte_mask = native_page_entries - 1;
}

NativeTable::NativeTable(const unsigned levels) : levels_(levels) {
  if (levels == 0 || levels > sizeof(std::uint32_t)) {
    throw std::invalid_argument("Native index exceeds its key width");
  }
}

const void *NativeTable::find(const std::uint32_t key) const noexcept {
  const auto *page = &index_;
  for (auto shift = (levels_ - 1) * native_page_shift; shift != 0; shift -= native_page_shift) {
    const auto *child = (*page)[(key >> shift) & byte_mask];
    if (child == nullptr) { return nullptr; }
    page = static_cast<const NativePage *>(child);
  }
  return (*page)[key & byte_mask];
}

void NativeTable::insert(const std::uint32_t key, const void *const entry) {
  auto *page = &index_;
  for (auto shift = (levels_ - 1) * native_page_shift; shift != 0; shift -= native_page_shift) {
    auto &child = (*page)[(key >> shift) & byte_mask];
    if (child == nullptr) {
      nodes_.push_back(std::make_unique<NativePage>());
      child = nodes_.back().get();
    }
    // Every child was allocated mutable by this owner. Only the native read
    // view is const; no table may be edited while its native reader executes.
    page = const_cast<NativePage *>(static_cast<const NativePage *>(child));
  }
  (*page)[key & byte_mask] = entry;
}

void NativeTable::clear() noexcept {
  index_.fill(nullptr);
  nodes_.clear();
}
}
