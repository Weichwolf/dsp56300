// SPDX-License-Identifier: GPL-3.0-only
#include "jit/index.hpp"

#include "jit/block.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#if defined(__x86_64__) && defined(__linux__)
#include <cerrno>
#include <sys/mman.h>
#include <system_error>
#endif

namespace dsp56300::detail {
namespace {
constexpr std::uint32_t byte_mask = native_page_entries - 1;
}

NativeTable::NativeTable(const unsigned levels) : direct_(nullptr, Unmap{.levels = levels}) {
  if (levels == 0 || levels > sizeof(std::uint32_t)) {
    throw std::invalid_argument("Native index exceeds its key width");
  }
#if defined(__x86_64__) && defined(__linux__)
  if (levels >= 3) {
    const auto entries = std::size_t{1} << (levels * native_page_shift);
    const auto bytes = entries * sizeof(void *);
    // Reserve target addresses, not RAM/swap for every possible context.
    constexpr auto flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE;
    auto *const allocation = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    if (allocation == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "Native target table mmap");
    }
    // Default construction starts pointer lifetimes without committing untouched pages.
    direct_.reset(static_cast<const void **>(allocation));
    std::uninitialized_default_construct_n(direct_.get(), entries);
    if (madvise(allocation, bytes, MADV_NOHUGEPAGE) != 0 && errno != EINVAL && errno != ENOSYS) {
      throw std::system_error(errno, std::generic_category(), "Native target table page policy");
    }
    index_.entries = direct_.get();
  }
#endif
}

void NativeTable::Unmap::operator()(const void **const entries) const noexcept {
#if defined(__x86_64__) && defined(__linux__)
  const auto bytes = (std::size_t{1} << (levels * native_page_shift)) * sizeof(void *);
  static_cast<void>(munmap(static_cast<void *>(entries), bytes));
#else
  static_cast<void>(entries);
#endif
}

const void *NativeTable::find(const std::uint32_t key) const noexcept {
  const auto levels = direct_.get_deleter().levels;
  if (levels < sizeof(key) && (key >> (levels * native_page_shift)) != 0) { return nullptr; }
  if (direct_) { return direct_.get()[key]; }
  const auto *page = &root_;
  for (auto shift = (levels - 1) * native_page_shift; shift != 0; shift -= native_page_shift) {
    const auto *child = (*page)[(key >> shift) & byte_mask];
    if (child == nullptr) { return nullptr; }
    page = static_cast<const NativePage *>(child);
  }
  return (*page)[key & byte_mask];
}

void NativeTable::insert(const std::uint32_t key, const void *const entry) {
  const auto levels = direct_.get_deleter().levels;
  if (levels < sizeof(key) && (key >> (levels * native_page_shift)) != 0) {
    throw std::out_of_range("Native target exceeds its key width");
  }
  if (direct_) {
    if (direct_.get()[key] == nullptr && entry != nullptr) { touched_.push_back(key); }
    direct_.get()[key] = entry;
    return;
  }
  auto *page = &root_;
  for (auto shift = (levels - 1) * native_page_shift; shift != 0; shift -= native_page_shift) {
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
  for (const auto key : touched_) { direct_.get()[key] = nullptr; }
  touched_.clear();
  root_.fill(nullptr);
  nodes_.clear();
}
}
