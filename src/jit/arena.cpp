// SPDX-License-Identifier: GPL-3.0-only
#include "jit/arena.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace dsp56300::detail {
namespace {
constexpr std::size_t arena_bytes = std::size_t{64} << 10;
constexpr std::size_t entry_alignment = 16;

std::size_t allocation_size(const std::size_t requested) {
  const auto page = sysconf(_SC_PAGESIZE);
  if (page <= 0) { throw std::runtime_error("Cannot determine JIT page size"); }
  const auto page_size = static_cast<std::size_t>(page);
  const auto size = std::max(arena_bytes, requested);
  const auto extra = (page_size - (size % page_size)) % page_size;
  if (size > static_cast<std::size_t>(std::numeric_limits<off_t>::max()) - extra) {
    throw std::length_error("JIT executable allocation exceeds host file size");
  }
  return size + extra;
}
}

class ExecutableArena::Segment {
public:
  explicit Segment(const std::size_t requested) : size_(allocation_size(requested)) {
    const auto descriptor = memfd_create("dsp56300-jit", MFD_CLOEXEC);
    if (descriptor < 0) { throw std::system_error(errno, std::generic_category(), "JIT memfd"); }
    if (ftruncate(descriptor, static_cast<off_t>(size_)) != 0) {
      const auto error = errno;
      close(descriptor);
      throw std::system_error(error, std::generic_category(), "JIT ftruncate");
    }
    writable_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, descriptor, 0);
    if (writable_ == MAP_FAILED) {
      const auto error = errno;
      close(descriptor);
      throw std::system_error(error, std::generic_category(), "JIT writable mmap");
    }
    executable_ = mmap(nullptr, size_, PROT_READ | PROT_EXEC, MAP_SHARED, descriptor, 0);
    const auto error = errno;
    close(descriptor);
    if (executable_ == MAP_FAILED) {
      munmap(writable_, size_);
      throw std::system_error(error, std::generic_category(), "JIT executable mmap");
    }
  }

  ~Segment() {
    munmap(writable_, size_);
    munmap(executable_, size_);
  }

  Segment(const Segment &) = delete;
  Segment &operator=(const Segment &) = delete;

  [[nodiscard]] const void *append(const std::span<const std::uint8_t> bytes) noexcept {
    const auto padding = (entry_alignment - (used_ % entry_alignment)) % entry_alignment;
    if (size_ - used_ < padding || size_ - used_ - padding < bytes.size()) { return nullptr; }
    const auto offset = used_ + padding;
    std::memcpy(static_cast<std::uint8_t *>(writable_) + offset, bytes.data(), bytes.size());
    used_ = offset + bytes.size();
    return static_cast<const std::uint8_t *>(executable_) + offset;
  }

private:
  void *writable_ = nullptr;
  void *executable_ = nullptr;
  std::size_t size_ = 0, used_ = 0;
};

ExecutableArena::ExecutableArena() = default;
ExecutableArena::~ExecutableArena() = default;

const void *ExecutableArena::append(const std::span<const std::uint8_t> bytes) {
  if (bytes.empty()) { throw std::invalid_argument("Cannot publish empty JIT code"); }
  if (!segments_.empty()) {
    if (const auto *entry = segments_.back()->append(bytes)) { return entry; }
  }
  auto segment = std::make_unique<Segment>(bytes.size());
  const auto *entry = segment->append(bytes);
  segments_.push_back(std::move(segment));
  return entry;
}

}
