// SPDX-License-Identifier: GPL-3.0-only
#include "jit/arena.hpp"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace dsp56300::detail {
namespace {
std::size_t allocation_size(const std::size_t bytes) {
  const auto page = sysconf(_SC_PAGESIZE);
  if (page <= 0) { throw std::runtime_error("Cannot determine executable page size"); }
  const auto page_size = static_cast<std::size_t>(page);
  const auto padding = (page_size - (bytes % page_size)) % page_size;
  if (bytes > std::numeric_limits<std::size_t>::max() - padding) {
    throw std::length_error("Native code size overflow");
  }
  return bytes + padding;
}
}

class ExecutableArena::Segment {
public:
  explicit Segment(const std::span<const std::uint8_t> bytes)
      : size_(allocation_size(bytes.size())) {
    data_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (data_ == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "JIT mmap");
    }
    std::memcpy(data_, bytes.data(), bytes.size());
    if (mprotect(data_, size_, PROT_READ | PROT_EXEC) != 0) {
      const auto error = errno;
      munmap(data_, size_);
      throw std::system_error(error, std::generic_category(), "JIT mprotect");
    }
  }

  ~Segment() { munmap(data_, size_); }

  Segment(const Segment &) = delete;
  Segment &operator=(const Segment &) = delete;

  [[nodiscard]] const void *entry() const noexcept { return data_; }

private:
  void *data_ = nullptr;
  std::size_t size_ = 0;
};

ExecutableArena::ExecutableArena() = default;
ExecutableArena::~ExecutableArena() = default;

const void *ExecutableArena::append(const std::span<const std::uint8_t> bytes) {
  if (bytes.empty()) { throw std::invalid_argument("Cannot publish empty JIT code"); }
  auto segment = std::make_unique<Segment>(bytes);
  const auto *entry = segment->entry();
  segments_.push_back(std::move(segment));
  return entry;
}

}
