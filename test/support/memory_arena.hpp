// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"

#include <cerrno>
#include <cstddef>
#include <span>
#include <sys/mman.h>
#include <system_error>
#include <utility>

namespace dsp56300::test {
// Test-owned complete address spaces for directed 24-bit wrap and I/O boundaries.
class MemoryArena {
public:
  MemoryArena() {
    void *const storage = mmap(
        nullptr, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (storage == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "Test DSP memory reservation");
    }
    words_ = {static_cast<Word *>(storage), words};
  }

  ~MemoryArena() {
    if (!words_.empty()) { static_cast<void>(munmap(words_.data(), bytes)); }
  }

  MemoryArena(const MemoryArena &) = delete;
  MemoryArena &operator=(const MemoryArena &) = delete;

  MemoryArena(MemoryArena &&other) noexcept : words_(std::exchange(other.words_, {})) {}

  MemoryArena &operator=(MemoryArena &&) = delete;

  [[nodiscard]] Buffers buffers() const noexcept {
    if (words_.empty()) { return {}; }
    return {.program = words_.first(address_words),
            .x = words_.subspan(address_words, address_words),
            .y = words_.subspan(address_words * 2, address_words)};
  }

  [[nodiscard]] Memory memory() const { return Memory::separate(buffers()); }

private:
  static constexpr std::size_t words = address_words * 3, bytes = words * sizeof(Word);
  std::span<Word> words_;
};
}
