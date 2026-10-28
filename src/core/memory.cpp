// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include <cerrno>
#include <cstddef>
#include <memory>
#include <span>
#include <sys/mman.h>
#include <system_error>

namespace dsp56300 {
namespace {
constexpr std::size_t arena_words = address_words * 3;
constexpr std::size_t arena_bytes = arena_words * sizeof(Word);
}

struct MemoryArena::Impl {
  Impl() {
    void *const memory = mmap(nullptr,
                              arena_bytes,
                              PROT_READ | PROT_WRITE,
                              MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE,
                              -1,
                              0);
    if (memory == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "DSP address-space reservation");
    }
    words = {static_cast<Word *>(memory), arena_words};
  }

  ~Impl() { static_cast<void>(munmap(words.data(), arena_bytes)); }

  Impl(const Impl &) = delete;
  Impl &operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl &operator=(Impl &&) = delete;
  std::span<Word> words;
};

MemoryArena::MemoryArena() : impl_(std::make_unique<Impl>()) {}

MemoryArena::~MemoryArena() = default;

MemoryArena::MemoryArena(MemoryArena &&) noexcept = default;

MemoryArena &MemoryArena::operator=(MemoryArena &&) noexcept = default;

Buffers MemoryArena::buffers() const noexcept {
  if (!impl_) { return {}; }
  return {.program = impl_->words.first(address_words),
          .x = impl_->words.subspan(address_words, address_words),
          .y = impl_->words.subspan(address_words * 2, address_words)};
}
} // namespace dsp56300
