// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#if defined(__APPLE__)
#include "jit/arena.hpp"
#endif

namespace dsp56300::test::codegen {
namespace {
thread_local Publications observed;
thread_local bool tracking = false;
thread_local std::vector<PublishedImage> images;

void record(const void *const entry, const std::size_t bytes) {
  ++observed.count;
  observed.bytes += bytes;
  if (tracking) {
    images.push_back({.entry = static_cast<const std::uint8_t *>(entry), .bytes = bytes});
  }
}

}

Publications publications() noexcept {
  return observed;
}

void track_images() {
  images.clear();
  tracking = true;
}

std::span<const PublishedImage> published_images() noexcept {
  return images;
}

#if !defined(__APPLE__)
extern "C" const void *system_append(void *arena, std::span<const std::uint8_t> bytes) asm(
    "__real__ZN8dsp563006detail15ExecutableArena6appendESt4spanIKhLm18446744073709551615EE");
extern "C" const void *observe_append(void *arena, std::span<const std::uint8_t> bytes) asm(
    "__wrap__ZN8dsp563006detail15ExecutableArena6appendESt4spanIKhLm18446744073709551615EE");

const void *observe_append(void *const arena, const std::span<const std::uint8_t> bytes) {
  const auto *const entry = system_append(arena, bytes);
  record(entry, bytes.size());
  return entry;
}
#endif
}
#if defined(__APPLE__)
namespace dsp56300::detail {
const void *ExecutableArena::append(const std::span<const std::uint8_t> bytes) {
  const auto *entry = publish(bytes);
  test::codegen::record(entry, bytes.size());
  return entry;
}
}
#endif
