// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::test::codegen {
namespace {
thread_local Publications observed;
thread_local bool tracking = false;
thread_local std::vector<PublishedImage> images;
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

extern "C" const void *system_append(void *arena, std::span<const std::uint8_t> bytes) asm(
    "__real__ZN8dsp563006detail15ExecutableArena6appendESt4spanIKhLm18446744073709551615EE");
extern "C" const void *observe_append(void *arena, std::span<const std::uint8_t> bytes) asm(
    "__wrap__ZN8dsp563006detail15ExecutableArena6appendESt4spanIKhLm18446744073709551615EE");

const void *observe_append(void *const arena, const std::span<const std::uint8_t> bytes) {
  const auto *const entry = system_append(arena, bytes);
  ++observed.count;
  observed.bytes += bytes.size();
  if (tracking) {
    images.push_back({.entry = static_cast<const std::uint8_t *>(entry), .bytes = bytes.size()});
  }
  return entry;
}
}
