// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::test::codegen {
struct Publications {
  std::uint64_t count = 0;
  std::size_t bytes = 0;
  bool operator==(const Publications &) const = default;
};

struct PublishedImage {
  const std::uint8_t *entry = nullptr;
  std::size_t bytes = 0;
};

// Test linkage observes successful append calls, outside the core. Bytes count
// actual published instructions and literals, independently of reserved pages.
[[nodiscard]] Publications publications() noexcept;
// Lifetime checks opt in to exact image spans; benchmarks need no image log.
void track_images();
[[nodiscard]] std::span<const PublishedImage> published_images() noexcept;
}
