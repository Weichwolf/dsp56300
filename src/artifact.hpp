// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace dsp56300::detail {
class ArtifactWriter {
public:
  void integer(const std::uint64_t value) {
    constexpr unsigned byte_bits = 8;
    for (unsigned offset = 0; offset < sizeof(value); ++offset) {
      bytes_.push_back(static_cast<std::byte>(value >> (byte_bits * offset)));
    }
  }

  void bytes(const std::span<const std::byte> value) {
    integer(value.size());
    bytes_.insert(bytes_.end(), value.begin(), value.end());
  }

  void text(const std::string_view value) { bytes(std::as_bytes(std::span(value))); }

  [[nodiscard]] std::vector<std::byte> finish() const {
    const auto digest = llvm::SHA256::hash(
        llvm::ArrayRef(reinterpret_cast<const std::uint8_t *>(bytes_.data()), bytes_.size()));
    std::vector<std::byte> result;
    result.reserve(digest.size() + bytes_.size());
    for (const auto byte : digest) { result.push_back(static_cast<std::byte>(byte)); }
    result.insert(result.end(), bytes_.begin(), bytes_.end());
    return result;
  }

private:
  std::vector<std::byte> bytes_;
};

class ArtifactReader {
public:
  explicit ArtifactReader(const std::span<const std::byte> artifact) {
    constexpr std::size_t digest_bytes = 32;
    if (artifact.size() < digest_bytes) { fail(); }
    remaining_ = artifact.subspan(digest_bytes);
    const auto digest = llvm::SHA256::hash(llvm::ArrayRef(
        reinterpret_cast<const std::uint8_t *>(remaining_.data()), remaining_.size()));
    if (!std::ranges::equal(artifact.first(digest_bytes), std::as_bytes(std::span(digest)))) {
      fail();
    }
  }

  template <std::unsigned_integral T = std::uint64_t> [[nodiscard]] T integer() {
    if (remaining_.size() < sizeof(std::uint64_t)) { fail(); }
    std::uint64_t value = 0;
    constexpr unsigned byte_bits = 8;
    for (unsigned offset = 0; offset < sizeof(value); ++offset) {
      value |= std::to_integer<std::uint64_t>(remaining_[offset]) << (byte_bits * offset);
    }
    remaining_ = remaining_.subspan(sizeof(value));
    if (value > std::numeric_limits<T>::max()) { fail(); }
    return static_cast<T>(value);
  }

  [[nodiscard]] std::size_t count(const std::size_t minimum_bytes) {
    const auto size = integer<std::size_t>();
    if (minimum_bytes == 0 || size > remaining_.size() / minimum_bytes) { fail(); }
    return size;
  }

  [[nodiscard]] std::span<const std::byte> bytes() {
    const auto size = count(1);
    const auto result = remaining_.first(size);
    remaining_ = remaining_.subspan(size);
    return result;
  }

  [[nodiscard]] std::string text() {
    const auto value = bytes();
    return {reinterpret_cast<const char *>(value.data()), value.size()};
  }

  void finish() const {
    if (!remaining_.empty()) { fail(); }
  }

  [[noreturn]] static void fail() { throw std::invalid_argument("Invalid DSP semantic artifact"); }

private:
  std::span<const std::byte> remaining_;
};
}
