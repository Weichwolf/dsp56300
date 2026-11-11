// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace dsp56300::detail {
class LlvmModule;
enum class IoRole : std::uint8_t { read, write, bit, reset, pending, revision };

struct IoBinding {
  IoRole role;
  std::string_view name;
};

// Immutable inputs; selected functions use only the public Io ABI, never Core::Impl.
[[nodiscard]] std::vector<std::byte> fuse_io(std::span<const std::byte> semantics,
                                             std::span<const std::byte> host,
                                             std::span<const IoBinding> bindings);
// Retained adapter boundaries are cold binding points, never extra runtime calls.
void inline_io(LlvmModule &module);
}
