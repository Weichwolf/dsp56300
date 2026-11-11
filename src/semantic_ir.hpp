// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <span>
#include <string_view>

namespace dsp56300::detail {
[[nodiscard]] std::span<const std::byte> semantic_ir() noexcept;
[[nodiscard]] std::string_view compiler_identity() noexcept;
}
