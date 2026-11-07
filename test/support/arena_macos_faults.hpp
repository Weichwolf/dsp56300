// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>

namespace dsp56300::test {
enum class ArenaFailure : std::uint8_t { none, allocation, writable, executable };
void set_arena_failure(ArenaFailure requested) noexcept;
bool arena_failure_released() noexcept;
}
