// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"

#include <array>
#include <span>

namespace dsp56300::detail {
void validate_memory_views(const std::array<std::span<const MemoryWindow>, 3> &regions,
                           const std::array<Memory::Views, 3> &views);
}
