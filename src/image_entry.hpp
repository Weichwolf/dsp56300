// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "jit/llvm/image.hpp"

#include <cstdint>

namespace dsp56300::detail {
// The generated entry has a verified native ABI, not a Clang C++ type prefix.
extern "C" std::uint64_t dsp56300_image_enter(ImageEntry entry,
                                              void *owner,
                                              Word pc,
                                              std::uint32_t instructions,
                                              std::uint64_t cycles) noexcept;
}
