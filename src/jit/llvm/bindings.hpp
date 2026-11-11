// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"

#include <cstddef>
#include <span>

namespace dsp56300::detail {
class LlvmModule;
void declare_callbacks(LlvmModule &module, std::span<const CallbackBinding> bindings);
void link_callbacks(LlvmModule &module,
                    std::span<const std::byte> host,
                    std::span<const CallbackBinding> bindings);
void mark_io(LlvmModule &module);
void inline_io(LlvmModule &module);
void finalize_io(LlvmModule &module);
}
