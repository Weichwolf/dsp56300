// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

namespace dsp56300::detail {
[[nodiscard]] std::uint64_t move_storage(const Registers &state, RegisterUpdate update);
void merge_move_storage(Registers &state, RegisterUpdate update);
} // namespace dsp56300::detail
