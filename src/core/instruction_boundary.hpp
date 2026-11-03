// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "instruction.hpp"

namespace dsp56300::detail {
// A conservative complete-instruction owner boundary, before any CPU or bus effect.
[[nodiscard]] bool requires_owner(const Instruction &instruction);
}
