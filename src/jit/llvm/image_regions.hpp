// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/image_plan.hpp"

#include <vector>

namespace dsp56300::detail {
// Natural complete DO bodies; nested or unsupported bodies retain precise image sites.
[[nodiscard]] std::vector<std::vector<BlockInstruction>> image_regions(const ImagePlan &plan);
}
