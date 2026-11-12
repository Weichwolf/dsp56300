// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit.hpp"
#include "jit/image_plan.hpp"

#include <span>
#include <vector>

namespace dsp56300::detail {
// Natural DO bodies and ordinary prefixes; unsupported effects retain precise sites.
[[nodiscard]] std::vector<std::vector<BlockInstruction>> image_regions(const ImagePlan &plan);
[[nodiscard]] std::vector<std::vector<BlockInstruction>>
image_regions(const ImagePlan &plan, std::span<const RegionLayout> memory);
}
