// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/value_graph.hpp"

#include <cstdint>

namespace dsp56300::detail::a64 {
void RegionEmitter::saturate_words(const GraphNode &node, const Vec target) {
  if (node.parameter != 0) {
    word_words(node, target, false);
    return;
  }
  const auto value = aligned_words(node.inputs[0], 0, first_scratch);
  neon_.move(target, value);
}

void RegionEmitter::saturate(const GraphNode &node, const Vec target, const unsigned part) {
  const auto value = node.inputs[0];
  canonical(target, value, part);
  if (value.scale != 0) { neon_.shift(target, target, Shift::left, value.scale); }
  const auto &bounds = bounds_[value.node];
  const auto factor = std::int64_t{1} << value.scale;
  constexpr auto low = -(std::int64_t{1} << (double_word_bits - 1));
  constexpr auto high = -low - 1;
  if (bounds.low * factor < low) {
    neon_.binary(VectorOp::greater, first_scratch, saturation_low, target);
    neon_.binary(VectorOp::bit_insert, target, saturation_low, first_scratch);
    merge_wide(StickyEvent::limit, first_scratch, part);
  }
  if (bounds.high * factor > high) {
    neon_.binary(VectorOp::greater, first_scratch, target, saturation_high);
    neon_.binary(VectorOp::bit_insert, target, saturation_high, first_scratch);
    merge_wide(StickyEvent::limit, first_scratch, part);
  }
}
}
