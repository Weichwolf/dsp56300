// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace dsp56300::detail::a64 {
void RegionEmitter::fuse_products() {
  const auto &nodes = region_.graph.nodes;
  std::vector<std::size_t> uses(nodes.size());
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    if (!needed_[index]) { continue; }
    for (const auto input : nodes[index].inputs) {
      if (input.valid()) { ++uses[input.node]; }
    }
  }
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const auto &node = nodes[index];
    if (!needed_[index] || compact_[index] ||
        (node.operation != GraphOp::add && node.operation != GraphOp::subtract)) {
      continue;
    }
    const auto left = node.inputs[0];
    const auto right = node.inputs[1];
    const auto &product = nodes[right.node];
    if (compact(left) || !left.canonical || left.scale > right.scale ||
        product.operation != GraphOp::multiply || uses[right.node] != 1 ||
        last_[right.node] != index || last_[left.node] != index || !compact(product.inputs[0]) ||
        !compact(product.inputs[1]) || product.inputs[0].width > 32 ||
        right.scale - left.scale > 32 - product.inputs[0].width) {
      continue;
    }
    fused_[index] = right.node;
    needed_[right.node] = false;
    for (const auto input : product.inputs) {
      last_[input.node] = std::max(last_[input.node], index);
    }
  }
}
}
