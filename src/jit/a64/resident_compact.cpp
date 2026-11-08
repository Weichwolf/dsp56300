// SPDX-License-Identifier: GPL-3.0-only
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::detail::a64 {
bool RegionEmitter::compact(const GraphValue value) const {
  return compact_.at(value.node);
}

Vec RegionEmitter::aligned_words(const GraphValue value, const unsigned scale, const Vec scratch) {
  auto source = read(value);
  if (!compact(value)) {
    neon_.narrow(scratch, source);
    if (parts_ == 2) { neon_.narrow(scratch, read(value, 1), true); }
    source = scratch;
  }
  if (value.scale == scale) { return source; }
  const auto amount = value.scale - scale;
  if (amount >= 32) {
    neon_.zero(scratch);
  } else {
    neon_.shift(scratch, source, Shift::left, amount, true);
  }
  return scratch;
}

void RegionEmitter::multiply(const GraphNode &node, const Vec target, const unsigned part) {
  if (compact(node.inputs[0]) && compact(node.inputs[1])) {
    neon_.binary(part == 0 ? VectorOp::multiply : VectorOp::multiply_upper,
                 target,
                 read(node.inputs[0]),
                 read(node.inputs[1]));
    return;
  }
  canonical(first_scratch, node.inputs[0], part);
  neon_.narrow(first_scratch, first_scratch);
  canonical(second_scratch, node.inputs[1], part);
  neon_.narrow(second_scratch, second_scratch);
  neon_.binary(VectorOp::multiply, target, first_scratch, second_scratch);
}

void RegionEmitter::pack(const GraphNode &node, const Vec target) {
  constexpr Vec temporary{30};
  for (unsigned part = 0; part < parts_; ++part) {
    if (node.operation == GraphOp::word) {
      word(node, temporary, part);
    } else {
      canonical(temporary, node.inputs[0], part);
      neon_.shift(temporary, temporary, Shift::arithmetic, 1);
    }
    neon_.narrow(target, temporary, part != 0);
  }
}

void RegionEmitter::emit_words(const GraphNode &node, const Vec target) {
  switch (node.operation) {
    case GraphOp::load: load_words(target, node.parameter); break;
    case GraphOp::invariant:
      code_.load(Reg::x9,
                 Reg::x0,
                 offsetof(RegionFrame, invariants) + (node.parameter * sizeof(std::uint64_t)),
                 Access::doubleword);
      neon_.broadcast(target, Reg::x9, true);
      break;
    case GraphOp::zero: neon_.zero(target); break;
    case GraphOp::add:
    case GraphOp::subtract: {
      const auto scale = std::min(node.inputs[0].scale, node.inputs[1].scale);
      const auto left = aligned_words(node.inputs[0], scale, first_scratch);
      const auto right = aligned_words(node.inputs[1], scale, second_scratch);
      neon_.binary(node.operation == GraphOp::add ? VectorOp::add_words : VectorOp::subtract_words,
                   target,
                   left,
                   right);
      break;
    }
    case GraphOp::multiply: {
      const auto left = aligned_words(node.inputs[0], node.inputs[0].scale, first_scratch);
      const auto right = aligned_words(node.inputs[1], node.inputs[1].scale, second_scratch);
      neon_.binary(VectorOp::multiply_words, target, left, right);
      break;
    }
    case GraphOp::negate: {
      const auto value = aligned_words(node.inputs[0], node.inputs[0].scale, second_scratch);
      neon_.zero(first_scratch);
      neon_.binary(VectorOp::subtract_words, target, first_scratch, value);
      break;
    }
    case GraphOp::shift:
      if (compact(node.inputs[0])) {
        neon_.shift(target, read(node.inputs[0]), Shift::arithmetic, 1, true);
      } else {
        pack(node, target);
      }
      break;
    case GraphOp::word:
      if (compact(node.inputs[0])) {
        word_words(node, target);
      } else {
        pack(node, target);
      }
      break;
    case GraphOp::round: round_words(node, target); break;
    case GraphOp::saturate: saturate_words(node, target); break;
    case GraphOp::store: throw std::logic_error("Unexpected compact graph store");
    default: throw std::logic_error("Unknown compact graph operation");
  }
}
}
