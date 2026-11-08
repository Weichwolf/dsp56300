// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/value_graph.hpp"

#include <cstdint>

namespace dsp56300::detail::a64 {
void RegionEmitter::constant_words(const Vec target, const std::uint32_t value) {
  if (neon_.constant_words(target, value)) { return; }
  constant(target, std::uint64_t{value} | (std::uint64_t{value} << 32));
}

void RegionEmitter::merge_words(const StickyEvent event, const Vec source) {
  const auto target = event == StickyEvent::growth ? growth_register : limit_register;
  if (packed_events()) {
    neon_.binary(VectorOp::bit_or, target, target, source);
    return;
  }
  for (unsigned part = 0; part < parts_; ++part) {
    neon_.extend(second_scratch, source, part != 0);
    neon_.binary(VectorOp::bit_or, target + part, target + part, second_scratch);
  }
}

void RegionEmitter::merge_wide(const StickyEvent event, const Vec source, const unsigned part) {
  const auto target = event == StickyEvent::growth ? growth_register : limit_register;
  if (!packed_events()) {
    neon_.binary(VectorOp::bit_or, target + part, target + part, source);
    return;
  }
  const auto temporary = target + 1;
  if (part != 0) { neon_.zero(temporary); }
  neon_.narrow(temporary, source, part != 0);
  neon_.binary(VectorOp::bit_or, target, target, temporary);
}

void RegionEmitter::word_words(const GraphNode &node, const Vec target) {
  const auto value = node.inputs[0];
  const auto source = read(value);
  constexpr unsigned growth_position = double_word_bits - 3;
  const auto position = growth_position - value.scale;
  constexpr unsigned word_sign = 31;
  if (value.width > position + 1 && position < word_sign) {
    neon_.shift(first_scratch, source, Shift::right, 1, true);
    neon_.binary(VectorOp::bit_xor, first_scratch, first_scratch, source);
    if (region_.growth_shift == 0) {
      neon_.shift(first_scratch, first_scratch, Shift::right, position, true);
    }
    merge_words(StickyEvent::growth, first_scratch);
  }
  const auto bits = word_bits - value.scale;
  neon_.shift(target, source, Shift::arithmetic, bits, true);
  const auto &bounds = bounds_[value.node];
  const bool lower = (bounds.low >> bits) < minimum_word;
  const bool upper = (bounds.high >> bits) > maximum_word;
  if (!lower && !upper) { return; }
  neon_.move(first_scratch, target);
  if (lower) {
    constant_words(second_scratch, static_cast<std::uint32_t>(minimum_word));
    neon_.binary(VectorOp::maximum, target, target, second_scratch);
  }
  if (upper) {
    constant_words(second_scratch, static_cast<std::uint32_t>(maximum_word));
    neon_.binary(VectorOp::minimum, target, target, second_scratch);
  }
  neon_.binary(VectorOp::bit_xor, first_scratch, first_scratch, target);
  merge_words(StickyEvent::limit, first_scratch);
}

void RegionEmitter::round_words(const GraphNode &node, const Vec target) {
  constexpr Vec temporary{30};
  for (unsigned part = 0; part < parts_; ++part) {
    const auto value = node.inputs[0];
    Vec source = read(value, part);
    if (compact(value) || !value.canonical) {
      canonical(temporary, value, part);
      source = temporary;
    }
    if ((region_.mode & rounding_mode) == 0) {
      neon_.shift(second_scratch, source, Shift::right, node.parameter);
      neon_.binary(VectorOp::bit_and, second_scratch, second_scratch, one_register);
      neon_.binary(VectorOp::subtract, first_scratch, one_register, second_scratch);
      neon_.binary(VectorOp::subtract, second_scratch, source, first_scratch);
      source = second_scratch;
    }
    neon_.round_narrow(target, source, node.parameter, part != 0);
  }
}

}
