// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace dsp56300::detail::a64 {
RegionEmitter::RegionEmitter(ResidentRegion &region, const unsigned lanes)
    : region_(region),
      lanes_(region.nested && lanes == 1 ? 4U : lanes),
      parts_(lanes_ == 4 ? 2U : 1U),
      value_registers_((region.mode & arithmetic_saturation) != 0 ? region_values - 1
                                                                  : region_values),
      inner_vector_(region.nested && lanes == 1),
      bounds_(graph_bounds(region.graph, region.mode)),
      needed_(region.graph.nodes.size()),
      compact_(region.graph.nodes.size()),
      last_(region.graph.nodes.size()),
      locations_(region.graph.nodes.size(), no_value),
      spill_slots_(region.graph.nodes.size(), no_value),
      fused_(region.graph.nodes.size(), no_value) {
  owners_.fill(no_value);
  inputs_.fill(no_value);
  liveness();
  for (std::size_t index = 0; index < compact_.size(); ++index) {
    compact_[index] = bounds_[index].low >= std::numeric_limits<std::int32_t>::min() &&
                      bounds_[index].high <= std::numeric_limits<std::int32_t>::max();
  }
  fuse_products();
}

void RegionEmitter::pin(const GraphValue value) {
  if (!value.valid()) { return; }
  needed_[value.node] = true;
  last_[value.node] = region_.graph.nodes.size();
}

void RegionEmitter::liveness() {
  const auto &graph = region_.graph;
  for (unsigned index = 0; index < graph.registers.size(); ++index) {
    if ((graph.changed_registers & (Word{1} << index)) != 0) { pin(graph.registers[index]); }
  }
  pin(graph.flags);
  pin(graph.overflow_input);
  if (graph.carry != GraphCarry::unchanged && graph.carry != GraphCarry::zero) {
    for (const auto value : graph.carry_inputs) { pin(value); }
  }
  for (auto index = graph.nodes.size(); index != 0; --index) {
    const auto &node = graph.nodes[index - 1];
    needed_[index - 1] = needed_[index - 1] || node.operation == GraphOp::word ||
                         node.operation == GraphOp::store || node.operation == GraphOp::saturate;
    if (!needed_[index - 1]) { continue; }
    for (const auto value : node.inputs) {
      if (!value.valid()) { continue; }
      needed_[value.node] = true;
      last_[value.node] = std::max(last_[value.node], index - 1);
    }
  }
}

void RegionEmitter::spill(const unsigned reg) {
  const auto node = owners_[reg];
  auto &slot = spill_slots_[node];
  if (slot == no_value) {
    slot = region_.spills.size();
    region_.spills.emplace_back();
  }
  code_.load(Reg::x9, Reg::x0, offsetof(RegionFrame, spills), Access::doubleword);
  for (unsigned part = 0; part < (compact_[node] ? 1U : parts_); ++part) {
    neon_.store(static_cast<Vec>((2 * reg) + part),
                Reg::x9,
                (slot * sizeof(RegionVector)) + (std::size_t{part} * 16));
  }
  locations_[node] = no_value;
  owners_[reg] = no_value;
}

unsigned RegionEmitter::allocate() {
  unsigned selected = 0;
  std::size_t farthest = 0;
  bool found = false;
  for (unsigned reg = 0; reg < value_registers_; ++reg) {
    const auto node = owners_[reg];
    if (node == no_value) { return reg; }
    if (std::ranges::find(inputs_, node) != inputs_.end() || node == current_) { continue; }
    if (!found || last_[node] > farthest) {
      selected = reg;
      farthest = last_[node];
      found = true;
    }
  }
  if (!found) { throw std::logic_error("No NEON region spill candidate"); }
  spill(selected);
  return selected;
}

Vec RegionEmitter::read(const GraphValue value, const unsigned part) {
  if (!value.valid()) { throw std::logic_error("Invalid NEON graph input"); }
  auto &location = locations_[value.node];
  if (location != no_value) {
    return static_cast<Vec>((2 * location) + (compact(value) ? 0 : part));
  }
  if (spill_slots_[value.node] == no_value) {
    throw std::logic_error("Undefined NEON graph value");
  }
  const auto reg = allocate();
  location = reg;
  owners_[reg] = value.node;
  code_.load(Reg::x9, Reg::x0, offsetof(RegionFrame, spills), Access::doubleword);
  for (unsigned half = 0; half < (compact(value) ? 1U : parts_); ++half) {
    neon_.load(static_cast<Vec>((2 * reg) + half),
               Reg::x9,
               (spill_slots_[value.node] * sizeof(RegionVector)) + (std::size_t{half} * 16));
  }
  return static_cast<Vec>((2 * reg) + (compact(value) ? 0 : part));
}

Vec RegionEmitter::destination() {
  auto reg = 0U;
  if (fused_[current_] != no_value) {
    const auto input = region_.graph.nodes[current_].inputs[0].node;
    reg = static_cast<unsigned>(locations_[input]);
    locations_[input] = no_value;
  } else {
    reg = allocate();
  }
  locations_[current_] = reg;
  owners_[reg] = current_;
  return static_cast<Vec>(2 * reg);
}

void RegionEmitter::operands(const GraphNode &node) {
  inputs_.fill(no_value);
  for (unsigned index = 0; index < node.inputs.size(); ++index) {
    if (node.inputs[index].valid()) { inputs_[index] = node.inputs[index].node; }
  }
  if (fused_[current_] != no_value) {
    const auto &product = region_.graph.nodes[fused_[current_]];
    inputs_[1] = product.inputs[0].node;
    inputs_[2] = product.inputs[1].node;
  }
  for (const auto input : inputs_) {
    if (input != no_value) { static_cast<void>(read({.node = input})); }
  }
}

void RegionEmitter::constant(const Vec target, const std::uint64_t value) {
  if (value == 1) {
    neon_.move(target, one_register);
    return;
  }
  const std::array values{value, value};
  const auto found = std::ranges::search(region_.constants, values).begin();
  const auto offset = static_cast<std::size_t>(found - region_.constants.begin());
  if (found == region_.constants.end()) {
    region_.constants.insert(region_.constants.end(), values.begin(), values.end());
  }
  neon_.load(target, Reg::x3, offset * sizeof(value));
}

void RegionEmitter::canonical(const Vec target, const GraphValue value, const unsigned part) {
  const auto source = read(value, part);
  if (compact(value)) {
    neon_.extend(target, source, part != 0);
  } else if (value.canonical) {
    neon_.move(target, source);
  } else {
    neon_.shift(target, source, Shift::left, 32);
    neon_.shift(target, target, Shift::arithmetic, 32);
  }
}

Vec RegionEmitter::aligned(const GraphValue value,
                           const unsigned scale,
                           const Vec scratch,
                           const unsigned part) {
  if (!compact(value) && value.canonical && value.scale == scale) { return read(value, part); }
  canonical(scratch, value, part);
  if (value.scale > scale) { neon_.shift(scratch, scratch, Shift::left, value.scale - scale); }
  return scratch;
}

void RegionEmitter::binary(const GraphNode &node, const Vec target, const unsigned part) {
  if (fused_[current_] != no_value) {
    const auto &product = region_.graph.nodes[fused_[current_]];
    auto opcode = part == 0 ? VectorOp::multiply_add : VectorOp::multiply_add_upper;
    if (node.operation == GraphOp::subtract) {
      opcode = part == 0 ? VectorOp::multiply_subtract : VectorOp::multiply_subtract_upper;
    }
    auto left = read(product.inputs[0]);
    const auto shift = node.inputs[1].scale - node.inputs[0].scale;
    if (shift != 0) {
      if (part == 0) { neon_.shift(first_scratch, left, Shift::left, shift, true); }
      left = first_scratch;
    }
    neon_.binary(opcode, target, left, read(product.inputs[1]));
    return;
  }
  const auto scale = std::min(node.inputs[0].scale, node.inputs[1].scale);
  const auto left = aligned(node.inputs[0], scale, first_scratch, part);
  const auto right = aligned(node.inputs[1], scale, second_scratch, part);
  neon_.binary(
      node.operation == GraphOp::add ? VectorOp::add : VectorOp::subtract, target, left, right);
}

void RegionEmitter::round(const GraphNode &node, const Vec target, const unsigned part) {
  canonical(target, node.inputs[0], part);
  const auto bits = node.parameter;
  const auto half = std::uint64_t{1} << (bits - 1);
  if ((region_.mode & rounding_mode) != 0) {
    constant(first_scratch, half);
    neon_.binary(VectorOp::add, target, target, first_scratch);
  } else {
    neon_.shift(second_scratch, target, Shift::right, bits);
    constant(first_scratch, 1);
    neon_.binary(VectorOp::bit_and, second_scratch, second_scratch, first_scratch);
    if (half != 1) {
      constant(first_scratch, half - 1);
      neon_.binary(VectorOp::add, second_scratch, second_scratch, first_scratch);
    }
    neon_.binary(VectorOp::add, target, target, second_scratch);
  }
  neon_.shift(target, target, Shift::arithmetic, bits);
}

void RegionEmitter::word(const GraphNode &node, const Vec target, const unsigned part) {
  const auto value = node.inputs[0];
  canonical(target, value, part);
  const auto source = target;
  constexpr unsigned growth_position = double_word_bits - 3;
  const auto position = growth_position - value.scale;
  if (value.width > position + 1) {
    if (region_.growth_shift != 0) {
      neon_.shift(first_scratch, source, Shift::right, 1);
      neon_.binary(VectorOp::bit_xor, first_scratch, first_scratch, source);
    } else {
      neon_.shift(first_scratch, source, Shift::right, position);
      neon_.shift(second_scratch, source, Shift::right, position + 1);
      neon_.binary(VectorOp::bit_xor, first_scratch, first_scratch, second_scratch);
    }
    merge_wide(StickyEvent::growth, first_scratch, part);
  }
  const auto bits = word_bits - value.scale;
  if (bits != 0) { neon_.shift(target, target, Shift::arithmetic, bits); }
  const auto &bounds = bounds_[value.node];
  const bool lower = (bounds.low >> bits) < minimum_word;
  const bool upper = (bounds.high >> bits) > maximum_word;
  if (!lower && !upper) { return; }
  neon_.move(second_scratch, target);
  if (lower) {
    constant(first_scratch, static_cast<std::uint64_t>(static_cast<std::int64_t>(minimum_word)));
    neon_.binary(VectorOp::maximum, target, target, first_scratch);
  }
  if (upper) {
    constant(first_scratch, static_cast<std::uint64_t>(maximum_word));
    neon_.binary(VectorOp::minimum, target, target, first_scratch);
  }
  neon_.binary(VectorOp::bit_xor, second_scratch, second_scratch, target);
  merge_wide(StickyEvent::limit, second_scratch, part);
}

void RegionEmitter::emit(const GraphNode &node) {
  operands(node);
  if (node.operation == GraphOp::store) {
    if (compact(node.inputs[0])) {
      store_words(node.inputs[0], node.parameter);
      return;
    }
    for (unsigned part = 0; part < parts_; ++part) {
      store(node.inputs[0], {.stream = node.parameter, .part = part});
    }
    return;
  }
  const auto output = destination();
  if (compact_[current_]) {
    emit_words(node, output);
    return;
  }
  for (unsigned part = 0; part < parts_; ++part) {
    const auto target = output + part;
    switch (node.operation) {
      case GraphOp::load: load(target, {.stream = node.parameter, .part = part}); break;
      case GraphOp::invariant:
        code_.load(Reg::x9,
                   Reg::x0,
                   offsetof(RegionFrame, invariants) + (node.parameter * sizeof(std::uint64_t)),
                   Access::doubleword);
        neon_.broadcast(target, Reg::x9);
        break;
      case GraphOp::zero: neon_.zero(target); break;
      case GraphOp::add:
      case GraphOp::subtract: binary(node, target, part); break;
      case GraphOp::multiply: multiply(node, target, part); break;
      case GraphOp::negate:
        canonical(target, node.inputs[0], part);
        neon_.zero(first_scratch);
        neon_.binary(VectorOp::subtract, target, first_scratch, target);
        break;
      case GraphOp::shift:
        canonical(target, node.inputs[0], part);
        neon_.shift(target, target, Shift::arithmetic, 1);
        break;
      case GraphOp::round: round(node, target, part); break;
      case GraphOp::word: word(node, target, part); break;
      case GraphOp::saturate: saturate(node, target, part); break;
      case GraphOp::store: break;
      default: throw std::logic_error("Unknown NEON graph operation");
    }
  }
}

void RegionEmitter::publish(const GraphValue value, const ValueOutput output) {
  const auto lane = output.lane;
  if (!value.valid()) { return; }
  const bool packed = compact(value);
  const auto location = locations_[value.node];
  if (location == no_value) {
    const auto slot = spill_slots_[value.node];
    if (slot == no_value) { throw std::logic_error("Undefined published NEON graph value"); }
    code_.load(Reg::x9, Reg::x0, offsetof(RegionFrame, spills), Access::doubleword);
    const auto element_bytes = packed ? sizeof(Word) : sizeof(std::uint64_t);
    code_.constant(Reg::x10, (slot * sizeof(RegionVector)) + (lane * element_bytes));
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x10, Width::x64);
    code_.load(Reg::x9, Reg::x9, 0, packed ? Access::word : Access::doubleword);
  } else if (packed) {
    neon_.extract(Reg::x9, static_cast<Vec>(2 * location), lane, true);
  } else {
    neon_.extract(Reg::x9, static_cast<Vec>((2 * location) + (lane / 2)), lane & 1U);
  }
  if (packed) { code_.extract(Reg::x9, Reg::x9, 0, 32, Width::x64, true); }
  code_.store(Reg::x9, Reg::x0, output.offset, Access::doubleword);
}

void RegionEmitter::outputs(const unsigned lane, const bool preceding) {
  const auto base =
      preceding ? offsetof(RegionFrame, preceding_flags) : offsetof(RegionFrame, flags);
  publish(region_.graph.flags, {.offset = base, .lane = lane});
  publish(region_.graph.overflow_input,
          {.offset = base + (3 * sizeof(std::uint64_t)), .lane = lane});
  if (region_.graph.carry == GraphCarry::unchanged || region_.graph.carry == GraphCarry::zero) {
    return;
  }
  for (unsigned index = 0; index < region_.graph.carry_inputs.size(); ++index) {
    publish(region_.graph.carry_inputs[index],
            {.offset = base + ((index + 1) * sizeof(std::uint64_t)), .lane = lane});
  }
}
}
