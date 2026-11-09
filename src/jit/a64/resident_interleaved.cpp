// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace dsp56300::detail::a64 {
std::optional<RegionQuad>
RegionEmitter::interleaved_root(const std::size_t root,
                                const std::vector<std::size_t> &order) const {
  const auto address = region_.roots[root].address;
  const auto &delta = region_.graph.addresses[address];
  if (region_.prefix[address] || (delta.units != 4 && delta.units != 8) || delta.offsets != 0) {
    return std::nullopt;
  }
  RegionQuad result{.fields = delta.units};
  result.nodes.fill(no_value);
  for (unsigned stream = 0; stream < region_.graph.streams.size(); ++stream) {
    if (region_.stream_roots[stream] != root) { continue; }
    const auto &input = region_.graph.streams[stream];
    if (input.units >= result.fields || input.offsets != 0 || order[stream] == no_value) {
      return std::nullopt;
    }
    if (result.nodes[input.units] != no_value) { return std::nullopt; }
    result.nodes[input.units] = order[stream];
  }
  if (std::ranges::find(result.members(), no_value) != result.members().end()) {
    return std::nullopt;
  }
  const auto kind = region_.graph.nodes[result.nodes[0]].operation;
  result.store = kind == GraphOp::store;
  for (const auto index : result.members()) {
    const auto &node = region_.graph.nodes[index];
    if (!needed_[index] || node.operation != kind ||
        !(result.store ? compact(node.inputs[0]) : compact_[index])) {
      return std::nullopt;
    }
  }
  return result;
}

void RegionEmitter::plan_interleaved() {
  if (lanes_ != 4 || region_.nested) { return; }
  if (region_.interleaved.empty()) {
    std::vector<std::size_t> order(region_.graph.streams.size(), no_value);
    for (std::size_t index = 0; index < region_.graph.nodes.size(); ++index) {
      const auto &node = region_.graph.nodes[index];
      if (node.operation != GraphOp::load && node.operation != GraphOp::store) { continue; }
      if (order[node.parameter] != no_value) { return; }
      order[node.parameter] = index;
    }
    for (std::size_t root = 0; root < region_.roots.size(); ++root) {
      if (auto group = interleaved_root(root, order)) { region_.interleaved.push_back(*group); }
    }
    if (!region_.interleaved.empty()) { region_.memory_order = std::move(order); }
  }
  for (std::size_t index = 0; index < region_.interleaved.size(); ++index) {
    const auto &group = region_.interleaved[index];
    const auto last = std::ranges::max(group.members());
    for (const auto node : group.members()) {
      interleaved_[node] = index;
      if (group.store) {
        const auto input = region_.graph.nodes[node].inputs[0];
        last_[input.node] = std::max(last_[input.node], last);
      }
    }
  }
}

void RegionEmitter::preserve_growth(const bool restore) {
  const unsigned count = packed_events() ? 1U : parts_;
  for (unsigned part = 0; part < count; ++part) {
    const auto offset = offsetof(RegionFrame, growth) + (std::size_t{part} * 16);
    if (restore) {
      neon_.load(growth_register + part, Reg::x0, offset);
    } else {
      neon_.store(growth_register + part, Reg::x0, offset);
    }
  }
}

void RegionEmitter::load_interleaved(const RegionQuad &group) {
  inputs_.fill(no_value);
  const auto original = current_;
  const auto stream = region_.graph.nodes[group.nodes[0]].parameter;
  pointer({.stream = stream, .part = 0});
  preserve_growth(false);
  neon_.load_interleaved(first_scratch, Reg::x10);
  for (unsigned column = 0; column < group.fields; ++column) {
    current_ = group.nodes[column];
    const auto target = destination();
    if (group.fields == 8) {
      neon_.move(target, first_scratch + (column % 4));
    } else {
      neon_.shift(target, first_scratch + column, Shift::left, padding_bits, true);
      neon_.shift(target, target, Shift::arithmetic, padding_bits, true);
    }
  }
  if (group.fields == 8) {
    constexpr unsigned half_bytes = 64;
    code_.add(Reg::x10, Reg::x10, half_bytes, Width::x64);
    neon_.load_interleaved(first_scratch, Reg::x10);
    for (unsigned column = 0; column < group.fields; ++column) {
      const auto node = group.nodes[column];
      inputs_.fill(no_value);
      inputs_[0] = node;
      const auto target = read({.node = node});
      neon_.binary(column < 4 ? VectorOp::unzip_even : VectorOp::unzip_odd,
                   target,
                   target,
                   first_scratch + (column % 4));
      neon_.shift(target, target, Shift::left, padding_bits, true);
      neon_.shift(target, target, Shift::arithmetic, padding_bits, true);
    }
  }
  preserve_growth(true);
  current_ = original;
}

void RegionEmitter::store_interleaved(const RegionQuad &group) {
  preserve_growth(false);
  const auto stream = region_.graph.nodes[group.nodes[0]].parameter;
  for (unsigned half = 0; half < group.fields / 4; ++half) {
    for (unsigned column = 0; column < 4; ++column) {
      const auto first = region_.graph.nodes[group.nodes[column]].inputs[0];
      inputs_.fill(no_value);
      inputs_[0] = first.node;
      if (group.fields == 8) {
        const auto second = region_.graph.nodes[group.nodes[column + 4]].inputs[0];
        inputs_[1] = second.node;
        neon_.binary(half == 0 ? VectorOp::zip_low : VectorOp::zip_high,
                     first_scratch + column,
                     read(first),
                     read(second));
      } else {
        neon_.move(first_scratch + column, read(first));
      }
      neon_.shift(first_scratch + column, first_scratch + column, Shift::left, padding_bits, true);
      neon_.shift(first_scratch + column, first_scratch + column, Shift::right, padding_bits, true);
    }
    pointer({.stream = stream, .part = 0});
    if (half != 0) {
      constexpr unsigned half_bytes = 64;
      code_.add(Reg::x10, Reg::x10, half_bytes, Width::x64);
    }
    neon_.store_interleaved(first_scratch, Reg::x10);
  }
  preserve_growth(true);
}

void RegionEmitter::emit_interleaved(const RegionQuad &group) {
  const auto leader =
      group.store ? std::ranges::max(group.members()) : std::ranges::min(group.members());
  if (current_ != leader) { return; }
  if (group.store) {
    store_interleaved(group);
  } else {
    load_interleaved(group);
  }
}
}
