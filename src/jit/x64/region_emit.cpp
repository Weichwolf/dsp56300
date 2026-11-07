// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/code.hpp"
#include "region.hpp"
#include "value_graph.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::VectorOp;
using x64::VectorShift;
using x64::Width;
using x64::Xmm;
constexpr Xmm growth_register = Xmm::x12, limit_register = Xmm::x13;
constexpr Xmm first_scratch = Xmm::x14, second_scratch = Xmm::x15;
constexpr unsigned value_registers = 12, group_add = 0, group_and = 4, group_subtract = 5,
                   group_compare = 7;
constexpr unsigned op_add = 0x01, op_and = 0x21, push_register = 0x50, pop_register = 0x58,
                   high_rex = 0x41;
constexpr unsigned growth_position = double_word_bits - 3, canonical_signs = 0xa0,
                   canonical_high = 0xaa;
constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr unsigned return_opcode = 0xc3, repeat_word_pair = 0x44;
constexpr std::array root_registers{Reg::bx, Reg::bp, Reg::r12, Reg::r13, Reg::r14, Reg::r15};

std::size_t cursor_offset(const std::size_t root, const std::size_t member) {
  return offsetof(RegionFrame, cursors) + (root * sizeof(RegionCursor)) + member;
}

bool possible_unit_stride(const GraphAddress delta) {
  return delta.units <= 1 && (delta.units == 1 || delta.offsets == 1);
}

struct ValueOutput {
  std::size_t offset;
  unsigned lane;
};

class RegionEmitter {
public:
  RegionEmitter(NativeRegion &region, unsigned lanes);
  [[nodiscard]] std::vector<std::uint8_t> generate();

private:
  void liveness();
  void pin(GraphValue value);
  void operands(const GraphNode &node);
  [[nodiscard]] Xmm allocate();
  [[nodiscard]] Xmm read(GraphValue value);
  [[nodiscard]] Xmm destination();
  void spill(unsigned reg);
  void broadcast(Xmm result, std::uint64_t value);
  void vector_constant(Xmm result, const std::array<std::uint64_t, 4> &values);
  void reverse_load(Xmm result, unsigned stream);
  void reverse_indices(Xmm result, unsigned stream);
  void advance_reverse();
  void zero(Xmm result);
  void canonical(Xmm result, GraphValue value);
  [[nodiscard]] Xmm aligned(GraphValue value, unsigned scale, Xmm scratch);
  void pointer(unsigned stream);
  void load(Xmm result, unsigned stream);
  void store(GraphValue value, unsigned stream);
  void store_packed(Xmm source);
  void store_strided(Xmm source, std::size_t root, bool stride_loaded);
  void input(Xmm result, unsigned slot);
  void emit(const GraphNode &node);
  void binary(const GraphNode &node, Xmm result);
  void round(const GraphNode &node, Xmm result);
  void word(const GraphNode &node, Xmm result);
  void shift(const GraphNode &node, Xmm result);
  void publish(GraphValue value, ValueOutput output);
  void outputs(unsigned lane, bool preceding);
  [[nodiscard]] bool advance_known(std::size_t root, bool inner);
  void advance(bool inner);
  void push(Reg reg);
  void pop(Reg reg);
  NativeRegion &region_;
  x64::Code code_;
  unsigned lanes_;
  bool inner_vector_;
  std::size_t current_ = 0;
  std::vector<GraphBounds> bounds_;
  std::vector<bool> needed_;
  std::vector<std::size_t> last_, locations_, spill_slots_;
  std::array<std::size_t, value_registers> owners_{};
  std::array<std::size_t, 2> inputs_{};
};

RegionEmitter::RegionEmitter(NativeRegion &region, const unsigned lanes)
    : region_(region),
      lanes_(region.nested && lanes == 1 ? 4U : lanes),
      inner_vector_(region.nested && lanes == 1),
      bounds_(graph_bounds(region.graph, region.mode)),
      needed_(region.graph.nodes.size()),
      last_(region.graph.nodes.size()),
      locations_(region.graph.nodes.size(), no_value),
      spill_slots_(region.graph.nodes.size(), no_value) {
  owners_.fill(no_value);
  inputs_.fill(no_value);
  liveness();
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
  if (graph.carry != GraphCarry::unchanged && graph.carry != GraphCarry::zero) {
    for (const auto value : graph.carry_inputs) { pin(value); }
  }
  for (auto index = graph.nodes.size(); index != 0; --index) {
    const auto &node = graph.nodes[index - 1];
    needed_[index - 1] =
        needed_[index - 1] || node.operation == GraphOp::word || node.operation == GraphOp::store;
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
  code_.load(Reg::ax, Reg::di, offsetof(RegionFrame, spills));
  code_.vector_store({.base = Reg::ax, .offset = slot * sizeof(RegionVector)},
                     static_cast<Xmm>(reg));
  locations_[node] = no_value;
  owners_[reg] = no_value;
}

Xmm RegionEmitter::allocate() {
  unsigned selected = 0;
  std::size_t farthest = 0;
  bool found = false;
  for (unsigned reg = 0; reg < owners_.size(); ++reg) {
    const auto node = owners_[reg];
    if (node == no_value) { return static_cast<Xmm>(reg); }
    if (node == inputs_[0] || node == inputs_[1] || node == current_) { continue; }
    if (!found || last_[node] > farthest) {
      selected = reg;
      farthest = last_[node];
      found = true;
    }
  }
  if (!found) { throw std::logic_error("No AVX2 region spill candidate"); }
  spill(selected);
  return static_cast<Xmm>(selected);
}

Xmm RegionEmitter::read(const GraphValue value) {
  if (!value.valid()) { throw std::logic_error("Invalid native graph input"); }
  auto &location = locations_[value.node];
  if (location != no_value) { return static_cast<Xmm>(location); }
  if (spill_slots_[value.node] == no_value) { throw std::logic_error("Undefined graph value"); }
  const auto reg = allocate();
  location = static_cast<unsigned>(reg);
  owners_[location] = value.node;
  code_.load(Reg::ax, Reg::di, offsetof(RegionFrame, spills));
  code_.vector_load(reg,
                    {.base = Reg::ax, .offset = spill_slots_[value.node] * sizeof(RegionVector)});
  return reg;
}

Xmm RegionEmitter::destination() {
  const auto reg = allocate();
  locations_[current_] = static_cast<unsigned>(reg);
  owners_[static_cast<unsigned>(reg)] = current_;
  return reg;
}

void RegionEmitter::operands(const GraphNode &node) {
  inputs_.fill(no_value);
  for (unsigned index = 0; index < node.inputs.size(); ++index) {
    if (node.inputs[index].valid()) { inputs_[index] = node.inputs[index].node; }
  }
  for (const auto input : node.inputs) {
    if (input.valid()) { static_cast<void>(read(input)); }
  }
}

void RegionEmitter::broadcast(const Xmm result, const std::uint64_t value) {
  const auto found = std::ranges::find(region_.constants, value);
  const auto index = static_cast<std::size_t>(found - region_.constants.begin());
  if (found == region_.constants.end()) { region_.constants.push_back(value); }
  code_.vector_broadcast(result, {.base = Reg::r11, .offset = index * sizeof(value)});
}

void RegionEmitter::vector_constant(const Xmm result, const std::array<std::uint64_t, 4> &values) {
  const auto found = std::ranges::search(region_.constants, values).begin();
  const auto index = static_cast<std::size_t>(found - region_.constants.begin());
  if (found == region_.constants.end()) {
    region_.constants.insert(region_.constants.end(), values.begin(), values.end());
  }
  code_.vector_load(result, {.base = Reg::r11, .offset = index * sizeof(std::uint64_t)});
}

void RegionEmitter::reverse_indices(const Xmm result, const unsigned stream) {
  const auto &address = region_.graph.streams[stream];
  const auto base = offsetof(RegionFrame, reverse) + (address.address * sizeof(ReverseCursor));
  code_.vector_broadcast(result, {.base = Reg::di, .offset = base});
  std::array<std::uint64_t, 4> displacement{};
  for (unsigned lane = 0; lane < displacement.size(); ++lane) {
    displacement[lane] =
        std::uint64_t{lane % lanes_} * region_.graph.addresses[address.address].offsets +
        address.offsets;
  }
  vector_constant(second_scratch, displacement);
  code_.vector(VectorOp::add, {.destination = result, .left = result, .right = second_scratch});
  code_.vector_broadcast(
      first_scratch, {.base = Reg::di, .offset = base + offsetof(ReverseCursor, mask)}, true);
  code_.vector(VectorOp::bit_and, {.destination = result, .left = result, .right = first_scratch});
  constexpr unsigned nibble_bits = 4;
  code_.vector_shift(second_scratch, result, VectorShift::right, nibble_bits, true);
  constexpr std::uint64_t nibble_mask = 0x0f0f0f0f0f0f0f0f;
  broadcast(first_scratch, nibble_mask);
  code_.vector(VectorOp::bit_and, {.destination = result, .left = result, .right = first_scratch});
  code_.vector(VectorOp::bit_and,
               {.destination = second_scratch, .left = second_scratch, .right = first_scratch});
  constexpr std::array<std::uint64_t, 4> reversed_nibbles{
      0x0e060a020c040800, 0x0f070b030d050901, 0x0e060a020c040800, 0x0f070b030d050901};
  vector_constant(first_scratch, reversed_nibbles);
  code_.vector(VectorOp::shuffle_bytes,
               {.destination = result, .left = first_scratch, .right = result});
  code_.vector(VectorOp::shuffle_bytes,
               {.destination = second_scratch, .left = first_scratch, .right = second_scratch});
  code_.vector_shift(result, result, VectorShift::left, nibble_bits, true);
  code_.vector(VectorOp::bit_or, {.destination = result, .left = result, .right = second_scratch});
  constexpr std::array<std::uint64_t, 4> reversed_bytes{
      0x0405060700010203, 0x0c0d0e0f08090a0b, 0x0405060700010203, 0x0c0d0e0f08090a0b};
  vector_constant(first_scratch, reversed_bytes);
  code_.vector(VectorOp::shuffle_bytes,
               {.destination = result, .left = result, .right = first_scratch});
  code_.vector_load_dword(first_scratch,
                          {.base = Reg::di, .offset = base + offsetof(ReverseCursor, shift)});
  code_.vector(VectorOp::shift_right_qword,
               {.destination = result, .left = result, .right = first_scratch});
}

void RegionEmitter::reverse_load(const Xmm result, const unsigned stream) {
  reverse_indices(result, stream);
  zero(first_scratch);
  for (unsigned lane = 0; lane < 4; ++lane) {
    if (lane == 2) { code_.vector_extract(second_scratch, result, 1); }
    code_.vector_scalar(Reg::dx, lane < 2 ? result : second_scratch, lane & 1U);
    code_.vector_insert_dword(first_scratch, {.base = Reg::r10, .index = Reg::dx}, lane);
  }
  code_.vector_unary(VectorOp::copy, result, first_scratch);
}

void RegionEmitter::zero(const Xmm result) {
  code_.vector(VectorOp::bit_xor, {.destination = result, .left = result, .right = result});
}

void RegionEmitter::canonical(const Xmm result, const GraphValue value) {
  const auto source = read(value);
  if (value.canonical) {
    code_.vector_unary(VectorOp::copy, result, source);
    return;
  }
  constexpr unsigned dword_sign = 31;
  code_.vector_shift(result, source, VectorShift::arithmetic, dword_sign, true);
  code_.vector_immediate(VectorOp::shuffle_dword,
                         {.destination = result, .left = Xmm::x0, .right = result},
                         canonical_signs);
  code_.vector_immediate(VectorOp::blend_dword,
                         {.destination = result, .left = source, .right = result},
                         canonical_high);
}

Xmm RegionEmitter::aligned(const GraphValue value, const unsigned scale, const Xmm scratch) {
  const auto source = read(value);
  if (value.canonical && value.scale == scale) { return source; }
  canonical(scratch, value);
  if (value.scale > scale) {
    code_.vector_shift(scratch, scratch, VectorShift::left, value.scale - scale);
  }
  return scratch;
}

void RegionEmitter::pointer(const unsigned stream) {
  const auto root = region_.stream_roots[stream];
  if (root < root_registers.size()) {
    code_.mov({.destination = Reg::r10, .source = root_registers[root]});
  } else {
    code_.load(Reg::r10, Reg::di, cursor_offset(root, offsetof(RegionCursor, pointer)));
  }
  const auto &address = region_.graph.streams[stream];
  if ((region_.reverse_addresses & (Word{1} << address.address)) != 0) { return; }
  if (address.offsets != 0) {
    code_.load(Reg::r9, Reg::di, offsetof(RegionFrame, offsets));
    code_.load(Reg::r9, Reg::r9, stream * sizeof(std::uint64_t));
    code_.binary(op_add, {.destination = Reg::r10, .source = Reg::r9});
  } else if (address.units != 0) {
    code_.immediate(group_add, Reg::r10, static_cast<Word>(address.units * sizeof(Word)));
  }
}

void RegionEmitter::load(const Xmm result, const unsigned stream) {
  const auto root = region_.stream_roots[stream];
  const auto &address = region_.roots[root];
  const auto &delta = inner_vector_ ? region_.inner_addresses[address.address]
                                    : region_.graph.addresses[address.address];
  pointer(stream);
  if ((region_.reverse_addresses & (Word{1} << address.address)) != 0) {
    reverse_load(result, stream);
  } else if (lanes_ == 1 || (!inner_vector_ && region_.prefix[address.address]) ||
             (delta.units == 0 && delta.offsets == 0)) {
    code_.vector_broadcast(result, {.base = Reg::r10}, true);
  } else if (lanes_ == 4 && delta.units == 1 && delta.offsets == 0) {
    code_.vector_load(result, {.base = Reg::r10}, false);
  } else {
    code_.load(Reg::r8,
               Reg::di,
               cursor_offset(root,
                             inner_vector_ ? offsetof(RegionCursor, inner_stride)
                                           : offsetof(RegionCursor, stride)));
    x64::Label strided;
    x64::Label loaded;
    if (lanes_ == 4 && possible_unit_stride(delta)) {
      code_.immediate(group_compare, Reg::r8, 1);
      code_.jump_if(x64::Branch::not_equal, strided);
      code_.vector_load(result, {.base = Reg::r10}, false);
      code_.jump(loaded);
      code_.bind(strided);
    }
    code_.mov({.destination = Reg::dx, .source = Reg::r8});
    code_.vector_load_dword(result, {.base = Reg::r10});
    code_.vector_insert_dword(result, {.base = Reg::r10, .index = Reg::dx}, 1);
    if (lanes_ == 2) {
      code_.vector_immediate(VectorOp::shuffle_dword,
                             {.destination = result, .left = Xmm::x0, .right = result},
                             repeat_word_pair);
    } else {
      for (unsigned lane = 2; lane < lanes_; ++lane) {
        code_.binary(op_add, {.destination = Reg::dx, .source = Reg::r8});
        code_.vector_insert_dword(result, {.base = Reg::r10, .index = Reg::dx}, lane);
      }
    }
    code_.bind(loaded);
  }
  code_.vector_shift(result, result, VectorShift::left, padding_bits, true);
  code_.vector_shift(result, result, VectorShift::arithmetic, padding_bits, true);
  code_.vector_unary(VectorOp::extend_dword, result, result);
}

void RegionEmitter::store(const GraphValue value, const unsigned stream) {
  const auto source = read(value);
  const auto root = region_.stream_roots[stream];
  const auto address = region_.roots[root].address;
  const auto &delta =
      inner_vector_ ? region_.inner_addresses[address] : region_.graph.addresses[address];
  pointer(stream);
  const bool fixed_unit = delta.units == 1 && delta.offsets == 0;
  const bool probe_unit = lanes_ >= 2 && !fixed_unit && possible_unit_stride(delta);
  x64::Label strided;
  x64::Label stored;
  if (probe_unit) {
    code_.load(Reg::r8,
               Reg::di,
               cursor_offset(root,
                             inner_vector_ ? offsetof(RegionCursor, inner_stride)
                                           : offsetof(RegionCursor, stride)));
    code_.immediate(group_compare, Reg::r8, 1);
    code_.jump_if(x64::Branch::not_equal, strided);
  }
  if (lanes_ >= 2 && (fixed_unit || probe_unit)) {
    store_packed(source);
    if (fixed_unit) { return; }
    code_.jump(stored);
    code_.bind(strided);
  }
  store_strided(source, root, probe_unit);
  code_.bind(stored);
}

void RegionEmitter::store_packed(const Xmm source) {
  code_.vector_extract(second_scratch, source, 1);
  code_.vector_pack(first_scratch, source, second_scratch);
  code_.vector_shift(first_scratch, first_scratch, VectorShift::left, padding_bits, true);
  code_.vector_shift(first_scratch, first_scratch, VectorShift::right, padding_bits, true);
  if (lanes_ == 4) {
    code_.vector_store({.base = Reg::r10}, first_scratch, false);
  } else {
    code_.vector_scalar(Reg::ax, first_scratch, 0);
    code_.store(Reg::r10, 0, Reg::ax);
  }
}

void RegionEmitter::store_strided(const Xmm source,
                                  const std::size_t root,
                                  const bool stride_loaded) {
  auto words = source;
  if (lanes_ == 4) {
    code_.vector_shift(first_scratch, source, VectorShift::left, padding_bits, true);
    code_.vector_shift(first_scratch, first_scratch, VectorShift::right, padding_bits, true);
    words = first_scratch;
  }
  if (!stride_loaded) {
    code_.load(Reg::r8,
               Reg::di,
               cursor_offset(root,
                             inner_vector_ ? offsetof(RegionCursor, inner_stride)
                                           : offsetof(RegionCursor, stride)));
  }
  code_.mov({.destination = Reg::dx, .source = Reg::r8});
  for (unsigned lane = 0; lane < lanes_; ++lane) {
    if (lane == 2) { code_.vector_extract(second_scratch, words, 1); }
    code_.vector_scalar(Reg::ax, lane < 2 ? words : second_scratch, lane & 1U);
    if (lanes_ != 4) { code_.immediate(group_and, Reg::ax, word_mask, Width::dword); }
    if (lane == 0) {
      code_.store(Reg::r10, 0, Reg::ax, Width::dword);
    } else {
      code_.store_indexed({.base = Reg::r10, .index = Reg::dx}, Reg::ax);
      code_.binary(op_add, {.destination = Reg::dx, .source = Reg::r8});
    }
  }
}

void RegionEmitter::input(const Xmm result, const unsigned slot) {
  code_.vector_broadcast(
      result,
      {.base = Reg::di,
       .offset = offsetof(RegionFrame, invariants) + (slot * sizeof(std::uint64_t))});
}

void RegionEmitter::binary(const GraphNode &node, const Xmm result) {
  const auto &left = node.inputs[0];
  const auto &right = node.inputs[1];
  const auto scale = std::min(left.scale, right.scale);
  const auto a = aligned(left, scale, first_scratch);
  const auto b = aligned(right, scale, second_scratch);
  code_.vector(node.operation == GraphOp::add ? VectorOp::add : VectorOp::subtract,
               {.destination = result, .left = a, .right = b});
}

void RegionEmitter::round(const GraphNode &node, const Xmm result) {
  auto source = read(node.inputs[0]);
  if (!node.inputs[0].canonical) {
    canonical(result, node.inputs[0]);
    source = result;
  }
  const auto bits = node.parameter;
  const auto half = std::uint64_t{1} << (bits - 1);
  if ((region_.mode & rounding_mode) != 0) {
    broadcast(first_scratch, half);
    code_.vector(VectorOp::add, {.destination = result, .left = source, .right = first_scratch});
  } else {
    code_.vector_shift(second_scratch, source, VectorShift::right, bits);
    broadcast(first_scratch, 1);
    code_.vector(VectorOp::bit_and,
                 {.destination = second_scratch, .left = second_scratch, .right = first_scratch});
    if (half != 1) {
      broadcast(first_scratch, half - 1);
      code_.vector(VectorOp::add,
                   {.destination = second_scratch, .left = second_scratch, .right = first_scratch});
    }
    code_.vector(VectorOp::add, {.destination = result, .left = source, .right = second_scratch});
  }
  code_.vector_shift(result, result, VectorShift::right, bits);
}

void RegionEmitter::word(const GraphNode &node, const Xmm result) {
  const auto value = node.inputs[0];
  const auto source = read(value);
  const auto position = growth_position - value.scale;
  if (value.width > position + 1) {
    if (region_.growth_shift != 0) {
      code_.vector_shift(first_scratch, source, VectorShift::right, 1);
      code_.vector(VectorOp::bit_xor,
                   {.destination = first_scratch, .left = first_scratch, .right = source});
    } else {
      code_.vector_shift(first_scratch, source, VectorShift::right, position);
      code_.vector_shift(second_scratch, source, VectorShift::right, position + 1);
      code_.vector(VectorOp::bit_xor,
                   {.destination = first_scratch, .left = first_scratch, .right = second_scratch});
    }
    code_.vector(VectorOp::bit_or,
                 {.destination = growth_register, .left = growth_register, .right = first_scratch});
  }
  const auto bits = word_bits - value.scale;
  const auto &bounds = bounds_[value.node];
  const bool lower = (bounds.low >> bits) < minimum_word;
  const bool upper = (bounds.high >> bits) > maximum_word;
  auto clamped = source;
  if (bits != 0) {
    canonical(result, value);
    code_.vector_shift(result, result, VectorShift::right, bits);
    clamped = result;
  }
  if (!lower && !upper) {
    if (bits == 0) { code_.vector_unary(VectorOp::copy, result, source); }
    return;
  }
  code_.vector_unary(VectorOp::copy, second_scratch, clamped);
  if (lower) {
    broadcast(first_scratch, static_cast<std::uint64_t>(static_cast<std::int64_t>(minimum_word)));
    code_.vector(VectorOp::maximum_dword,
                 {.destination = result, .left = clamped, .right = first_scratch});
    clamped = result;
  }
  if (upper) {
    broadcast(first_scratch, static_cast<std::uint64_t>(maximum_word));
    code_.vector(VectorOp::minimum_dword,
                 {.destination = result, .left = clamped, .right = first_scratch});
  }
  code_.vector(VectorOp::bit_xor,
               {.destination = second_scratch, .left = second_scratch, .right = result});
  code_.vector(VectorOp::bit_or,
               {.destination = limit_register, .left = limit_register, .right = second_scratch});
}

void RegionEmitter::shift(const GraphNode &node, const Xmm result) {
  const auto value = node.inputs[0];
  const auto source = read(value);
  if (value.width <= 32) {
    code_.vector_shift(result, source, VectorShift::arithmetic, 1, true);
    return;
  }
  zero(first_scratch);
  code_.vector(VectorOp::greater_qword,
               {.destination = second_scratch, .left = first_scratch, .right = source});
  code_.vector_shift(second_scratch, second_scratch, VectorShift::left, host_sign_bit);
  code_.vector_shift(result, source, VectorShift::right, 1);
  code_.vector(VectorOp::bit_or, {.destination = result, .left = result, .right = second_scratch});
}

void RegionEmitter::emit(const GraphNode &node) {
  operands(node);
  if (node.operation == GraphOp::store) {
    store(node.inputs[0], node.parameter);
    return;
  }
  const auto result = destination();
  switch (node.operation) {
    case GraphOp::load: load(result, node.parameter); break;
    case GraphOp::invariant: input(result, node.parameter); break;
    case GraphOp::zero: zero(result); break;
    case GraphOp::add:
    case GraphOp::subtract: binary(node, result); break;
    case GraphOp::multiply:
      code_.vector(
          VectorOp::multiply,
          {.destination = result, .left = read(node.inputs[0]), .right = read(node.inputs[1])});
      break;
    case GraphOp::negate:
      canonical(result, node.inputs[0]);
      zero(first_scratch);
      code_.vector(VectorOp::subtract,
                   {.destination = result, .left = first_scratch, .right = result});
      break;
    case GraphOp::shift: shift(node, result); break;
    case GraphOp::round: round(node, result); break;
    case GraphOp::word: word(node, result); break;
    case GraphOp::store: break;
    default: throw std::logic_error("Unknown native graph operation");
  }
}

void RegionEmitter::publish(const GraphValue value, const ValueOutput output) {
  if (!value.valid()) { return; }
  const auto lane = output.lane;
  const auto location = locations_[value.node];
  if (location == no_value) {
    const auto slot = spill_slots_[value.node];
    if (slot == no_value) { throw std::logic_error("Undefined published graph value"); }
    code_.load(Reg::ax, Reg::di, offsetof(RegionFrame, spills));
    code_.load(Reg::ax, Reg::ax, (slot * sizeof(RegionVector)) + (lane * sizeof(std::uint64_t)));
  } else {
    const auto source = static_cast<Xmm>(location);
    auto selected = source;
    if (lane >= 2) {
      code_.vector_extract(second_scratch, source, 1);
      selected = second_scratch;
    }
    code_.vector_scalar(Reg::ax, selected, lane & 1U);
  }
  code_.store(Reg::di, output.offset, Reg::ax);
}

void RegionEmitter::outputs(const unsigned lane, const bool preceding) {
  const auto base =
      preceding ? offsetof(RegionFrame, preceding_flags) : offsetof(RegionFrame, flags);
  publish(region_.graph.flags, {.offset = base, .lane = lane});
  if (region_.graph.carry != GraphCarry::unchanged && region_.graph.carry != GraphCarry::zero) {
    for (unsigned index = 0; index < region_.graph.carry_inputs.size(); ++index) {
      publish(region_.graph.carry_inputs[index],
              {.offset = base + ((index + 1) * sizeof(std::uint64_t)), .lane = lane});
    }
  }
}

bool RegionEmitter::advance_known(const std::size_t root, const bool inner) {
  const auto address = region_.roots[root].address;
  const auto &delta = inner ? region_.inner_addresses[address] : region_.graph.addresses[address];
  if (delta.offsets != 0 || (!inner && region_.nested)) { return false; }
  const auto bytes =
      std::uint64_t{delta.units} * sizeof(Word) * (inner && !inner_vector_ ? 1U : lanes_);
  if (bytes == 0) { return true; }
  if (root < root_registers.size()) {
    code_.immediate(group_add, root_registers[root], static_cast<Word>(bytes));
  } else {
    code_.memory_immediate(group_add,
                           Reg::di,
                           cursor_offset(root, offsetof(RegionCursor, pointer)),
                           Width::qword,
                           static_cast<Word>(bytes));
  }
  return true;
}

void RegionEmitter::advance_reverse() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    if ((region_.reverse_addresses & (Word{1} << index)) == 0) { continue; }
    const auto base = offsetof(RegionFrame, reverse) + (index * sizeof(ReverseCursor));
    code_.load(Reg::ax, Reg::di, base, Width::dword);
    code_.immediate(
        group_add, Reg::ax, region_.graph.addresses[index].offsets * lanes_, Width::dword);
    code_.load(Reg::r8, Reg::di, base + offsetof(ReverseCursor, mask), Width::dword);
    code_.binary(op_and, {.destination = Reg::ax, .source = Reg::r8}, Width::dword);
    code_.store(Reg::di, base, Reg::ax);
  }
}

void RegionEmitter::advance(const bool inner) {
  if (!inner) { advance_reverse(); }
  for (std::size_t root = 0; root < region_.roots.size(); ++root) {
    if ((inner && !region_.nested) ||
        (region_.reverse_addresses & (Word{1} << region_.roots[root].address)) != 0) {
      continue;
    }
    if (advance_known(root, inner)) { continue; }
    const auto member =
        inner ? offsetof(RegionCursor, inner_step_bytes) : offsetof(RegionCursor, step_bytes);
    code_.load(Reg::ax, Reg::di, cursor_offset(root, member));
    if (inner && inner_vector_) { code_.shift(4, Reg::ax, 2); }
    if (root < root_registers.size()) {
      code_.binary(op_add, {.destination = root_registers[root], .source = Reg::ax});
    } else {
      code_.load(Reg::dx, Reg::di, cursor_offset(root, offsetof(RegionCursor, pointer)));
      code_.binary(op_add, {.destination = Reg::dx, .source = Reg::ax});
      code_.store(Reg::di, cursor_offset(root, offsetof(RegionCursor, pointer)), Reg::dx);
    }
  }
}

void RegionEmitter::push(const Reg reg) {
  const auto index = static_cast<unsigned>(reg);
  if (index >= 8) { code_.byte(high_rex); }
  code_.byte(push_register + (index & 7U));
}

void RegionEmitter::pop(const Reg reg) {
  const auto index = static_cast<unsigned>(reg);
  if (index >= 8) { code_.byte(high_rex); }
  code_.byte(pop_register + (index & 7U));
}

std::vector<std::uint8_t> RegionEmitter::generate() {
  code_.raw(landing);
  const auto saved = std::min(root_registers.size(), region_.roots.size());
  const auto inner_trips = region_.inner_iterations / (inner_vector_ ? lanes_ : 1U);
  for (std::size_t index = 0; index < saved; ++index) {
    push(root_registers[index]);
    code_.load(
        root_registers[index], Reg::di, cursor_offset(index, offsetof(RegionCursor, pointer)));
  }
  code_.load(Reg::r11, Reg::di, offsetof(RegionFrame, constants));
  code_.load(Reg::si, Reg::di, offsetof(RegionFrame, iterations), Width::dword);
  zero(growth_register);
  zero(limit_register);
  x64::Label outer;
  x64::Label inner;
  x64::Label complete;
  code_.bind(outer);
  if (region_.nested && !inner_vector_) {
    x64::Label continuing;
    code_.immediate(group_compare, Reg::si, lanes_, Width::dword);
    code_.jump_if(x64::Branch::not_equal, continuing);
    code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, preceding_growth)},
                       growth_register);
    code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, preceding_limits)},
                       limit_register);
    code_.bind(continuing);
  }
  if (inner_trips > 1) { code_.constant(Reg::cx, inner_trips); }
  code_.bind(inner);
  for (current_ = 0; current_ < region_.graph.nodes.size(); ++current_) {
    for (auto &node : owners_) {
      if (node != no_value && last_[node] < current_) {
        locations_[node] = no_value;
        node = no_value;
      }
    }
    if (needed_[current_]) { emit(region_.graph.nodes[current_]); }
  }
  advance(true);
  if (inner_trips > 1) {
    code_.immediate(group_subtract, Reg::cx, 1, Width::dword);
    code_.jump_if(x64::Branch::not_equal, inner);
  }
  if (inner_vector_) {
    // The final parent's inner DO saves the previous parent's final status.
    x64::Label continuing;
    code_.immediate(group_compare, Reg::si, 2, Width::dword);
    code_.jump_if(x64::Branch::not_equal, continuing);
    code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, preceding_growth)},
                       growth_register);
    code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, preceding_limits)},
                       limit_register);
    outputs(lanes_ - 1, true);
    code_.bind(continuing);
  }
  code_.immediate(group_subtract, Reg::si, inner_vector_ ? 1U : lanes_, Width::dword);
  code_.jump_if(x64::Branch::equal, complete);
  advance(false);
  code_.jump(outer);
  code_.bind(complete);
  inputs_.fill(no_value);
  for (unsigned index = 0; index < region_.graph.registers.size(); ++index) {
    if ((region_.graph.changed_registers & (Word{1} << index)) != 0) {
      publish(region_.graph.registers[index],
              {.offset = offsetof(RegionFrame, results) + (index * sizeof(std::uint64_t)),
               .lane = lanes_ - 1});
    }
  }
  outputs(lanes_ - 1, false);
  if (region_.nested && !inner_vector_) { outputs(lanes_ - 2, true); }
  code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, growth)}, growth_register);
  code_.vector_store({.base = Reg::di, .offset = offsetof(RegionFrame, limits)}, limit_register);
  for (auto index = saved; index != 0; --index) { pop(root_registers[index - 1]); }
  code_.vector_finish();
  code_.byte(return_opcode);
  return code_.bytes();
}
} // namespace

std::vector<std::uint8_t> emit_region(NativeRegion &region, const unsigned lanes) {
  return RegionEmitter(region, lanes).generate();
}

std::vector<std::uint8_t> select_region(NativeRegion &region,
                                        const std::span<const std::uint8_t> fallback) {
  constexpr std::array<std::uint8_t, 10> prologue{
      0xf3, 0x0f, 0x1e, 0xfa, 0x57, 0x56, 0x48, 0x83, 0xec, 0x08};
  constexpr std::array<std::uint8_t, 10> invoke{
      0xff, 0xd0, 0x48, 0x83, 0xc4, 0x08, 0x5e, 0x5f, 0x85, 0xc0};
  x64::Code code;
  x64::Label rejected;
  code.raw(prologue);
  code.constant(x64::Reg::dx, reinterpret_cast<std::uintptr_t>(&region));
  code.constant(x64::Reg::ax, std::bit_cast<std::uintptr_t>(&run_region));
  code.raw(invoke);
  code.jump_if(x64::Branch::equal, rejected);
  code.byte(return_opcode);
  code.bind(rejected);
  code.raw(fallback);
  return code.bytes();
}
} // namespace dsp56300::detail
