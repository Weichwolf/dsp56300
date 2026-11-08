// SPDX-License-Identifier: GPL-3.0-only
#include "value_graph.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "move.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr unsigned accumulator_slot = 4, graph_slots = 6;
constexpr Word product_bit = 0x80, operation_mask = 0xf7;
constexpr std::int64_t saturation_minimum = -(std::int64_t{1} << (double_word_bits - 1));
constexpr std::int64_t saturation_maximum = -saturation_minimum - 1;
constexpr Word clear_code = 0x13, shift_code = 0x22, round_code = 0x11;

GraphBounds aligned_bounds(GraphBounds bounds, const unsigned shift) {
  const auto factor = std::int64_t{1} << shift;
  bounds.low *= factor;
  bounds.high *= factor;
  return bounds;
}

std::int64_t rounded_bound(const std::int64_t value, const unsigned bits, const Word mode) {
  const auto half = std::int64_t{1} << (bits - 1);
  const auto odd = static_cast<std::int64_t>(static_cast<std::uint64_t>(value >> bits) & 1);
  return (value + half - ((mode & rounding_mode) != 0 ? 0 : 1 - odd)) >> bits;
}

GraphBounds
node_bounds(const GraphNode &node, const std::vector<GraphBounds> &bounds, const Word mode) {
  const auto input = [&](const unsigned index) { return bounds.at(node.inputs.at(index).node); };
  switch (node.operation) {
    case GraphOp::load: return {.low = minimum_word, .high = maximum_word};
    case GraphOp::invariant: {
      const auto sign = std::int64_t{1}
                        << (node.parameter < 4 ? word_bits - 1 : accumulator_bits - 1);
      return {.low = -sign, .high = sign - 1};
    }
    case GraphOp::zero:
    case GraphOp::store: return {};
    case GraphOp::add:
    case GraphOp::subtract: {
      const auto scale = std::min(node.inputs[0].scale, node.inputs[1].scale);
      const auto left = aligned_bounds(input(0), node.inputs[0].scale - scale);
      const auto right = aligned_bounds(input(1), node.inputs[1].scale - scale);
      return node.operation == GraphOp::add
                 ? GraphBounds{.low = left.low + right.low, .high = left.high + right.high}
                 : GraphBounds{.low = left.low - right.high, .high = left.high - right.low};
    }
    case GraphOp::multiply: {
      const auto left = input(0);
      const auto right = input(1);
      const std::array products{left.low * right.low,
                                left.low * right.high,
                                left.high * right.low,
                                left.high * right.high};
      const auto [low, high] = std::ranges::minmax(products);
      return {.low = low, .high = high};
    }
    case GraphOp::negate: return {.low = -input(0).high, .high = -input(0).low};
    case GraphOp::shift: return {.low = input(0).low >> 1, .high = input(0).high >> 1};
    case GraphOp::round:
      return {.low = rounded_bound(input(0).low, node.parameter, mode),
              .high = rounded_bound(input(0).high, node.parameter, mode)};
    case GraphOp::saturate: {
      const auto shift = node.parameter != 0 ? 0U : node.inputs[0].scale;
      const auto before = aligned_bounds(input(0), shift);
      const auto low = node.parameter != 0 ? std::int64_t{minimum_word} : saturation_minimum;
      const auto high = node.parameter != 0 ? std::int64_t{maximum_word} : saturation_maximum;
      return {.low = std::clamp(before.low, low, high), .high = std::clamp(before.high, low, high)};
    }
    case GraphOp::word: {
      const auto shift = word_bits - node.inputs[0].scale;
      return {.low = std::clamp(
                  input(0).low >> shift, std::int64_t{minimum_word}, std::int64_t{maximum_word}),
              .high = std::clamp(
                  input(0).high >> shift, std::int64_t{minimum_word}, std::int64_t{maximum_word})};
    }
    default: throw std::logic_error("Unknown graph interval operation");
  }
}

struct AluOperands {
  Word code;
  unsigned destination;
};

struct InputOperand {
  unsigned selector, destination;
};

std::optional<unsigned> slot(const unsigned reg) {
  if (reg >= x0_register && reg <= y1_register) { return reg - x0_register; }
  if (reg == a_register || reg == b_register) { return reg - a_register + accumulator_slot; }
  return std::nullopt;
}

class ValueGraphBuilder {
public:
  explicit ValueGraphBuilder(const GraphSupport support) : support_(support) {}

  [[nodiscard]] std::optional<ValueGraph> build(std::span<const BlockInstruction> operations);

private:
  using Slots = std::array<GraphValue, graph_slots>;
  [[nodiscard]] bool classify(const BlockInstruction &operation);
  [[nodiscard]] bool address_constant(const Instruction &instruction);
  [[nodiscard]] GraphValue read(unsigned index, Slots &state);
  [[nodiscard]] GraphValue node(GraphOp operation,
                                GraphValue result,
                                std::array<GraphValue, 2> inputs = {},
                                unsigned parameter = 0);
  [[nodiscard]] unsigned stream(const NativeMove &move);
  [[nodiscard]] GraphValue word(GraphValue value);
  [[nodiscard]] GraphValue sum(GraphValue left, GraphValue right, bool subtract);
  [[nodiscard]] GraphValue round(GraphValue value);
  [[nodiscard]] GraphValue saturate(GraphValue value, bool rounded);
  [[nodiscard]] bool moves(const BlockInstruction &operation, Slots &old);
  [[nodiscard]] bool arithmetic(Word code, Slots &old);
  [[nodiscard]] bool product(AluOperands operands, Slots &old);
  [[nodiscard]] GraphValue operand(InputOperand input, Slots &old);
  ValueGraph graph_;
  std::array<bool, graph_slots> writes_{};
  std::array<bool, agu_registers> accessed_{};
  std::vector<GraphBounds> bounds_;
  GraphSupport support_;
  Word mode_ = 0;
  bool valid_ = true;
};

bool ValueGraphBuilder::classify(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  if (instruction.operation != Operation::parallel && instruction.operation != Operation::nop) {
    return false;
  }
  if (plan_address_immediate(instruction).valid()) { return address_constant(instruction); }
  if (instruction.words != 1 || instruction.cycles != 1) { return false; }
  if (instruction.operation == Operation::nop) { return true; }
  const auto plan = plan_moves(instruction);
  const auto reg = plan_register_move(instruction);
  if (plan.long_pair) { return false; }
  if (reg.valid()) {
    const auto destination = slot(reg.destination);
    if (!destination || !slot(reg.source)) { return false; }
    writes_.at(*destination) = true;
  } else if ((instruction.opcode & upper_move_mask) != no_move_group && plan.count == 0) {
    return false;
  }
  std::array<bool, agu_registers> updated{};
  for (const auto &move : std::span(plan.transfers).first(plan.count)) {
    const auto target = slot(move.reg);
    if (!target || move.immediate() ||
        (move.address.mode != 1 && move.address.mode != 3 && move.address.mode != 4) ||
        updated.at(move.address.index)) {
      return false;
    }
    updated.at(move.address.index) = true;
    accessed_.at(move.address.index) = true;
    if (move.load) { writes_.at(*target) = true; }
  }
  if ((instruction.opcode & byte_mask) != 0) {
    writes_.at(accumulator_slot + ((instruction.opcode >> 3) & 1U)) = true;
  }
  return true;
}

bool ValueGraphBuilder::address_constant(const Instruction &instruction) {
  const auto immediate = plan_address_immediate(instruction);
  if (!support_.address_constants || immediate.destination < r_register_base ||
      immediate.destination >= n_register_base || instruction.words > 2 ||
      instruction.cycles != instruction.words) {
    return false;
  }
  const auto index = immediate.destination - r_register_base;
  if (accessed_[index]) { return false; }
  graph_.address_constants[index] = immediate.value;
  return true;
}

GraphValue ValueGraphBuilder::node(const GraphOp operation,
                                   GraphValue result,
                                   const std::array<GraphValue, 2> inputs,
                                   const unsigned parameter) {
  if (!valid_ || result.width > accumulator_bits ||
      result.scale > accumulator_bits - result.width) {
    valid_ = false;
    return {};
  }
  result.node = graph_.nodes.size();
  graph_.nodes.push_back({.operation = operation, .inputs = inputs, .parameter = parameter});
  bounds_.push_back(node_bounds(graph_.nodes.back(), bounds_, mode_));
  return result;
}

GraphValue ValueGraphBuilder::read(const unsigned index, Slots &state) {
  if (state.at(index).valid()) { return state.at(index); }
  if (writes_.at(index)) {
    valid_ = false;
    return {};
  }
  const auto value = node(GraphOp::invariant,
                          {.width = index < accumulator_slot ? word_bits : accumulator_bits},
                          {},
                          index);
  graph_.invariant_registers |= Word{1} << index;
  graph_.registers.at(index) = value;
  state.at(index) = value;
  return value;
}

unsigned ValueGraphBuilder::stream(const NativeMove &move) {
  auto &offset = graph_.addresses.at(move.address.index);
  const auto found = std::ranges::find_if(graph_.streams, [&](const auto &candidate) {
    return candidate.space == move.space && candidate.address == move.address.index &&
           candidate.units == offset.units && candidate.offsets == offset.offsets;
  });
  const auto index = static_cast<unsigned>(found - graph_.streams.begin());
  if (found == graph_.streams.end()) {
    graph_.streams.push_back({.space = move.space,
                              .address = move.address.index,
                              .units = offset.units,
                              .offsets = offset.offsets});
  }
  if (!move.load) { graph_.streams.at(index).write = true; }
  offset.units += static_cast<unsigned>(move.address.mode == 3);
  offset.offsets += static_cast<unsigned>(move.address.mode == 1);
  return index;
}

GraphValue ValueGraphBuilder::word(const GraphValue value) {
  if (value.scale > word_bits) {
    valid_ = false;
    return {};
  }
  return node(GraphOp::word, {.canonical = false}, {value, {}});
}

bool ValueGraphBuilder::moves(const BlockInstruction &operation, Slots &old) {
  if (plan_address_immediate(operation.instruction).valid()) { return true; }
  const auto reg = plan_register_move(operation.instruction);
  if (reg.valid()) {
    const auto source_slot = slot(reg.source);
    const auto destination_slot = slot(reg.destination);
    if (!source_slot || !destination_slot) { return false; }
    const auto source = *source_slot;
    const auto destination = *destination_slot;
    auto value = read(source, old);
    if (source >= accumulator_slot) { value = word(value); }
    if (destination >= accumulator_slot) { value.scale = word_bits; }
    graph_.registers.at(destination) = value;
    return valid_;
  }
  const auto plan = plan_moves(operation.instruction);
  std::array<GraphValue, 2> stores{};
  std::array<unsigned, 2> streams{};
  for (unsigned index = 0; index < plan.count; ++index) {
    const auto &move = plan.transfers.at(index);
    const auto target_slot = slot(move.reg);
    if (!target_slot) { return false; }
    const auto target = *target_slot;
    streams.at(index) = stream(move);
    if (move.load) {
      auto value = node(GraphOp::load, {}, {}, streams.at(index));
      if (target >= accumulator_slot) { value.scale = word_bits; }
      graph_.registers.at(target) = value;
    } else {
      auto value = read(target, old);
      if (target >= accumulator_slot) { value = word(value); }
      stores.at(index) = value;
    }
  }
  // Both parallel loads precede either store. ALU operands still use old slots.
  for (unsigned index = 0; index < plan.count; ++index) {
    if (stores.at(index).valid()) {
      static_cast<void>(
          node(GraphOp::store, {.width = 0}, {stores.at(index), {}}, streams.at(index)));
    }
  }
  return valid_;
}

GraphValue
ValueGraphBuilder::sum(const GraphValue left, const GraphValue right, const bool subtract) {
  if (right.width == 0) { return left; }
  if (left.width == 0 && !subtract) { return right; }
  const auto scale = std::min(left.scale, right.scale);
  const auto width =
      std::max(left.width + left.scale - scale, right.width + right.scale - scale) + 1;
  return node(
      subtract ? GraphOp::subtract : GraphOp::add, {.scale = scale, .width = width}, {left, right});
}

GraphValue ValueGraphBuilder::round(const GraphValue value) {
  if (value.scale >= word_bits) { return value; }
  const auto shift = word_bits - value.scale;
  const auto width = value.width >= shift ? value.width - shift + 1 : 1;
  if (width > 32) {
    valid_ = false;
    return {};
  }
  return node(
      GraphOp::round, {.scale = word_bits, .width = width, .canonical = false}, {value, {}}, shift);
}

GraphValue ValueGraphBuilder::saturate(const GraphValue value, const bool rounded) {
  graph_.overflow_input = {};
  if ((mode_ & arithmetic_saturation) == 0 || !value.valid()) { return value; }
  const auto raw = aligned_bounds(bounds_[value.node], value.scale);
  const auto high = rounded ? std::int64_t{maximum_word} << word_bits : saturation_maximum;
  if (raw.low >= saturation_minimum && raw.high <= high) { return value; }
  constexpr auto period = std::int64_t{1} << (double_word_bits + 1);
  // In this interval SM's three-bit action is exactly a signed 48-bit clamp.
  if (raw.low < -period || raw.high >= period) {
    valid_ = false;
    return {};
  }
  graph_.overflow_input = value;
  return node(GraphOp::saturate,
              {.scale = rounded ? word_bits : 0U, .width = rounded ? word_bits : double_word_bits},
              {value, {}},
              static_cast<unsigned>(rounded));
}

GraphValue ValueGraphBuilder::operand(const InputOperand input, Slots &old) {
  const auto selector = input.selector;
  const auto destination = input.destination;
  if (selector < 2) { return read((2 * accumulator_slot) + 1 - destination, old); }
  if (selector < 4) {
    valid_ = false;
    return {};
  }
  auto value = read(((selector & 1U) * 2) + static_cast<unsigned>(selector >= 6), old);
  value.scale = word_bits;
  return value;
}

bool ValueGraphBuilder::product(const AluOperands operands, Slots &old) {
  const auto code = operands.code;
  const auto destination = operands.destination;
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const auto &pair = pairs.at((code >> 4) & 7);
  auto value = node(GraphOp::multiply,
                    {.scale = 1, .width = double_word_bits},
                    {read(pair[0], old), read(pair[1], old)});
  const bool negate = (code & 4U) != 0;
  if ((code & 2U) != 0) {
    value = sum(read(destination, old), value, negate);
  } else if (negate) {
    value = node(GraphOp::negate, {.scale = value.scale, .width = value.width + 1}, {value, {}});
  }
  const bool rounded = (code & 1U) != 0;
  if (rounded) { value = round(value); }
  value = saturate(value, rounded);
  graph_.registers.at(destination) = graph_.flags = value;
  return valid_;
}

bool ValueGraphBuilder::arithmetic(const Word code, Slots &old) {
  if ((code & byte_mask) == 0) { return true; }
  const auto destination = accumulator_slot + ((code >> 3) & 1U);
  if ((code & product_bit) != 0) {
    return product({.code = code, .destination = destination}, old);
  }
  const auto kind = code & operation_mask;
  const auto selector = (code >> 4) & 7U;
  GraphValue value;
  if (kind == clear_code) {
    value = node(GraphOp::zero, {.width = 0});
  } else if (kind == shift_code) {
    const auto input = read(destination, old);
    graph_.carry = input.scale == 0 ? GraphCarry::bit : GraphCarry::zero;
    graph_.carry_inputs = {input, {}};
    if (input.scale != 0) {
      value = input;
      --value.scale;
    } else {
      value = node(GraphOp::shift,
                   {.width = input.width > 1 ? input.width - 1 : 1, .canonical = input.canonical},
                   {input, {}});
    }
  } else if (kind == round_code) {
    value = round(read(destination, old));
  } else if ((code & 7U) == 1 && (selector == 0 || selector >= 4)) {
    graph_.registers.at(destination) =
        operand({.selector = selector, .destination = destination}, old);
    return valid_;
  } else if (selector >= 1 && ((code & 7U) == 0 || (code & 7U) == 4)) {
    const bool subtract = (code & 7U) == 4;
    const auto left = read(destination, old);
    const auto right = operand({.selector = selector, .destination = destination}, old);
    value = sum(left, right, subtract);
    graph_.carry = subtract ? GraphCarry::subtract : GraphCarry::add;
    graph_.carry_inputs = {left, right};
  } else {
    return false;
  }
  value = saturate(value, kind == round_code);
  graph_.registers.at(destination) = graph_.flags = value;
  return valid_;
}

std::optional<ValueGraph>
ValueGraphBuilder::build(const std::span<const BlockInstruction> operations) {
  if (operations.empty() || !at_loop_end(operations.back())) { return std::nullopt; }
  mode_ = operations.front().mode;
  if (!std::ranges::all_of(operations, [&](const auto &op) {
        return op.mode == mode_ && (op.mode & ~support_.numeric_modes) == 0 && classify(op);
      })) {
    return std::nullopt;
  }
  std::uint64_t cycles = 0;
  for (const auto &operation : operations) { cycles += operation.instruction.cycles; }
  if (!std::in_range<Word>(operations.size()) || cycles < operations.size() ||
      !std::in_range<Word>(cycles - operations.size())) {
    return std::nullopt;
  }
  graph_.extra_cycles = static_cast<Word>(cycles - operations.size());
  for (const auto &operation : operations) {
    auto old = graph_.registers;
    if (!moves(operation, old) || !arithmetic(operation.instruction.opcode & byte_mask, old)) {
      return std::nullopt;
    }
  }
  if (!std::ranges::any_of(graph_.streams, &GraphStream::write)) { return std::nullopt; }
  graph_.instructions = static_cast<Word>(operations.size());
  for (unsigned index = 0; index < writes_.size(); ++index) {
    if (writes_[index]) { graph_.changed_registers |= Word{1} << index; }
  }
  return std::move(graph_);
}
} // namespace

std::optional<ValueGraph> value_graph(const std::span<const BlockInstruction> operations,
                                      const GraphSupport support) {
  return ValueGraphBuilder{support}.build(operations);
}

std::vector<GraphBounds> graph_bounds(const ValueGraph &graph, const Word mode) {
  std::vector<GraphBounds> bounds;
  bounds.reserve(graph.nodes.size());
  for (const auto &node : graph.nodes) { bounds.push_back(node_bounds(node, bounds, mode)); }
  return bounds;
}

} // namespace dsp56300::detail
