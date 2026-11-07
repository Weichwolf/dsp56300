// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
inline constexpr std::size_t no_value = static_cast<std::size_t>(-1);

// Integer value times 2^scale is the architectural, unpadded accumulator.
struct GraphValue {
  std::size_t node = no_value;
  unsigned scale = 0, width = word_bits;
  bool canonical = true;

  [[nodiscard]] bool valid() const noexcept { return node != no_value; }
};

enum class GraphOp : std::uint8_t {
  load,
  invariant,
  zero,
  add,
  subtract,
  multiply,
  negate,
  shift,
  round,
  word,
  store
};

struct GraphNode {
  GraphOp operation;
  std::array<GraphValue, 2> inputs{};
  unsigned parameter = 0;
};

struct GraphStream {
  Space space = Space::x;
  unsigned address = 0;
  Word units = 0, offsets = 0;
  bool write = false;
};

struct GraphAddress {
  Word units = 0, offsets = 0;
};

enum class GraphCarry : std::uint8_t { unchanged, zero, bit, add, subtract };

struct ValueGraph {
  std::vector<GraphNode> nodes;
  std::vector<GraphStream> streams;
  std::array<GraphValue, 6> registers{};
  std::array<GraphAddress, agu_registers> addresses{};
  GraphValue flags;
  GraphCarry carry = GraphCarry::unchanged;
  std::array<GraphValue, 2> carry_inputs{};
  Word instructions = 0, changed_registers = 0, invariant_registers = 0;
};

struct GraphBounds {
  std::int64_t low = 0, high = 0;
};

// Signed integer coordinates before each edge's binary-point scaling.
[[nodiscard]] std::vector<GraphBounds> graph_bounds(const ValueGraph &graph, Word mode);

// Compile-time graph construction. Unproved arithmetic or iteration dependencies
// leave the existing scalar native lowering responsible for the whole region.
[[nodiscard]] std::optional<ValueGraph> value_graph(std::span<const BlockInstruction> operations);
} // namespace dsp56300::detail
