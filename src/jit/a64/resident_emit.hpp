// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace dsp56300::detail::a64 {
inline constexpr unsigned region_values = 12;
inline constexpr Vec first_scratch{24}, second_scratch{25}, growth_register{26}, limit_register{28};

struct StreamSlice {
  unsigned stream, part;
};

struct ValueOutput {
  std::size_t offset;
  unsigned lane;
};

inline constexpr std::array root_registers{
    Reg::x19, Reg::x20, Reg::x21, Reg::x22, Reg::x23, Reg::x24, Reg::x25, Reg::x26};

constexpr std::size_t cursor_offset(const std::size_t root, const std::size_t member) {
  return offsetof(RegionFrame, cursors) + (root * sizeof(RegionCursor)) + member;
}

class RegionEmitter {
public:
  RegionEmitter(ResidentRegion &region, unsigned lanes);
  [[nodiscard]] std::vector<std::uint8_t> generate();

private:
  void pin(GraphValue value);
  void liveness();
  void operands(const GraphNode &node);
  [[nodiscard]] unsigned allocate();
  [[nodiscard]] Vec read(GraphValue value, unsigned part = 0);
  [[nodiscard]] Vec destination();
  void spill(unsigned reg);
  void constant(Vec target, std::uint64_t value);
  void canonical(Vec target, GraphValue value, unsigned part);
  [[nodiscard]] Vec aligned(GraphValue value, unsigned scale, Vec scratch, unsigned part);
  void pointer(StreamSlice slice);
  [[nodiscard]] std::optional<std::uint64_t> fixed_stride(unsigned stream) const;
  void stride(unsigned stream);
  void reverse_load(Vec target, StreamSlice slice);
  void load(Vec target, StreamSlice slice);
  void store(GraphValue value, StreamSlice slice);
  void store_packed(Vec source);
  void emit(const GraphNode &node);
  void binary(const GraphNode &node, Vec target, unsigned part);
  void round(const GraphNode &node, Vec target, unsigned part);
  void word(const GraphNode &node, Vec target, unsigned part);
  void publish(GraphValue value, ValueOutput output);
  void outputs(unsigned lane, bool preceding);
  void advance(bool inner);
  void advance_reverse();
  void emit_body();
  void save_events(bool preceding);
  void step(std::size_t root, bool inner);
  void save();
  void restore();
  ResidentRegion &region_;
  Code code_;
  Neon neon_{code_};
  unsigned lanes_, parts_;
  bool inner_vector_;
  std::size_t current_ = 0;
  std::vector<GraphBounds> bounds_;
  std::vector<bool> needed_;
  std::vector<std::size_t> last_, locations_, spill_slots_;
  std::array<std::size_t, region_values> owners_{};
  std::array<std::size_t, 2> inputs_{};
};
}
