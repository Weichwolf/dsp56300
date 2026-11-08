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
inline constexpr Vec first_scratch{24}, second_scratch{25}, growth_register{26}, limit_register{28},
    one_register{31};

struct StreamSlice {
  unsigned stream, part;
};

struct ValueOutput {
  std::size_t offset;
  unsigned lane;
};

inline constexpr std::array stride_registers{
    Reg::x4, Reg::x5, Reg::x6, Reg::x7, Reg::x8, Reg::x13, Reg::x14, Reg::x15};

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
  enum class StickyEvent : std::uint8_t { growth, limit };
  void pin(GraphValue value);
  void liveness();
  void fuse_products();
  void operands(const GraphNode &node);
  [[nodiscard]] unsigned allocate();
  [[nodiscard]] Vec read(GraphValue value, unsigned part = 0);
  [[nodiscard]] Vec destination();
  void spill(unsigned reg);
  void constant(Vec target, std::uint64_t value);
  [[nodiscard]] bool compact(GraphValue value) const;
  [[nodiscard]] Vec aligned_words(GraphValue value, unsigned scale, Vec scratch);
  void emit_words(const GraphNode &node, Vec target);
  void word_words(const GraphNode &node, Vec target);
  void round_words(const GraphNode &node, Vec target);
  void merge_words(StickyEvent event, Vec source);
  void merge_wide(StickyEvent event, Vec source, unsigned part);

  [[nodiscard]] bool packed_events() const noexcept { return region_.growth_shift < 32; }

  void constant_words(Vec target, std::uint32_t value);
  void pack(const GraphNode &node, Vec target);
  void load_words(Vec target, unsigned stream);
  void store_words(GraphValue value, unsigned stream);
  void multiply(const GraphNode &node, Vec target, unsigned part);
  void canonical(Vec target, GraphValue value, unsigned part);
  [[nodiscard]] Vec aligned(GraphValue value, unsigned scale, Vec scratch, unsigned part);
  void pointer(StreamSlice slice);
  [[nodiscard]] std::optional<std::uint64_t> fixed_stride(unsigned stream) const;
  [[nodiscard]] std::optional<std::uint64_t> fixed_offset(unsigned stream) const;
  [[nodiscard]] Reg stride(unsigned stream);
  void cache_strides();
  void reverse_load(Vec target, StreamSlice slice, bool packed = false);
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
  Word cached_strides_ = 0;
  Reg stride_ = Reg::x11;
  std::size_t current_ = 0;
  std::vector<GraphBounds> bounds_;
  std::vector<bool> needed_, compact_;
  std::vector<std::size_t> last_, locations_, spill_slots_, fused_;
  std::array<std::size_t, region_values> owners_{};
  std::array<std::size_t, 3> inputs_{};
};
}
