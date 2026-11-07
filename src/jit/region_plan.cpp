// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "move.hpp"
#include "region.hpp"
#include "translation.hpp"
#include "value_graph.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>

namespace dsp56300::detail {
namespace {
constexpr Word relative_end_bit = 16;

unsigned growth_shift(const ValueGraph &graph) {
  std::optional<unsigned> common;
  for (const auto &node : graph.nodes) {
    if (node.operation != GraphOp::word) { continue; }
    const auto position = (double_word_bits - 3) - node.inputs[0].scale;
    if (node.inputs[0].width <= position + 1) { continue; }
    if (common && *common != position) { return 0; }
    common = position;
  }
  return common.value_or(0);
}

bool nested(const std::span<const Word> program,
            const Registers &state,
            const JitRuntime &runtime,
            const std::span<const BlockInstruction> operations,
            NativeRegion &region) {
  const auto &setup = operations.back();
  const auto &instruction = setup.instruction;
  if (instruction.operation != Operation::loop_immediate || setup.loop_address == inactive_loop) {
    return false;
  }
  const auto count = ((instruction.opcode >> 8) & byte_mask) | ((instruction.opcode & 15U) << 8);
  const auto last = ((instruction.opcode & relative_end_bit) != 0 ? setup.pc + instruction.extension
                                                                  : instruction.extension) &
                    word_mask;
  if (count == 0 || last >= setup.loop_address) { return false; }
  auto body_state = state;
  body_state.pc = (setup.pc + instruction.words) & word_mask;
  body_state.la = last;
  body_state.sr = setup.mode | loop_flag;
  const auto body = parse_block(program, body_state, runtime);
  auto graph = value_graph(body);
  if (!graph) { return false; }
  region.inner_entry = body_state.pc;
  body_state.pc = (last + 1) & word_mask;
  body_state.la = setup.loop_address;
  const auto post = parse_block(program, body_state, runtime);
  if (post.empty() || !at_loop_end(post.back())) { return false; }
  for (const auto &operation : operations.first(operations.size() - 1)) {
    const auto value = plan_address_immediate(operation.instruction);
    if (!value.valid() || value.destination < r_register_base ||
        value.destination >= n_register_base) {
      return false;
    }
    region.prefix[value.destination - r_register_base] = value.value;
  }
  region.inner_addresses = graph->addresses;
  for (auto &delta : graph->addresses) {
    const auto units = std::uint64_t{delta.units} * count;
    const auto offsets = std::uint64_t{delta.offsets} * count;
    if (units > word_mask || offsets > word_mask) { return false; }
    delta = {.units = static_cast<Word>(units), .offsets = static_cast<Word>(offsets)};
  }
  for (const auto &operation : post) {
    const auto opcode = operation.instruction.opcode;
    if (!pure_address_update(operation.instruction) || ((opcode >> 11) & 7U) != 1) { return false; }
    ++graph->addresses[(opcode >> 8) & 7U].offsets;
  }
  const auto overhead = static_cast<Word>(operations.size() + post.size());
  Word cycles = 0;
  for (const auto &operation : operations) { cycles += operation.instruction.cycles; }
  for (const auto &operation : post) { cycles += operation.instruction.cycles; }
  const auto instructions = (std::uint64_t{graph->instructions} * count) + overhead;
  if (!std::in_range<Word>(instructions)) { return false; }
  graph->instructions = static_cast<Word>(instructions);
  region.graph = std::move(*graph);
  region.extra_cycles = cycles - overhead;
  region.inner_iterations = count;
  region.last = setup.loop_address;
  region.nested = true;
  return true;
}

bool roots(NativeRegion &region, const Registers &state) {
  for (const auto &stream : region.graph.streams) {
    const auto &delta = region.graph.addresses[stream.address];
    if ((state.m[stream.address] & modifier_mask) == 0 && delta.offsets != 0) {
      if (region.nested || delta.units != 0 || stream.write) { return false; }
      region.reverse_addresses |= Word{1} << stream.address;
    }
    const auto index = stream.space == Space::x ? 0U : 1U;
    if (region.runtime.memory[index].data == nullptr || !region.runtime.windows[index].empty() ||
        (stream.write && region.runtime.memory[index].aliases_program)) {
      return false;
    }
    const auto found = std::ranges::find_if(region.roots, [&](const auto &root) {
      return root.address == stream.address && root.space == stream.space;
    });
    const auto id = static_cast<unsigned>(found - region.roots.begin());
    if (found == region.roots.end()) {
      region.roots.push_back({.space = stream.space, .address = stream.address});
    }
    region.stream_roots.push_back(id);
  }
  region.offsets.resize(region.graph.streams.size());
  region.footprints.resize(region.graph.streams.size());
  return true;
}
} // namespace

std::unique_ptr<NativeRegion> plan_region(const std::span<const Word> program,
                                          const Registers &state,
                                          const JitRuntime &runtime,
                                          const std::span<const BlockInstruction> operations) {
  if (operations.empty()) { return nullptr; }
  auto region = std::make_unique<NativeRegion>();
  region->runtime = runtime;
  region->entry = operations.front().pc;
  region->last = operations.back().loop_address;
  region->mode = operations.front().mode;
  if (!nested(program, state, runtime, operations, *region)) {
    auto graph = value_graph(operations);
    if (!graph) { return nullptr; }
    region->graph = std::move(*graph);
  }
  region->growth_shift = growth_shift(region->graph);
  if (!roots(*region, state)) { return nullptr; }
  return region;
}
} // namespace dsp56300::detail
