// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "region.hpp"
#include "value_graph.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr Word loop_status = loop_flag | forever_flag;
constexpr unsigned sign_bit = accumulator_bits - 1, extension_shift = double_word_bits - 1;
constexpr std::uint64_t extension_mask = 511;
constexpr Word stack_pointer_mask = 63;
constexpr Word arithmetic_status = overflow | zero | negative | unnormalized | extension;

bool overlap(const std::uintptr_t first,
             const std::uint64_t first_size,
             const std::uintptr_t second,
             const std::uint64_t second_size) {
  return first <= second ? second - first < first_size : first - second < second_size;
}

bool scaled_offset(const GraphAddress amount, const Word n, std::uint64_t &result) {
  // Two uint32 inputs and one uint32 addend fit in uint64, including endpoints.
  result = amount.units + (std::uint64_t{amount.offsets} * n);
  return result <= word_mask;
}

bool header(const Cpu &cpu, const NativeContext &context, const NativeRegion &region) {
  const auto &reg = cpu.reg;
  return reg.pc == region.entry && reg.la == region.last && (reg.sr & loop_flag) != 0 &&
         (reg.sr & forever_flag) == 0 && (reg.sr & numeric_mode_mask) == region.mode &&
         (!context.runtime.cacheable_program || (reg.sr & cache_enable) == 0) &&
         cpu.repeat_remaining == 0 && cpu.processing == ExecutionState::running &&
         cpu.debug_flags == 0 && reg.lc != 0 && reg.lc <= word_mask &&
         (reg.stack[reg.sp & (stack_entries - 1)] >> word_bits) == region.entry &&
         (context.exit_requested == nullptr || !*context.exit_requested);
}

bool memory(const Cpu &cpu, const NativeContext &context, const NativeRegion &region) {
  std::array<bool, 2> seen{};
  for (const auto &stream : region.graph.streams) {
    const auto index = stream.space == Space::x ? 0U : 1U;
    const auto &ram = context.runtime.memory[index];
    if (stream.write && ram.aliases_program) { return false; }
    if (seen[index]) { continue; }
    seen[index] = true;
    const auto bytes = std::min(ram.words, std::size_t{io_begin}) * sizeof(Word);
    const auto base = reinterpret_cast<std::uintptr_t>(ram.data);
    if (ram.data == nullptr || !context.runtime.windows[index].empty() ||
        overlap(base, bytes, reinterpret_cast<std::uintptr_t>(&cpu), sizeof(cpu)) ||
        overlap(base, bytes, reinterpret_cast<std::uintptr_t>(&context), sizeof(context))) {
      return false;
    }
  }
  return true;
}

bool address(const Registers &reg, NativeRegion &region, const unsigned index) {
  const auto iterations = region.frame.iterations;
  const auto initial = region.prefix[index].value_or(reg.r[index]);
  const auto &delta = region.graph.addresses[index];
  const bool uses_n = delta.offsets != 0;
  const auto n = uses_n ? reg.n[index] : 0;
  if (initial > word_mask || n >= sign24_bit) { return false; }
  std::uint64_t step = 0;
  if (!scaled_offset(delta, n, step)) { return false; }
  // A reset root still advances through every child iteration and the postlude
  // of the final parent; only the next parent's prefix resets it again.
  const auto distance = region.prefix[index] ? step : iterations * step;
  if (distance > word_mask - initial) { return false; }
  const auto final = initial + distance;
  const auto modifier = reg.m[index] & modifier_mask;
  if (step != 0 && modifier != modifier_mask) {
    if (modifier == 0 || modifier >= multiple_wrap_bit) { return false; }
    const auto begin = initial & ~(std::bit_ceil(modifier + 1) - 1);
    const auto limit = begin + modifier + 1;
    if (initial >= limit || final >= limit) { return false; }
  }
  region.frame.addresses[index] = static_cast<Word>(final);
  return true;
}

bool cursors(const Registers &reg, const NativeContext &context, NativeRegion &region) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    if (!address(reg, region, index)) { return false; }
  }
  for (std::size_t index = 0; index < region.roots.size(); ++index) {
    const auto &root = region.roots[index];
    auto &cursor = region.frame.cursors[index];
    const auto first = region.prefix[root.address].value_or(reg.r[root.address]);
    const auto &delta = region.graph.addresses[root.address];
    std::uint64_t stride = 0;
    if (!scaled_offset(delta, reg.n[root.address], stride)) { return false; }
    if (region.prefix[root.address]) { stride = 0; }
    cursor.stride = stride;
    if (!scaled_offset(
            region.inner_addresses[root.address], reg.n[root.address], cursor.inner_stride)) {
      return false;
    }
    cursor.inner_step_bytes = cursor.inner_stride * sizeof(Word);
    cursor.step_bytes =
        static_cast<std::int64_t>(stride * region.lanes * sizeof(Word)) -
        static_cast<std::int64_t>(cursor.inner_step_bytes * region.inner_iterations);
    for (unsigned lane = 0; lane < cursor.indices.size(); ++lane) {
      const auto displacement = (lane % region.lanes) * stride;
      if (displacement > word_mask) { return false; }
      cursor.indices[lane] = static_cast<std::uint32_t>(displacement);
    }
    const auto &ram = context.runtime.memory[root.space == Space::x ? 0 : 1];
    if (first >= std::min(ram.words, std::size_t{io_begin})) { return false; }
    cursor.pointer = ram.data + first;
  }
  return true;
}

bool footprint(const Registers &reg,
               const NativeContext &context,
               NativeRegion &region,
               const std::size_t index,
               RegionFootprint &result) {
  const auto &stream = region.graph.streams[index];
  const auto &cursor = region.frame.cursors[region.stream_roots[index]];
  const auto &ram = context.runtime.memory[stream.space == Space::x ? 0 : 1];
  const auto first = region.prefix[stream.address].value_or(reg.r[stream.address]);
  std::uint64_t displacement = 0;
  if (!scaled_offset({.units = stream.units, .offsets = stream.offsets},
                     reg.n[stream.address],
                     displacement)) {
    return false;
  }
  const auto span = (region.inner_iterations - 1) * cursor.inner_stride;
  const auto end = first + displacement + span + ((region.frame.iterations - 1) * cursor.stride);
  if (end >= std::min(ram.words, std::size_t{io_begin}) || (stream.write && cursor.stride == 0)) {
    return false;
  }
  region.offsets[index] = displacement * sizeof(Word);
  result = {.first = reinterpret_cast<std::uintptr_t>(ram.data + first + displacement),
            .bytes = (((region.frame.iterations - 1) * cursor.stride) + span + 1) * sizeof(Word),
            .stride = cursor.stride * sizeof(Word),
            .inner_span = span * sizeof(Word)};
  return true;
}

bool independent(const RegionFootprint &left, const RegionFootprint &right) {
  if (!overlap(left.first, left.bytes, right.first, right.bytes)) { return true; }
  if (left.stride != right.stride) { return false; }
  if (left.first == right.first) {
    return left.inner_span < left.stride && right.inner_span < right.stride;
  }
  const auto distance =
      left.first > right.first ? left.first - right.first : right.first - left.first;
  if (left.inner_span == 0 && right.inner_span == 0) {
    return left.stride != 0 && distance % left.stride != 0;
  }
  const auto later_span = left.first > right.first ? left.inner_span : right.inner_span;
  return distance < left.stride && left.inner_span < left.stride &&
         right.inner_span < right.stride && later_span < left.stride - distance;
}

bool streams(const Registers &reg, const NativeContext &context, NativeRegion &region) {
  // Scratch storage is sized during translation, never allocated by execution.
  for (std::size_t index = 0; index < region.graph.streams.size(); ++index) {
    auto &current = region.footprints[index];
    if (!footprint(reg, context, region, index, current)) { return false; }
    for (std::size_t other = 0; other < index; ++other) {
      if (!region.graph.streams[index].write && !region.graph.streams[other].write) { continue; }
      if (!independent(current, region.footprints[other])) { return false; }
    }
  }
  return true;
}

void invariants(const Registers &reg, NativeRegion &region) {
  for (unsigned index = 0; index < region.frame.invariants.size(); ++index) {
    if ((region.graph.invariant_registers & (Word{1} << index)) == 0) { continue; }
    std::int64_t value = 0;
    if (index < 4) {
      const auto packed = index < 2 ? reg.x : reg.y;
      value = signed_word(static_cast<Word>(packed >> ((index & 1U) * word_bits)));
    } else {
      value = std::bit_cast<std::int64_t>(index == 4 ? reg.a : reg.b) >> padding_bits;
    }
    region.frame.invariants[index] = static_cast<std::uint64_t>(value);
  }
}

std::uint64_t guest(const std::uint64_t raw, const GraphValue value) {
  const auto canonical = value.canonical ? raw
                                         : static_cast<std::uint64_t>(static_cast<std::int64_t>(
                                               static_cast<std::int32_t>(static_cast<Word>(raw))));
  return (canonical << value.scale) & accumulator_mask;
}

Word flags(const std::array<std::uint64_t, 3> &values, const ValueGraph &graph) {
  Word result = 0;
  if (graph.flags.valid()) {
    const auto value = guest(values[0], graph.flags);
    if (value == 0) { result |= zero; }
    if ((value & (std::uint64_t{1} << sign_bit)) != 0) { result |= negative; }
    if ((value >> extension_shift) != 0 && (value >> extension_shift) != extension_mask) {
      result |= extension;
    }
    if (((value >> extension_shift) & 1U) == ((value >> (extension_shift - 1)) & 1U)) {
      result |= unnormalized;
    }
  }
  if (graph.carry == GraphCarry::unchanged || graph.carry == GraphCarry::zero) { return result; }
  const auto left = guest(values[1], graph.carry_inputs[0]);
  if (graph.carry == GraphCarry::bit) { return result | static_cast<Word>(left & 1); }
  const auto right = guest(values[2], graph.carry_inputs[1]);
  auto carry_value = static_cast<Word>((left + right) >> accumulator_bits);
  if (graph.carry == GraphCarry::subtract) { carry_value = static_cast<Word>(left < right); }
  return result | carry_value;
}

Word sticky(const RegionVector &growth_events,
            const RegionVector &limit_events,
            const unsigned lanes) {
  std::uint64_t changed = 0;
  std::uint64_t limited = 0;
  for (unsigned index = 0; index < lanes; ++index) {
    changed |= growth_events.words[index];
    limited |= limit_events.words[index];
  }
  return ((changed & 1) != 0 ? growth : 0U) |
         ((limited & std::numeric_limits<Word>::max()) != 0 ? limit : 0U);
}

void stack(Registers &reg, const NativeRegion &region, const Word changed) {
  const auto &frame = region.frame;
  if (region.nested) {
    const auto preceding = sticky(frame.preceding_growth, frame.preceding_limits, 4) |
                           sticky(frame.growth, frame.limits, region.lanes - 1);
    const auto before =
        (reg.sr & ~changed) | preceding | flags(frame.preceding_flags, region.graph);
    reg.stack[(reg.sp + 1) & (stack_entries - 1)] = (std::uint64_t{region.last} << word_bits) | 1;
    reg.stack[(reg.sp + 2) & (stack_entries - 1)] =
        (std::uint64_t{region.inner_entry} << word_bits) | before;
  }
  if (reg.lc != 0) { return; }
  const auto saved_status = static_cast<Word>(reg.stack[reg.sp & (stack_entries - 1)]);
  const auto saved = reg.stack[(reg.sp + stack_pointer_mask) & (stack_entries - 1)];
  reg.pc = (reg.la + 1) & word_mask;
  reg.sr = (reg.sr & ~loop_status) | (saved_status & loop_status);
  reg.lc = static_cast<Word>(saved) & word_mask;
  reg.la = static_cast<Word>(saved >> word_bits);
  reg.sp = (reg.sp + stack_pointer_mask - 1) & stack_pointer_mask;
  reg.sc -= 2;
}

void commit(Cpu &cpu, NativeContext &context, const NativeRegion &region) {
  auto &reg = cpu.reg;
  const auto &frame = region.frame;
  Word changed = region.graph.flags.valid() ? arithmetic_status : 0;
  if (region.graph.carry != GraphCarry::unchanged) { changed |= carry; }
  // Inner SSL is the state before the final iteration, before publishing its flags.
  reg.lc -= frame.iterations;
  stack(reg, region, changed);
  const auto loop_bits = reg.sr & loop_status;
  reg.sr = (reg.sr & ~changed) | sticky(frame.growth, frame.limits, 4) |
           flags(frame.flags, region.graph);
  reg.sr = (reg.sr & ~loop_status) | loop_bits;
  for (unsigned index = 0; index < region.graph.registers.size(); ++index) {
    const auto value = region.graph.registers[index];
    if (!value.valid() || (region.graph.changed_registers & (Word{1} << index)) == 0) { continue; }
    const auto result = guest(frame.results[index], value);
    if (index >= 4) {
      (index == 4 ? reg.a : reg.b) = result << padding_bits;
    } else {
      auto &bank = index < 2 ? reg.x : reg.y;
      const auto shift = (index & 1U) * word_bits;
      bank = (bank & ~(std::uint64_t{word_mask} << shift)) | ((result & word_mask) << shift);
    }
  }
  for (unsigned index = 0; index < agu_registers; ++index) {
    const auto &delta = region.graph.addresses[index];
    if (delta.units != 0 || delta.offsets != 0 || region.prefix[index]) {
      reg.r[index] = frame.addresses[index];
    }
  }
  const auto count = frame.iterations * region.graph.instructions;
  cpu.instructions = wrapped_add(cpu.instructions, count);
  cpu.cycles =
      wrapped_add(cpu.cycles, count + (std::uint64_t{frame.iterations} * region.extra_cycles));
  cpu.repeat_completed = false;
  cpu.interrupt_inhibited = false;
  if (reg.pc != region.entry) {
    context.boundary = static_cast<NativeBoundary>(static_cast<unsigned>(context.boundary) |
                                                   static_cast<unsigned>(NativeBoundary::context));
  }
}
} // namespace

std::uint32_t
run_region(Cpu *const cpu, NativeContext *const context, NativeRegion *const region) noexcept {
  if (!header(*cpu, *context, *region) || !memory(*cpu, *context, *region)) { return 0; }
  const auto count = std::min(cpu->reg.lc, context->budget / region->graph.instructions);
  region->lanes = 4;
  if (count < 4) { region->lanes = 2; }
  if (count < 2) { region->lanes = 1; }
  if (region->nested) {
    if ((count & 1U) != 0) { return 0; }
    region->lanes = count % 4 == 0 ? 4U : 2U;
  }
  region->frame.iterations = count - (count % region->lanes);
  if (region->frame.iterations == 0 || region->native[region->lanes / 2] == nullptr ||
      (region->frame.iterations == cpu->reg.lc && cpu->reg.sc < 2) ||
      (region->nested &&
       (region->frame.iterations != cpu->reg.lc || cpu->reg.sc >= stack_entries - 2)) ||
      !cursors(cpu->reg, *context, *region) || !streams(cpu->reg, *context, *region)) {
    return 0;
  }
  invariants(cpu->reg, *region);
  region->frame.constants = region->constants.data();
  region->frame.offsets = region->offsets.data();
  region->frame.spills = region->spills.data();
  if (!enter_region(region->native[region->lanes / 2], region->frame)) { return 0; }
  commit(*cpu, *context, *region);
  return region->frame.iterations * region->graph.instructions;
}
} // namespace dsp56300::detail
