// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "resident_region.hpp"
#include "value_graph.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

bool header(const Cpu &cpu, const NativeContext &context, const ResidentRegion &region) {
  const auto &reg = cpu.reg;
  return reg.pc == region.entry && reg.la == region.last && (reg.sr & loop_flag) != 0 &&
         (reg.sr & forever_flag) == 0 && (reg.sr & numeric_mode_mask) == region.mode &&
         (!context.runtime.cacheable_program || (reg.sr & cache_enable) == 0) &&
         cpu.repeat_remaining == 0 && cpu.processing == ExecutionState::running &&
         cpu.debug_flags == 0 && reg.lc != 0 && reg.lc <= word_mask &&
         (reg.stack[reg.sp & (stack_entries - 1)] >> word_bits) == region.entry &&
         (context.exit_requested == nullptr || !*context.exit_requested);
}

bool host_storage(const Cpu &cpu,
                  const NativeContext &context,
                  const std::uintptr_t base,
                  const std::size_t bytes) {
  return !overlap(base, bytes, reinterpret_cast<std::uintptr_t>(&cpu), sizeof(cpu)) &&
         !overlap(base, bytes, reinterpret_cast<std::uintptr_t>(&context), sizeof(context));
}

bool select_memory(const Cpu &cpu,
                   const NativeContext &context,
                   ResidentRegion &region,
                   const RegionRoot &root) {
  const auto index = root.space == Space::x ? 0U : 1U;
  auto &view = region.memory[index];
  const auto windows = context.runtime.windows[index];
  const auto first = region.prefix[root.address].value_or(cpu.reg.r[root.address]);
  // The owner retains descriptors until a mapping revision leaves this entry.
  // Continuing packets inside the same grant need no descriptor copy or search.
  if (!view.words.empty() && first >= view.address && first - view.address < view.words.size()) {
    return true;
  }
  if (windows.empty()) {
    const auto &ram = context.runtime.memory[index];
    if (ram.data == nullptr) { return false; }
    view = {.words = {ram.data, std::min(ram.words, std::size_t{io_begin})},
            .known = {},
            .writable = true,
            .aliases_program = ram.aliases_program};
  } else {
    const auto found = std::ranges::find_if(windows, [first](const auto &window) {
      return first >= window.address && first - window.address < window.words.size();
    });
    if (found == windows.end()) { return false; }
    view = *found;
  }
  const bool valid =
      host_storage(cpu,
                   context,
                   reinterpret_cast<std::uintptr_t>(view.words.data()),
                   view.words.size_bytes()) &&
      (view.known.empty() || host_storage(cpu,
                                          context,
                                          reinterpret_cast<std::uintptr_t>(view.known.data()),
                                          view.known.size_bytes()));
  if (!valid) { view = {}; }
  return valid;
}

bool memory(const Cpu &cpu, const NativeContext &context, ResidentRegion &region) {
  std::array<bool, 2> seen{};
  for (const auto &root : region.roots) {
    const auto index = root.space == Space::x ? 0U : 1U;
    if (seen[index]) { continue; }
    seen[index] = true;
    if (!select_memory(cpu, context, region, root)) { return false; }
  }
  for (const auto &stream : region.graph.streams) {
    const auto &view = region.memory[stream.space == Space::x ? 0 : 1];
    if (stream.write && (!view.writable || view.aliases_program)) { return false; }
  }
  return true;
}

Word reverse_bits(Word value) {
  constexpr Word alternating = 0x55555555;
  constexpr Word pairs = 0x33333333;
  constexpr Word nibbles = 0x0f0f0f0f;
  value = ((value >> 1) & alternating) | ((value & alternating) << 1);
  value = ((value >> 2) & pairs) | ((value & pairs) << 2);
  value = ((value >> 4) & nibbles) | ((value & nibbles) << 4);
  return std::byteswap(value);
}

bool reverse_address(const Registers &reg, ResidentRegion &region, const unsigned index) {
  const auto initial = reg.r[index];
  const auto n = reg.n[index];
  if ((reg.m[index] & modifier_mask) != 0 || initial > word_mask || n > sign24_bit ||
      !std::has_single_bit(n)) {
    return false;
  }
  auto &cursor = region.frame.reverse[index];
  cursor.mask = (n * 2) - 1;
  cursor.shift = static_cast<Word>(std::numeric_limits<Word>::digits - std::bit_width(n));
  cursor.index = reverse_bits(initial & cursor.mask) >> cursor.shift;
  const auto distance =
      std::uint64_t{region.graph.addresses[index].offsets} * region.frame.iterations;
  const auto final = static_cast<Word>(cursor.index + distance) & cursor.mask;
  region.frame.addresses[index] = (initial & ~cursor.mask) | (reverse_bits(final) >> cursor.shift);
  return true;
}

Word iteration_limit(const Registers &reg, const ResidentRegion &region, const unsigned index) {
  const auto modifier = reg.m[index] & modifier_mask;
  if (region.prefix[index] || modifier == 0 || modifier >= multiple_wrap_bit) { return word_mask; }
  const auto &delta = region.graph.addresses[index];
  const auto n = delta.offsets != 0 ? reg.n[index] : 0;
  std::uint64_t step = 0;
  if (n >= sign24_bit || !scaled_offset(delta, n, step)) { return 0; }
  if (step == 0) { return word_mask; }
  const auto initial = reg.r[index];
  const auto begin = initial & ~(std::bit_ceil(modifier + 1) - 1);
  const auto limit = begin + modifier + 1;
  if (initial >= limit) { return 0; }
  const auto room = limit - initial;
  auto count = static_cast<Word>(room / step);
  if (count == 0 || room % step != 0) { return count; }
  for (const auto &stream : region.graph.streams) {
    if (stream.address != index) { continue; }
    std::uint64_t offset = 0;
    if (!scaled_offset({.units = stream.units, .offsets = stream.offsets}, n, offset)) { return 0; }
    if (offset == step) { return count - 1; }
  }
  return count;
}

Word bounded_iterations(const Registers &reg, const ResidentRegion &region, Word count) {
  if (region.nested) { return count; }
  for (unsigned index = 0; index < agu_registers; ++index) {
    count = std::min(count, iteration_limit(reg, region, index));
  }
  return count;
}

bool address(const Registers &reg, ResidentRegion &region, const unsigned index) {
  if ((region.reverse_addresses & (Word{1} << index)) != 0) {
    return reverse_address(reg, region, index);
  }
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
  auto final = initial + distance;
  const auto modifier = reg.m[index] & modifier_mask;
  if (step != 0 && modifier != modifier_mask) {
    if (modifier == 0 || modifier >= multiple_wrap_bit) { return false; }
    const auto begin = initial & ~(std::bit_ceil(modifier + 1) - 1);
    const auto limit = begin + modifier + 1;
    if (initial >= limit || final > limit) { return false; }
    if (final == limit) {
      if (region.nested || region.prefix[index]) { return false; }
      final = begin;
    }
  }
  region.frame.addresses[index] = static_cast<Word>(final);
  return true;
}

bool cursors(const Registers &reg, ResidentRegion &region) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    if (!address(reg, region, index)) { return false; }
  }
  for (std::size_t index = 0; index < region.roots.size(); ++index) {
    const auto &root = region.roots[index];
    auto &cursor = region.frame.cursors[index];
    const auto first = region.prefix[root.address].value_or(reg.r[root.address]);
    if ((region.reverse_addresses & (Word{1} << root.address)) != 0) {
      const auto base = first & ~region.frame.reverse[root.address].mask;
      const auto &ram = region.memory[root.space == Space::x ? 0 : 1];
      if (base < ram.address || base - ram.address >= ram.words.size()) { return false; }
      cursor = {.pointer = ram.words.data() + (base - ram.address)};
      continue;
    }
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
    const auto &ram = region.memory[root.space == Space::x ? 0 : 1];
    if (first < ram.address || first - ram.address >= ram.words.size()) { return false; }
    cursor.pointer = ram.words.data() + (first - ram.address);
  }
  return true;
}

bool initialized(const MemoryWindow &view,
                 const Word first,
                 const std::uint64_t last,
                 const bool write) {
  if (first < view.address || last >= view.address + view.words.size()) { return false; }
  if (view.known.empty()) { return true; }
  // Writes to already canonical knowledge bytes leave metadata unchanged. Other
  // writes retain ordered scalar stores, including first initialization.
  auto knowledge =
      view.known.subspan(first - view.address, static_cast<std::size_t>(last - first + 1));
  if (!write) { return std::memchr(knowledge.data(), 0, knowledge.size()) == nullptr; }
  static constexpr auto canonical = [] {
    std::array<std::uint8_t, 64> result{};
    result.fill(1);
    return result;
  }();
  while (knowledge.size() > canonical.size()) {
    if (std::memcmp(knowledge.data(), canonical.data(), canonical.size()) != 0) { return false; }
    knowledge = knowledge.subspan(canonical.size());
  }
  return std::memcmp(knowledge.data(), canonical.data(), knowledge.size()) == 0;
}

bool footprint(const Registers &reg,
               ResidentRegion &region,
               const std::size_t index,
               RegionFootprint &result) {
  const auto &stream = region.graph.streams[index];
  const auto &cursor = region.frame.cursors[region.stream_roots[index]];
  const auto &ram = region.memory[stream.space == Space::x ? 0 : 1];
  const auto first = region.prefix[stream.address].value_or(reg.r[stream.address]);
  if ((region.reverse_addresses & (Word{1} << stream.address)) != 0) {
    const auto mask = region.frame.reverse[stream.address].mask;
    const auto base = first & ~mask;
    if (stream.write || !initialized(ram, base, std::uint64_t{base} + mask, false)) {
      return false;
    }
    region.offsets[index] = 0;
    result = {.first = reinterpret_cast<std::uintptr_t>(cursor.pointer),
              .bytes = (std::uint64_t{mask} + 1) * sizeof(Word)};
    return true;
  }
  std::uint64_t displacement = 0;
  if (!scaled_offset({.units = stream.units, .offsets = stream.offsets},
                     reg.n[stream.address],
                     displacement)) {
    return false;
  }
  const auto span = (region.inner_iterations - 1) * cursor.inner_stride;
  const auto end = first + displacement + span + ((region.frame.iterations - 1) * cursor.stride);
  if (!initialized(ram, first, end, stream.write) || (stream.write && cursor.stride == 0)) {
    return false;
  }
  region.offsets[index] = displacement * sizeof(Word);
  result = {.first = reinterpret_cast<std::uintptr_t>(cursor.pointer + displacement),
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

bool streams(const Registers &reg, ResidentRegion &region) {
  // Scratch storage is sized during translation, never allocated by execution.
  for (std::size_t index = 0; index < region.graph.streams.size(); ++index) {
    auto &current = region.footprints[index];
    if (!footprint(reg, region, index, current)) { return false; }
    if (region.graph.streams[index].write) {
      for (const auto &view : region.memory) {
        if (!view.known.empty() && overlap(current.first,
                                           current.bytes,
                                           reinterpret_cast<std::uintptr_t>(view.known.data()),
                                           view.known.size_bytes())) {
          return false;
        }
      }
    }
    for (std::size_t other = 0; other < index; ++other) {
      if (!region.graph.streams[index].write && !region.graph.streams[other].write) { continue; }
      if (!independent(current, region.footprints[other])) { return false; }
    }
  }
  return true;
}

bool admit_inner(const ResidentRegion &region, const bool parent_available) {
  std::size_t inner_cost = 0;
  std::size_t parent_cost = 0;
  for (std::size_t index = 0; index < region.graph.streams.size(); ++index) {
    const auto &cursor = region.frame.cursors[region.stream_roots[index]];
    inner_cost += cursor.inner_stride <= 1 ? 1U : 4U;
    parent_cost += cursor.stride <= 1 ? 1U : 4U;
  }
  // Prefer the existing parent kernel when changing axes saves no memory work.
  if (parent_available && inner_cost >= parent_cost) { return false; }
  for (std::size_t index = 0; index < region.graph.streams.size(); ++index) {
    const auto &stream = region.graph.streams[index];
    const auto &cursor = region.frame.cursors[region.stream_roots[index]];
    if (stream.write && cursor.inner_stride == 0) { return false; }
    auto left = region.footprints[index];
    left.bytes = left.inner_span + sizeof(Word);
    left.stride = cursor.inner_stride * sizeof(Word);
    left.inner_span = 0;
    for (std::size_t other = 0; other < index; ++other) {
      if (!stream.write && !region.graph.streams[other].write) { continue; }
      auto right = region.footprints[other];
      right.bytes = right.inner_span + sizeof(Word);
      right.stride = region.frame.cursors[region.stream_roots[other]].inner_stride * sizeof(Word);
      right.inner_span = 0;
      if (!independent(left, right)) { return false; }
    }
  }
  return true;
}

bool select_lanes(ResidentRegion &region, const Word count) {
  region.lanes = 4;
  if (count < 4) { region.lanes = 2; }
  if (count < 2) { region.lanes = 1; }
  if (region.nested) {
    if (region.native[0] != nullptr && count >= 2) {
      region.lanes = 1;
    } else {
      if ((count & 1U) != 0) { return false; }
      region.lanes = count % 4 == 0 ? 4U : 2U;
    }
  }
  return true;
}

void invariants(const Registers &reg, ResidentRegion &region) {
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

Word flags(const std::array<std::uint64_t, 4> &values, const ValueGraph &graph) {
  Word result = 0;
  if (graph.flags.valid()) {
    const auto value = guest(values[0], graph.flags);
    if (value == 0) { result |= zero; }
    if (graph.overflow_input.valid() && guest(values[3], graph.overflow_input) != value) {
      result |= overflow;
    }
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

Word sticky(const ResidentRegion &region,
            const RegionVector &growth_events,
            const RegionVector &limit_events,
            const unsigned lanes) {
  std::uint64_t changed = 0;
  std::uint64_t limited = 0;
  for (unsigned index = 0; index < lanes; ++index) {
    changed |= growth_events.words[index];
    limited |= limit_events.words[index];
  }
  return (((changed >> region.growth_shift) & 1) != 0 ? growth : 0U) |
         ((limited & std::numeric_limits<Word>::max()) != 0 ? limit : 0U);
}

void stack(Registers &reg, const ResidentRegion &region, const Word changed) {
  const auto &frame = region.frame;
  if (region.nested) {
    const auto preceding = sticky(region, frame.preceding_growth, frame.preceding_limits, 4) |
                           sticky(region, frame.growth, frame.limits, region.lanes - 1);
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

void commit(Cpu &cpu, NativeContext &context, const ResidentRegion &region) {
  auto &reg = cpu.reg;
  const auto &frame = region.frame;
  Word changed = region.graph.flags.valid() ? arithmetic_status : 0;
  if (region.graph.carry != GraphCarry::unchanged) { changed |= carry; }
  // Inner SSL is the state before the final iteration, before publishing its flags.
  reg.lc -= frame.iterations;
  stack(reg, region, changed);
  const auto loop_bits = reg.sr & loop_status;
  reg.sr = (reg.sr & ~changed) | sticky(region, frame.growth, frame.limits, 4) |
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

std::uint32_t run_resident_region(Cpu *const cpu,
                                  NativeContext *const context,
                                  ResidentRegion *const region) noexcept {
  if (!header(*cpu, *context, *region) || !memory(*cpu, *context, *region)) { return 0; }
  const auto count = bounded_iterations(
      cpu->reg, *region, std::min(cpu->reg.lc, context->budget / region->graph.instructions));
  if (!select_lanes(*region, count)) { return 0; }
  region->frame.iterations = count - (count % region->lanes);
  if (region->frame.iterations == 0 || region->native[region->lanes / 2] == nullptr ||
      (region->frame.iterations == cpu->reg.lc && cpu->reg.sc < 2) ||
      (region->nested &&
       (region->frame.iterations != cpu->reg.lc || cpu->reg.sc >= stack_entries - 2)) ||
      !cursors(cpu->reg, *region) || !streams(cpu->reg, *region)) {
    return 0;
  }
  if (region->nested && region->lanes == 1) {
    const Word parent_lanes = count % 4 == 0 ? 4U : 2U;
    const bool parent_available = (count & 1U) == 0 && region->native[parent_lanes / 2] != nullptr;
    if (!admit_inner(*region, parent_available)) {
      // A child recurrence may still be independent across separate parents.
      if (!parent_available) { return 0; }
      region->lanes = parent_lanes;
      if (!cursors(cpu->reg, *region)) { return 0; }
    }
  }
  invariants(cpu->reg, *region);
  region->frame.constants = region->constants.data();
  region->frame.offsets = region->offsets.data();
  region->frame.spills = region->spills.data();
  if (!enter_resident_region(region->native[region->lanes / 2], region->frame)) { return 0; }
  commit(*cpu, *context, *region);
  return region->frame.iterations * region->graph.instructions;
}
} // namespace dsp56300::detail
