// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace dsp56300::detail::a64 {
std::optional<std::uint64_t> RegionEmitter::fixed_stride(const unsigned stream) const {
  const auto root = region_.stream_roots[stream];
  const auto address = region_.roots[root].address;
  const auto &delta =
      inner_vector_ ? region_.inner_addresses[address] : region_.graph.addresses[address];
  if ((!inner_vector_ && region_.prefix[address]) || (delta.units == 0 && delta.offsets == 0)) {
    return 0;
  }
  if (delta.offsets == 0) { return std::uint64_t{delta.units} * sizeof(Word); }
  return std::nullopt;
}

std::optional<std::uint64_t> RegionEmitter::fixed_offset(const unsigned stream) const {
  const auto &input = region_.graph.streams[stream];
  if (input.offsets == 0) { return std::uint64_t{input.units} * sizeof(Word); }
  return std::nullopt;
}

void RegionEmitter::cache_strides() {
  const auto member =
      inner_vector_ ? offsetof(RegionCursor, inner_stride) : offsetof(RegionCursor, stride);
  for (unsigned stream = 0; stream < region_.graph.streams.size(); ++stream) {
    const auto root = region_.stream_roots[stream];
    if (root >= stride_registers.size() || (cached_strides_ & (Word{1} << root)) != 0 ||
        fixed_stride(stream)) {
      continue;
    }
    const auto target = stride_registers[root];
    code_.load(target, Reg::x0, cursor_offset(root, member), Access::doubleword);
    code_.shift(Shift::left, target, target, 2, Width::x64);
    cached_strides_ |= Word{1} << root;
  }
}

Reg RegionEmitter::stride(const unsigned stream) {
  if (const auto fixed = fixed_stride(stream)) {
    code_.constant(Reg::x11, *fixed);
    return Reg::x11;
  }
  const auto root = region_.stream_roots[stream];
  if ((cached_strides_ & (Word{1} << root)) != 0) { return stride_registers[root]; }
  const auto member =
      inner_vector_ ? offsetof(RegionCursor, inner_stride) : offsetof(RegionCursor, stride);
  code_.load(Reg::x11, Reg::x0, cursor_offset(root, member), Access::doubleword);
  code_.shift(Shift::left, Reg::x11, Reg::x11, 2, Width::x64);
  return Reg::x11;
}

void RegionEmitter::pointer(const StreamSlice slice) {
  const auto stream = slice.stream;
  const auto part = slice.part;
  stride_ = Reg::x11;
  const auto root = region_.stream_roots[stream];
  const auto base = root < root_registers.size() ? root_registers[root] : Reg::x10;
  if (base == Reg::x10) {
    code_.load(Reg::x10,
               Reg::x0,
               cursor_offset(root, offsetof(RegionCursor, pointer)),
               Access::doubleword);
  }
  const auto &address = region_.graph.streams[stream];
  if ((region_.reverse_addresses & (Word{1} << address.address)) != 0) {
    code_.move(Reg::x10, base);
    return;
  }
  const auto offset = fixed_offset(stream);
  constexpr std::uint64_t maximum_immediate = 4095;
  if (offset && *offset <= maximum_immediate) {
    code_.add(Reg::x10, base, static_cast<std::uint16_t>(*offset), Width::x64);
  } else {
    if (offset) {
      code_.constant(Reg::x9, *offset);
    } else {
      code_.load(Reg::x9, Reg::x0, offsetof(RegionFrame, offsets), Access::doubleword);
      code_.load(Reg::x9, Reg::x9, stream * sizeof(std::uint64_t), Access::doubleword);
    }
    code_.alu(Alu::add, Reg::x10, base, Reg::x9, Width::x64);
  }
  const auto fixed = fixed_stride(stream);
  if (lanes_ != 1 && (!fixed || (*fixed != 0 && *fixed != sizeof(Word)))) {
    stride_ = stride(stream);
  }
  if (part != 0) {
    if (fixed) {
      code_.constant(Reg::x9, *fixed * 2);
    } else {
      code_.shift(Shift::left, Reg::x9, stride_, 1, Width::x64);
    }
    code_.alu(Alu::add, Reg::x10, Reg::x10, Reg::x9, Width::x64);
  }
}

void RegionEmitter::reverse_load(const Vec target, const StreamSlice slice, const bool packed) {
  const auto stream = slice.stream;
  const auto part = slice.part;
  const auto &address = region_.graph.streams[stream];
  const auto base = offsetof(RegionFrame, reverse) + (address.address * sizeof(ReverseCursor));
  const unsigned count = packed ? lanes_ : std::min(lanes_, 2U);
  for (unsigned lane = 0; lane < count; ++lane) {
    code_.load(Reg::x9, Reg::x0, base, Access::doubleword);
    const auto displacement = (std::uint64_t{(packed ? 0U : 2 * part) + lane} *
                               region_.graph.addresses[address.address].offsets) +
                              address.offsets;
    code_.constant(Reg::x12, displacement);
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x12, Width::x64);
    code_.load(Reg::x12, Reg::x0, base + offsetof(ReverseCursor, mask));
    code_.alu(Alu::bit_and, Reg::x9, Reg::x9, Reg::x12);
    code_.reverse_bits(Reg::x9, Reg::x9);
    code_.load(Reg::x12, Reg::x0, base + offsetof(ReverseCursor, shift));
    code_.shift(Shift::right, Reg::x9, Reg::x9, Reg::x12);
    code_.shift(Shift::left, Reg::x9, Reg::x9, 2, Width::x64);
    code_.alu(Alu::add, Reg::x9, Reg::x10, Reg::x9, Width::x64);
    code_.load(Reg::x9, Reg::x9, 0);
    code_.extract(Reg::x9, Reg::x9, 0, word_bits, Width::x64, true);
    if (lane == 0) {
      neon_.broadcast(target, Reg::x9, packed);
    } else {
      neon_.insert(target, Reg::x9, lane, packed);
    }
  }
}

void RegionEmitter::load(const Vec target, const StreamSlice slice) {
  const auto stream = slice.stream;
  pointer(slice);
  const auto address = region_.graph.streams[stream].address;
  if ((region_.reverse_addresses & (Word{1} << address)) != 0) {
    reverse_load(target, slice);
    return;
  }
  const auto fixed = fixed_stride(stream);
  if (lanes_ == 1 || fixed == 0) {
    code_.load(Reg::x9, Reg::x10, 0);
    code_.extract(Reg::x9, Reg::x9, 0, word_bits, Width::x64, true);
    neon_.broadcast(target, Reg::x9);
    return;
  }
  Label strided;
  Label complete;
  if (!fixed) {
    code_.constant(Reg::x9, sizeof(Word));
    code_.compare(stride_, Reg::x9, Width::x64);
    code_.branch(Condition::not_equal, strided);
  }
  if (!fixed || fixed == sizeof(Word)) {
    neon_.load(target, Reg::x10, 0, true);
    neon_.shift(target, target, Shift::left, padding_bits, true);
    neon_.shift(target, target, Shift::arithmetic, padding_bits, true);
    neon_.extend(target, target);
    if (fixed) { return; }
    code_.branch(complete);
    code_.bind(strided);
  }
  code_.load(Reg::x9, Reg::x10, 0);
  code_.extract(Reg::x9, Reg::x9, 0, word_bits, Width::x64, true);
  neon_.broadcast(target, Reg::x9);
  code_.alu(Alu::add, Reg::x12, Reg::x10, stride_, Width::x64);
  code_.load(Reg::x9, Reg::x12, 0);
  code_.extract(Reg::x9, Reg::x9, 0, word_bits, Width::x64, true);
  neon_.insert(target, Reg::x9, 1);
  code_.bind(complete);
}

void RegionEmitter::store(const GraphValue value, const StreamSlice slice) {
  const auto part = slice.part;
  const auto source = read(value, part);
  pointer(slice);
  Label strided;
  Label complete;
  const auto fixed = fixed_stride(slice.stream);
  if (lanes_ != 1 && fixed == sizeof(Word)) {
    store_packed(source);
    return;
  }
  if (lanes_ != 1 && !fixed) {
    code_.constant(Reg::x9, sizeof(Word));
    code_.compare(stride_, Reg::x9, Width::x64);
    code_.branch(Condition::not_equal, strided);
    store_packed(source);
    code_.branch(complete);
    code_.bind(strided);
  }
  if (lanes_ != 1 && fixed == 0) { code_.constant(Reg::x11, 0); }
  neon_.extract(Reg::x9, source, 0);
  code_.extract(Reg::x9, Reg::x9, 0, word_bits);
  code_.store(Reg::x9, Reg::x10, 0);
  if (lanes_ != 1) {
    code_.alu(Alu::add, Reg::x12, Reg::x10, stride_, Width::x64);
    neon_.extract(Reg::x9, source, 1);
    code_.extract(Reg::x9, Reg::x9, 0, word_bits);
    code_.store(Reg::x9, Reg::x12, 0);
  }
  code_.bind(complete);
}

void RegionEmitter::store_packed(const Vec source) {
  neon_.narrow(first_scratch, source);
  neon_.shift(first_scratch, first_scratch, Shift::left, padding_bits, true);
  neon_.shift(first_scratch, first_scratch, Shift::right, padding_bits, true);
  neon_.store(first_scratch, Reg::x10, 0, true);
}

void RegionEmitter::advance_reverse() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    if ((region_.reverse_addresses & (Word{1} << index)) == 0) { continue; }
    const auto base = offsetof(RegionFrame, reverse) + (index * sizeof(ReverseCursor));
    code_.load(Reg::x9, Reg::x0, base, Access::doubleword);
    code_.constant(Reg::x10, std::uint64_t{region_.graph.addresses[index].offsets} * lanes_);
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x10, Width::x64);
    code_.load(Reg::x10, Reg::x0, base + offsetof(ReverseCursor, mask));
    code_.alu(Alu::bit_and, Reg::x9, Reg::x9, Reg::x10);
    code_.store(Reg::x9, Reg::x0, base, Access::doubleword);
  }
}

void RegionEmitter::step(const std::size_t root, const bool inner) {
  const auto address = region_.roots[root].address;
  const auto &delta = inner ? region_.inner_addresses[address] : region_.graph.addresses[address];
  if (delta.offsets == 0 && (inner || !region_.nested)) {
    const auto bytes =
        std::uint64_t{delta.units} * sizeof(Word) * (inner && !inner_vector_ ? 1U : lanes_);
    if (bytes == 0) { return; }
    code_.constant(Reg::x9, bytes);
  } else {
    const auto member =
        inner ? offsetof(RegionCursor, inner_step_bytes) : offsetof(RegionCursor, step_bytes);
    code_.load(Reg::x9, Reg::x0, cursor_offset(root, member), Access::doubleword);
    if (inner && inner_vector_) { code_.shift(Shift::left, Reg::x9, Reg::x9, 2, Width::x64); }
  }
  if (root < root_registers.size()) {
    code_.alu(Alu::add, root_registers[root], root_registers[root], Reg::x9, Width::x64);
  } else {
    const auto offset = cursor_offset(root, offsetof(RegionCursor, pointer));
    code_.load(Reg::x10, Reg::x0, offset, Access::doubleword);
    code_.alu(Alu::add, Reg::x10, Reg::x10, Reg::x9, Width::x64);
    code_.store(Reg::x10, Reg::x0, offset, Access::doubleword);
  }
}

void RegionEmitter::advance(const bool inner) {
  if (!inner) { advance_reverse(); }
  for (std::size_t root = 0; root < region_.roots.size(); ++root) {
    const auto address = region_.roots[root].address;
    if ((inner && !region_.nested) || (region_.reverse_addresses & (Word{1} << address)) != 0) {
      continue;
    }
    step(root, inner);
  }
}
}
