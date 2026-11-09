// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/value_graph.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void RegionEmitter::load_words(const Vec target, const unsigned stream) {
  pointer({.stream = stream, .part = 0});
  const auto address = region_.graph.streams[stream].address;
  if ((region_.reverse_addresses & (Word{1} << address)) != 0) {
    reverse_load(target, {.stream = stream, .part = 0}, true);
    return;
  }
  const auto fixed = fixed_stride(stream);
  if (lanes_ == 1 || fixed == 0) {
    code_.load(Reg::x9, Reg::x10, 0);
    code_.extract(Reg::x9, Reg::x9, 0, word_bits, Width::w32, true);
    neon_.broadcast(target, Reg::x9, true);
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
    neon_.load(target, Reg::x10, 0, lanes_ == 2);
    if (fixed) {
      neon_.shift(target, target, Shift::left, padding_bits, true);
      neon_.shift(target, target, Shift::arithmetic, padding_bits, true);
      return;
    }
    code_.branch(complete);
    code_.bind(strided);
  }
  code_.move(Reg::x12, Reg::x10);
  for (unsigned lane = 0; lane < lanes_; ++lane) {
    code_.load(Reg::x9, Reg::x12, 0);
    neon_.insert(target, Reg::x9, lane, true);
    if (lane + 1 != lanes_) { code_.alu(Alu::add, Reg::x12, Reg::x12, stride_, Width::x64); }
  }
  code_.bind(complete);
  neon_.shift(target, target, Shift::left, padding_bits, true);
  neon_.shift(target, target, Shift::arithmetic, padding_bits, true);
}

void RegionEmitter::store_words(const GraphValue value, const unsigned stream) {
  const auto source = read(value);
  pointer({.stream = stream, .part = 0});
  const auto fixed = fixed_stride(stream);
  Label strided;
  Label complete;
  if (lanes_ != 1 && !fixed) {
    code_.constant(Reg::x9, sizeof(Word));
    code_.compare(stride_, Reg::x9, Width::x64);
    code_.branch(Condition::not_equal, strided);
  }
  if (lanes_ != 1 && (!fixed || fixed == sizeof(Word))) {
    neon_.shift(first_scratch, source, Shift::left, padding_bits, true);
    neon_.shift(first_scratch, first_scratch, Shift::right, padding_bits, true);
    neon_.store(first_scratch, Reg::x10, 0, lanes_ == 2);
    if (fixed) { return; }
    code_.branch(complete);
    code_.bind(strided);
  }
  if (lanes_ != 1 && fixed == 0) { code_.constant(Reg::x11, 0); }
  code_.move(Reg::x12, Reg::x10);
  for (unsigned lane = 0; lane < lanes_; ++lane) {
    neon_.extract(Reg::x9, source, lane, true);
    code_.extract(Reg::x9, Reg::x9, 0, word_bits);
    code_.store(Reg::x9, Reg::x12, 0);
    if (lane + 1 != lanes_) { code_.alu(Alu::add, Reg::x12, Reg::x12, stride_, Width::x64); }
  }
  code_.bind(complete);
}
}
