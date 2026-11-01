// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace dsp56300 {
namespace {
constexpr Word stack_pointer_mask = 0x3f;
constexpr Word stack_counter_mask = 0x1f;

void validate(const Registers &state) {
  const std::array words{state.pc,
                         state.sr,
                         state.omr,
                         state.la,
                         state.lc,
                         state.sp,
                         state.sc,
                         state.sz,
                         state.ep,
                         state.vba};
  const auto invalid = [](const Word word) { return word > word_mask; };
  if (std::ranges::any_of(words, invalid) || std::ranges::any_of(state.r, invalid) ||
      std::ranges::any_of(state.n, invalid) || std::ranges::any_of(state.m, invalid) ||
      std::ranges::any_of(state.stack, [](const auto word) { return word > long_mask; }) ||
      state.x > long_mask || state.y > long_mask || state.a > accumulator_mask ||
      state.b > accumulator_mask || state.sp > stack_pointer_mask ||
      state.sc > stack_counter_mask) {
    throw std::invalid_argument("DSP register value exceeds its architectural width");
  }
}
}

Registers Core::registers() const {
  auto result = impl_->cpu.reg;
  result.a >>= detail::padding_bits;
  result.b >>= detail::padding_bits;
  return result;
}

void Core::set_registers(const Registers &state) {
  impl_->validate_staged_mutation();
  if (impl_->executing) {
    throw std::logic_error("Register initialization requires stopped DSP execution");
  }
  validate(state);
  impl_->last_outcome.reset();
  impl_->cpu.reg = state;
  impl_->cpu.reg.a <<= detail::padding_bits;
  impl_->cpu.reg.b <<= detail::padding_bits;
  impl_->synchronize_cache_mode();
}
} // namespace dsp56300
