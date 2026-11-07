// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "jit/mac_batch.hpp"

#include <algorithm>
#include <arm_neon.h>
#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
bool native_mac_available() noexcept {
  return true;
}

namespace {
int32x4_t operands(const Word *const words) noexcept {
  return vshrq_n_s32(vreinterpretq_s32_u32(vshlq_n_u32(vld1q_u32(words), padding_bits)),
                     padding_bits);
}

int32x4_t first_operands(const Word *const words, const Word old) noexcept {
  const auto retained =
      vshrq_n_s32(vreinterpretq_s32_u32(vshlq_n_u32(vdupq_n_u32(old), padding_bits)), padding_bits);
  return vextq_s32(retained, operands(words), 3);
}

void reduce(MacBatchState &state, const MacBatchInput &input, const unsigned count) noexcept {
  const auto x = first_operands(input.x.data(), state.x);
  const auto y = first_operands(input.y.data(), state.y);
  auto low = vmull_s32(vget_low_s32(x), vget_low_s32(y));
  auto high = vmull_high_s32(x, y);
  for (unsigned offset = mac_batch_width; offset < count; offset += mac_batch_width) {
    const auto next_x = operands(input.x.data() + offset - 1);
    const auto next_y = operands(input.y.data() + offset - 1);
    low = vmlal_s32(low, vget_low_s32(next_x), vget_low_s32(next_y));
    high = vmlal_high_s32(high, next_x, next_y);
  }
  const auto delta = 2 * (vaddvq_s64(low) + vaddvq_s64(high));
  state.accumulator += input.subtract ? -delta : delta;
  state.overflowed = false;
}

void prefixes(MacBatchState &state, const MacBatchInput &input) noexcept {
  for (std::size_t index = 0; index < input.x.size(); index += mac_batch_width) {
    const auto x = first_operands(input.x.data() + index, state.x);
    const auto y = first_operands(input.y.data() + index, state.y);
    const auto low = vshlq_n_s64(vmull_s32(vget_low_s32(x), vget_low_s32(y)), 1);
    const auto high = vshlq_n_s64(vmull_high_s32(x, y), 1);
    std::array<std::int64_t, mac_batch_width> products{};
    vst1q_s64(products.data(), low);
    vst1q_s64(products.data() + 2, high);
    consume_mac_products(state, products, input.subtract);
    state.x = input.x[index + mac_batch_width - 1] & word_mask;
    state.y = input.y[index + mac_batch_width - 1] & word_mask;
  }
}
}

void native_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept {
  for (std::size_t index = 0; index < input.x.size();) {
    auto count = mac_batch_math::reduction_width(state.accumulator, input.x.size() - index);
    if (count != mac_batch_width) {
      reduce(state,
             {.x = input.x.subspan(index, count),
              .y = input.y.subspan(index, count),
              .subtract = input.subtract},
             count);
    } else {
      count =
          static_cast<unsigned>(std::min<std::size_t>(mac_reduction_width, input.x.size() - index));
      prefixes(state,
               {.x = input.x.subspan(index, count),
                .y = input.y.subspan(index, count),
                .subtract = input.subtract});
    }
    index += count;
    state.x = input.x[index - 1] & word_mask;
    state.y = input.y[index - 1] & word_mask;
  }
}
}
