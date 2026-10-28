// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "mac_batch.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
void reduce(MacBatchState &state, const MacBatchInput &input, const unsigned count) noexcept {
  std::array<std::int64_t, mac_batch_width> sums{
      std::int64_t{signed_word(state.x)} * signed_word(state.y),
      std::int64_t{signed_word(input.x[0])} * signed_word(input.y[0]),
      std::int64_t{signed_word(input.x[1])} * signed_word(input.y[1]),
      std::int64_t{signed_word(input.x[2])} * signed_word(input.y[2])};
  for (unsigned offset = mac_batch_width; offset < count; offset += mac_batch_width) {
    for (unsigned lane = 0; lane < mac_batch_width; ++lane) {
      sums[lane] += std::int64_t{signed_word(input.x[offset + lane - 1])} *
                    signed_word(input.y[offset + lane - 1]);
    }
  }
  const auto delta = 2 * (sums[0] + sums[1] + sums[2] + sums[3]);
  state.accumulator += input.subtract ? -delta : delta;
  state.overflowed = false;
}

void prefixes(MacBatchState &state, const MacBatchInput &input) noexcept {
  auto current = state;
  const auto streams = input;
  for (std::size_t index = 0; index < streams.x.size(); index += mac_batch_width) {
    const std::array products{
        2 * std::int64_t{signed_word(current.x)} * signed_word(current.y),
        2 * std::int64_t{signed_word(streams.x[index])} * signed_word(streams.y[index]),
        2 * std::int64_t{signed_word(streams.x[index + 1])} * signed_word(streams.y[index + 1]),
        2 * std::int64_t{signed_word(streams.x[index + 2])} * signed_word(streams.y[index + 2])};
    consume_mac_products(current, products, streams.subtract);
    current.x = streams.x[index + 3] & word_mask;
    current.y = streams.y[index + 3] & word_mask;
  }
  state = current;
}
}

void scalar_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept {
  auto current = state;
  const auto streams = input;
  for (std::size_t index = 0; index < streams.x.size();) {
    auto count = mac_batch_math::reduction_width(current.accumulator, streams.x.size() - index);
    if (count != mac_batch_width) {
      reduce(current,
             {.x = streams.x.subspan(index, count),
              .y = streams.y.subspan(index, count),
              .subtract = streams.subtract},
             count);
    } else {
      count = static_cast<unsigned>(
          std::min<std::size_t>(mac_reduction_width, streams.x.size() - index));
      prefixes(current,
               {.x = streams.x.subspan(index, count),
                .y = streams.y.subspan(index, count),
                .subtract = streams.subtract});
    }
    index += count;
    current.x = streams.x[index - 1] & word_mask;
    current.y = streams.y[index - 1] & word_mask;
  }
  state = current;
}
} // namespace dsp56300::detail
