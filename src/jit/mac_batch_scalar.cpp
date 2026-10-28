// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "mac_batch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
void scalar_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept {
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
} // namespace dsp56300::detail
